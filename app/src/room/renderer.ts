/* The room in three.js. Triangle edges come from barycentrics in the fragment shader, so every
 * seam between triangles shows.
 */
import * as THREE from 'three';
import {KINDS, type Kind} from '../core/fixtures';
import {build, buildGround, release, type GroundMark, type Marks} from './marks';

const COLOUR: Record<Kind, number> = {ground: 0x2f4f6f, wall: 0x3d3f52, roof: 0x4a3550};
const EDGE: Record<Kind, number> = {ground: 0x7fb2e5, wall: 0x9aa0c0, roof: 0xbb8fd0};

/* The panel's `--sunken`: clear colour, and what faint surfaces are mixed towards. */
const BACK = 0x0a0e13;

/* Fill and edge opacity when a kind stops Link and when it does not; the off state is also dashed. */
const SHADE: Record<Kind, {on: [number, number]; off: [number, number]}> = {
  ground: {on: [1, 1], off: [0.3, 0.45]},
  wall: {on: [0.9, 1], off: [0.2, 0.5]},
  roof: {on: [0.9, 1], off: [0.2, 0.5]},
};

/** Screen pixels. */
const DASH_ON = 5, DASH_OFF = 4;

const EDGE_VERT = /* glsl */ `
  attribute vec3 bary;
  varying vec3 vBary;
  varying vec3 vNormal;
  void main(){
    vBary = bary;
    vNormal = normalize(normalMatrix * normal);
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }`;
const EDGE_FRAG = /* glsl */ `
  precision highp float;
  uniform vec3 fill;
  uniform vec3 edge;
  uniform vec3 back;
  uniform float showEdges;
  uniform float fillAlpha;
  uniform float edgeAlpha;
  uniform float dashed;
  uniform float dashOn;
  uniform float dashOff;
  varying vec3 vBary;
  varying vec3 vNormal;
  void main(){
    vec3 n = gl_FrontFacing ? vNormal : -vNormal;
    float lit = 0.55 + 0.60 * max(dot(n, normalize(vec3(0.4, 1.0, 0.25))), 0.0);
    /* Faint means mixed towards what is behind it, because this draws opaque. */
    vec3 c = mix(back, fill * lit, fillAlpha);
    /* EVERY DERIVATIVE IS TAKEN BEFORE ANY BRANCH. A derivative inside non-uniform control flow is
       undefined, and the nearest edge differs fragment to fragment inside one quad. */
    vec3 d = fwidth(vBary);
    vec3 grad = vec3(length(vec2(dFdx(vBary.x), dFdy(vBary.x))),
                     length(vec2(dFdx(vBary.y), dFdy(vBary.y))),
                     length(vec2(dFdx(vBary.z), dFdy(vBary.z))));
    if (showEdges > 0.5){
      vec3 a = smoothstep(vec3(0.0), d * 1.25, vBary);
      float cover = 1.0 - min(min(a.x, a.y), a.z);
      if (dashed > 0.5){
        /* A DASH IS MEASURED ALONG THE EDGE, IN PIXELS, or it breathes with the zoom. The nearest
           edge is the smallest barycentric; one of the other two runs 0..1 along it, and its own
           screen gradient turns that into a length: 1 / |grad| is how many pixels the whole edge
           is, so t / |grad| is how far along it this fragment sits. */
        float t = vBary.x <= vBary.y && vBary.x <= vBary.z ? vBary.y
                : vBary.y <= vBary.z ? vBary.z : vBary.x;
        float g = vBary.x <= vBary.y && vBary.x <= vBary.z ? grad.y
                : vBary.y <= vBary.z ? grad.z : grad.x;
        float along = g > 0.0 ? t / g : 0.0;
        float period = dashOn + dashOff;
        cover *= step(fract(along / period) * period, dashOn);
      }
      c = mix(c, mix(back, edge, edgeAlpha), cover);
    }
    gl_FragColor = vec4(c, 1.0);
  }`;

function baryFor(triangles: number): THREE.BufferAttribute {
  const a = new Float32Array(triangles * 9);
  for (let i = 0; i < triangles; i++) {
    a[i * 9 + 0] = 1;
    a[i * 9 + 4] = 1;
    a[i * 9 + 8] = 1;
  }
  return new THREE.BufferAttribute(a, 3);
}

/** A floor point, full precision; `px`/`py` are panel pixels. */
export interface Point {x: number; y: number; z: number; px: number; py: number}

/** `zoom` is relative to the initial framing (1 = as framed). */
export interface Cam {yaw: number; zoom: number}

/** Nine floats a triangle, per collision kind. */
export type Parts = Record<Kind, Float32Array> & {
  /** Drawn as floor, never given to the marks to stand on. */
  seabed?: Float32Array;
};

export interface RoomView {
  show(parts: Parts, roofs: boolean): void;
  marks(m: Marks): void;
  /** Which kinds stop Link; the rest are drawn faint and dashed. */
  solid(on: Record<Kind, boolean>): void;
  zoom(by: number): void;
  reset(): void;
  /** Floor height under a spot by a vertical ray against floors only, `null` where there is none. */
  groundAt(x: number, z: number): number | null;
  dispose(): void;
}

/**
 * @param onPoint the floor point under the cursor, `null` when there is none.
 * @param onMenu the floor point of a right-click, `null` when it missed the floor.
 */
export function mount(host: HTMLElement, onPoint: (p: Point | null) => void = () => {},
                      onCam: (c: Cam) => void = () => {},
                      onMenu: (p: Point | null) => void = () => {}): RoomView {
  /* Off, or the custom shader gets linear colours and the stylesheet's come out near-black. */
  THREE.ColorManagement.enabled = false;
  const renderer = new THREE.WebGLRenderer({antialias: true, powerPreference: 'high-performance'});
  renderer.outputColorSpace = THREE.LinearSRGBColorSpace;
  /* The check reads a `null` info log after a lost GPU context and throws. */
  renderer.debug.checkShaderErrors = false;
  renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
  host.appendChild(renderer.domElement);

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(BACK);
  const camera = new THREE.PerspectiveCamera(50, 1, 1, 200000);

  const home = {yaw: 0.62, tilt: 0.86};
  const orbit = {yaw: home.yaw, tilt: home.tilt, dist: 3000, target: new THREE.Vector3()};
  let framed = {dist: 3000, target: new THREE.Vector3()};

  /* Rescales marks carrying `userData.px` to that many screen pixels. */
  const world = new THREE.Vector3();
  function sizeMarks(): void {
    if (!marked) return;
    const h = renderer.domElement.clientHeight || 1;
    const perPixel = (2 * Math.tan((camera.fov * Math.PI) / 360)) / h;
    marked.traverse((o: THREE.Object3D) => {
      const px = o.userData.px as number | undefined;
      if (!px) return;
      o.scale.setScalar(px * perPixel * camera.position.distanceTo(o.getWorldPosition(world)));
    });
  }

  function place(): void {
    const cy = Math.cos(orbit.tilt), sy = Math.sin(orbit.tilt);
    camera.position.set(
      orbit.target.x + Math.sin(orbit.yaw) * cy * orbit.dist,
      orbit.target.y + sy * orbit.dist,
      orbit.target.z + Math.cos(orbit.yaw) * cy * orbit.dist);
    camera.lookAt(orbit.target);
    sizeMarks();
    onCam({yaw: Math.round((orbit.yaw * 180) / Math.PI), zoom: framed.dist / orbit.dist});
    draw();
  }

  let group = new THREE.Group();
  scene.add(group);
  let materials: Array<[Kind, THREE.ShaderMaterial]> = [];
  /* Held here so a room that arrives later is shaded to match. */
  let stops: Record<Kind, boolean> = {ground: false, wall: false, roof: false};
  let floor: THREE.Mesh | null = null;

  function shade(kind: Kind, material: THREE.ShaderMaterial): void {
    const [fillAlpha, edgeAlpha] = stops[kind] ? SHADE[kind].on : SHADE[kind].off;
    material.uniforms.fillAlpha.value = fillAlpha;
    material.uniforms.edgeAlpha.value = edgeAlpha;
    material.uniforms.dashed.value = stops[kind] ? 0 : 1;
    draw();
  }

  function solid(on: Record<Kind, boolean>): void {
    stops = {...on};
    for (const [kind, material] of materials) shade(kind, material);
  }

  /* `asked` is kept so a new room can rebuild the marks on its own floor. */
  let roomBox = new THREE.Box3();
  let floorTris: Float32Array = new Float32Array(0);
  let marked: THREE.Group | null = null;
  let asked: Marks | null = null;
  /* Rebuilt only when `m.ground` is a different object; an answer is replaced whole, never edited. */
  let region: THREE.Group | null = null;
  let regionOf: GroundMark | null = null;
  function marks(m: Marks): void {
    asked = m;
    if (marked) { scene.remove(marked); release(marked); }
    marked = build(m, roomBox, groundAt, floorTris);
    scene.add(marked);
    if (m.ground !== regionOf) {
      if (region) { scene.remove(region); release(region); region = null; }
      regionOf = m.ground;
      if (m.ground) { region = buildGround(m.ground); scene.add(region); }
    }
    sizeMarks();
    draw();
  }

  /* Synchronous: an await between teardown and build would let two builds merge. */
  function show(all: Parts, roofs: boolean): void {
    const parts: Array<[Kind, Float32Array]> = [];
    for (const kind of KINDS) {
      if (kind === 'roof' && !roofs) continue;
      /* The seabed is drawn with the floor but left out of `floorTris`. */
      const sea = kind === 'ground' ? all.seabed : undefined;
      if (sea && sea.length) {
        const both = new Float32Array(all.ground.length + sea.length);
        both.set(all.ground);
        both.set(sea, all.ground.length);
        parts.push([kind, both]);
        continue;
      }
      parts.push([kind, all[kind]]);
    }

    scene.remove(group);
    floor = null;
    group.traverse((o: THREE.Object3D) => {
      const m = o as THREE.Mesh;
      if (m.geometry) m.geometry.dispose();
    });
    materials.forEach(([, m]) => m.dispose());
    materials = [];
    group = new THREE.Group();

    for (const [kind, a] of parts) {
      if (!a.length) continue;
      const material = new THREE.ShaderMaterial({
        vertexShader: EDGE_VERT, fragmentShader: EDGE_FRAG, side: THREE.DoubleSide,
        uniforms: {
          fill: {value: new THREE.Color(COLOUR[kind])},
          edge: {value: new THREE.Color(EDGE[kind])},
          back: {value: new THREE.Color(BACK)},
          showEdges: {value: 1},
          fillAlpha: {value: 1},
          edgeAlpha: {value: 1},
          dashed: {value: 0},
          dashOn: {value: DASH_ON},
          dashOff: {value: DASH_OFF},
        },
      });
      shade(kind, material);
      materials.push([kind, material]);
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(a, 3));
      geometry.setAttribute('bary', baryFor(a.length / 9));
      geometry.computeVertexNormals();
      const mesh = new THREE.Mesh(geometry, material);
      /* Rays hit the floor alone, never a wall. */
      if (kind === 'ground') floor = mesh;
      group.add(mesh);
    }
    scene.add(group);

    const box = new THREE.Box3().setFromObject(group);
    roomBox = box.clone();
    floorTris = all.ground;
    if (asked) marks(asked);
    box.getCenter(orbit.target);
    orbit.dist = box.getSize(new THREE.Vector3()).length() * 0.75;
    framed = {dist: orbit.dist, target: orbit.target.clone()};
    speed = orbit.dist * 0.6;
    /* Centres on Link's start when there is one; `framed` keeps the room's centre. */
    homeTarget(orbit.target);
    place();
  }

  /* Link's start on its floor when set, else the room's framing. */
  function homeTarget(into: THREE.Vector3): THREE.Vector3 {
    const link = asked?.link;
    if (!link) return into.copy(framed.target);
    return into.set(link.x, groundAt(link.x, link.z) ?? framed.target.y, link.z);
  }

  let running = true;
  /* Last frame time; 0 means the next frame moves nothing. Steps are clamped to 0.1 s. */
  let last = 0;

  /* Frames only on demand, so an idle room leaves the CPU to the search; the loop stays alive only
     while a fly key is held. `pump` keeps at most one frame in flight. */
  let want = true;
  let pumped = false;
  function pump(): void {
    if (pumped || !running) return;
    pumped = true;
    requestAnimationFrame(frame);
  }
  function draw(): void {
    want = true;
    pump();
  }
  function frame(now: number): void {
    pumped = false;
    if (!running) return;
    if (sized) {
      renderer.setSize(sized.w, sized.h, false);
      camera.aspect = sized.w / sized.h;
      camera.updateProjectionMatrix();
      sized = null;
      place();
    }
    if (held.size) fly(last ? Math.min(0.1, (now - last) / 1000) : 0);
    last = now;
    if (want) {
      want = false;
      renderer.render(scene, camera);
    }
    if (held.size || want) pump();
  }

  /* The nearest floor hit under the cursor: what is drawn, not the game's ground test. */
  const ray = new THREE.Raycaster();
  const ndc = new THREE.Vector2();
  function floorAt(e: MouseEvent): Point | null {
    if (!floor) return null;
    const box = canvas.getBoundingClientRect();
    if (!box.width || !box.height) return null;
    ndc.set(((e.clientX - box.left) / box.width) * 2 - 1,
            -((e.clientY - box.top) / box.height) * 2 + 1);
    ray.setFromCamera(ndc, camera);
    const hit = ray.intersectObject(floor, false)[0];
    if (!hit) return null;
    const p = hit.point;
    return {x: p.x, y: p.y, z: p.z, px: e.clientX - box.left, py: e.clientY - box.top};
  }
  const dropPoint = (): void => onPoint(null);

  const DOWN = new THREE.Vector3(0, -1, 0);
  const from = new THREE.Vector3();
  function groundAt(x: number, z: number): number | null {
    if (!floor || roomBox.isEmpty()) return null;
    from.set(x, roomBox.max.y + 1000, z);
    ray.set(from, DOWN);
    const hit = ray.intersectObject(floor, false)[0];
    return hit ? hit.point.y : null;
  }

  let dragging: {x: number; y: number} | null = null;
  let dirty = false;
  const canvas = renderer.domElement;
  const onDown = (e: PointerEvent) => {
    /* Left button only, so a right-click does not move the view under its menu. */
    if (e.button !== 0) return;
    dragging = {x: e.clientX, y: e.clientY};
    canvas.setPointerCapture(e.pointerId);
  };
  const onMove = (e: PointerEvent) => {
    if (!dragging) { onPoint(floorAt(e)); return; }
    dropPoint();
    orbit.yaw += (e.clientX - dragging.x) * 0.006;
    orbit.tilt = Math.min(1.5, Math.max(-1.5, orbit.tilt + (e.clientY - dragging.y) * 0.004));
    dragging = {x: e.clientX, y: e.clientY};
    if (!dirty) {
      dirty = true;
      requestAnimationFrame(() => { dirty = false; place(); });
    }
  };
  const onUp = () => { dragging = null; };
  const onWheel = (e: WheelEvent) => {
    e.preventDefault();
    /* Shift turns the wheel horizontal. */
    const d = e.deltaY || e.deltaX;
    if (!d) return;
    /* Ctrl+wheel sets the fly speed. */
    if (e.ctrlKey) {
      speed = Math.max(1, Math.min(400000, speed * (d > 0 ? 0.833 : 1.2)));
      return;
    }
    zoom(d > 0 ? 1.1 : 0.909);
  };
  function zoom(by: number): void {
    orbit.dist = Math.max(50, Math.min(400000, orbit.dist * by));
    place();
  }

  /* WASD moves `orbit.target`. W/S follow the view direction, or with Shift slide along screen-up;
     A/D stay level. */
  const WAY: Record<string, [number, number]> = {w: [1, 0], s: [-1, 0], a: [0, -1], d: [0, 1]};
  const held = new Set<string>();
  /* Refreshed on every key event, since Shift can change while W is held. */
  let sliding = false;
  /* World units a second; `show` sets it from the room's size. */
  let speed = 2000;
  const fwd = new THREE.Vector3(), side = new THREE.Vector3(), step = new THREE.Vector3();
  const over = new THREE.Vector3();
  const UP = new THREE.Vector3(0, 1, 0);
  function fly(dt: number): void {
    if (!held.size || !dt) return;
    let f = 0, r = 0;
    for (const k of held) { const way = WAY[k]; if (way) { f += way[0]; r += way[1]; } }
    if (!f && !r) return;
    camera.getWorldDirection(fwd);
    side.crossVectors(fwd, UP);
    /* Zero rather than normalise a zero vector into NaN when looking straight down. */
    if (side.lengthSq() > 1e-8) side.normalize(); else side.set(0, 0, 0);
    /* Screen-up, not world-up. */
    over.crossVectors(side, fwd);
    if (over.lengthSq() > 1e-8) over.normalize(); else over.set(0, 0, 0);
    step.set(0, 0, 0).addScaledVector(sliding ? over : fwd, f).addScaledVector(side, r);
    if (step.lengthSq() < 1e-12) return;
    orbit.target.add(step.normalize().multiplyScalar(speed * dt));
    place();
  }
  const onKeyDown = (e: KeyboardEvent): void => {
    /* Before the guard: Shift alone is a keydown too. */
    sliding = e.shiftKey;
    const k = e.key.toLowerCase();
    if (!(k in WAY) || e.ctrlKey || e.metaKey || e.altKey) return;
    /* The loop was asleep, so the waking frame must not measure a step. */
    if (!held.size) last = 0;
    held.add(k);
    pump();
    e.preventDefault();
  };
  const onKeyUp = (e: KeyboardEvent): void => {
    sliding = e.shiftKey;
    held.delete(e.key.toLowerCase());
  };
  /* A key held while focus leaves never reports its release. */
  const dropKeys = (): void => held.clear();

  canvas.addEventListener('pointerdown', onDown);
  canvas.addEventListener('pointermove', onMove);
  canvas.addEventListener('wheel', onWheel, {passive: false});
  /* The browser's own menu is always suppressed. */
  const onContext = (e: MouseEvent): void => { e.preventDefault(); onMenu(floorAt(e)); };
  canvas.addEventListener('contextmenu', onContext);
  host.addEventListener('keydown', onKeyDown);
  host.addEventListener('blur', dropKeys);
  addEventListener('keyup', onKeyUp);
  addEventListener('blur', dropKeys);
  addEventListener('pointerup', onUp);
  /* `pointerleave` alone misses alt-tab, which never moves the cursor. */
  canvas.addEventListener('pointerleave', dropPoint);
  addEventListener('blur', dropPoint);
  const onOut = (e: PointerEvent): void => { if (!e.relatedTarget) dropPoint(); };
  document.addEventListener('pointerout', onOut);
  document.addEventListener('visibilitychange', dropPoint);
  document.addEventListener('visibilitychange', dropKeys);
  /* Frames are on demand, so a restored context needs a draw to refill. */
  canvas.addEventListener('webglcontextrestored', draw);

  /* Observes the panel, not the window. The size is applied in the frame, since resizing clears the
     drawing buffer and a resize in the callback can paint an empty canvas. */
  let sized: {w: number; h: number} | null = null;
  const size = new ResizeObserver(() => {
    const w = host.clientWidth, h = host.clientHeight;
    if (w && h) sized = {w, h};
  });
  size.observe(host);

  pump();

  return {
    show,
    marks,
    solid,
    zoom,
    groundAt,
    reset(): void {
      orbit.yaw = home.yaw;
      orbit.tilt = home.tilt;
      orbit.dist = framed.dist;
      homeTarget(orbit.target);
      place();
    },
    dispose(): void {
      running = false;
      size.disconnect();
      canvas.removeEventListener('pointerdown', onDown);
      canvas.removeEventListener('pointermove', onMove);
      canvas.removeEventListener('wheel', onWheel);
      canvas.removeEventListener('contextmenu', onContext);
      canvas.removeEventListener('pointerleave', dropPoint);
      host.removeEventListener('keydown', onKeyDown);
      host.removeEventListener('blur', dropKeys);
      removeEventListener('keyup', onKeyUp);
      removeEventListener('pointerup', onUp);
      removeEventListener('blur', dropPoint);
      removeEventListener('blur', dropKeys);
      document.removeEventListener('pointerout', onOut);
      document.removeEventListener('visibilitychange', dropPoint);
      document.removeEventListener('visibilitychange', dropKeys);
      canvas.removeEventListener('webglcontextrestored', draw);
      materials.forEach(([, m]) => m.dispose());
      group.traverse((o: THREE.Object3D) => {
        const m = o as THREE.Mesh;
        if (m.geometry) m.geometry.dispose();
      });
      if (marked) { scene.remove(marked); release(marked); marked = null; }
      if (region) { scene.remove(region); release(region); region = null; }
      renderer.dispose();
      canvas.remove();
    },
  };
}
