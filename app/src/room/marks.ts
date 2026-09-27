/* The room's marks: Link, the target, the bounds, the route, actors and sword. Drawn with depth
 * testing off and after the room, so the floor never buries them. Link and the target stand on the
 * floor under their X/Z (`groundAt`); Link's own `y`, when set, wins.
 */
import * as THREE from 'three';

export type Side = 'xmin' | 'xmax' | 'zmin' | 'zmax';

/** s16 angles: 65536 to the turn, 0 faces +Z. */
export interface FacingMark {a: number; b: number | null}

/** `y` is the running game's height, `null` once X or Z is typed. */
export interface LinkMark {x: number; z: number; y: number | null; facing: number | null}

export type TargetMark =
  /** A `null` axis is every value of that axis. */
  | {kind: 'point'; x: number | null; z: number | null; tol: number}
  /** `null` frees an axis; a huge finite float would overflow float32 on the GPU. */
  | {kind: 'range'; x0: number | null; x1: number | null; z0: number | null; z1: number | null;
     /** The hull of an address's bytes, not the target: drawn dashed, never filled or posted. */
     address?: boolean};

export interface BoundsMark {
  open: Record<Side, boolean>;
  value: Record<Side, number | null>;
}

/** The chosen plan's stops, where Link is; `item` is where the thrown item lands. */
export interface RouteMark {
  /** `y` is Link's height at the stop, `null` where none was measured. */
  points: Array<{x: number; z: number; y: number | null}>;
  item: {x: number; z: number} | null;
}

/** Present only when a chosen move swings; `on` is the collision switch. */
export interface SwordMark {on: boolean}

export interface ActorMark {x: number; z: number; r: number; h: number}

/** The core's ground triangles clipped to the target: `sides` vertices per polygon, `verts` as
 *  x, y, z in world units, each polygon coplanar with its floor triangle. */
export interface GroundMark {
  sides: number[];
  verts: number[];
}

export interface Marks {
  link: LinkMark | null;
  route: RouteMark | null;
  target: TargetMark | null;
  /** `null` until the core answers, or when the target has no floor. */
  ground: GroundMark | null;
  facing: FacingMark | null;
  bounds: BoundsMark | null;
  /** `null` when nothing chosen can swing, unlike a sword that is off. */
  sword: SwordMark | null;
  /** `null` with no game attached, unlike a game with no actors. */
  actors: {on: boolean; its: ActorMark[]} | null;
}

/* Every colour token, and whether it is drawn; one that is not says why. */
export const ROLES: Record<string, {drawn: boolean; why: string}> = {
  '--r-link': {drawn: true, why: ''},
  '--r-target': {drawn: true, why: ''},
  '--r-bounds': {drawn: true, why: ''},
  '--r-route': {drawn: true, why: ''},
  '--r-actor': {drawn: true, why: ''},
  '--r-sword': {drawn: true, why: ''},
};

const TURN = 65536;

/** Width, in units, under which a target piece also gets its edges drawn and dotted. */
const THIN = 1;

/** Size, in units, under which a target piece gets a dot at its middle. */
const SMALL = 40;
const dir = (f: number): number => (f / TURN) * Math.PI * 2;

function colour(token: string): THREE.Color {
  const raw = getComputedStyle(document.documentElement).getPropertyValue(token).trim();
  return new THREE.Color(raw || '#888888');
}

function lineMat(c: THREE.Color, opacity: number, width = 1): THREE.LineBasicMaterial {
  return new THREE.LineBasicMaterial({
    color: c, transparent: opacity < 1, opacity, depthTest: false, depthWrite: false,
    linewidth: width,
  });
}
function dashMat(c: THREE.Color, opacity: number, dash: number): THREE.LineDashedMaterial {
  return new THREE.LineDashedMaterial({
    color: c, transparent: opacity < 1, opacity, depthTest: false, depthWrite: false,
    dashSize: dash, gapSize: dash * 0.8,
  });
}
function fillMat(c: THREE.Color, opacity: number): THREE.MeshBasicMaterial {
  return new THREE.MeshBasicMaterial({
    color: c, transparent: true, opacity, depthTest: false, depthWrite: false,
    side: THREE.DoubleSide,
  });
}

function fromPoints(points: THREE.Vector3[]): THREE.BufferGeometry {
  return new THREE.BufferGeometry().setFromPoints(points);
}

function polyline(points: THREE.Vector3[], mat: THREE.Material, loop = false): THREE.Object3D {
  const dashed = (mat as THREE.LineDashedMaterial).isLineDashedMaterial === true;
  /* Dashed loops close with the first point: a `LineLoop` packs every dash into its closing side. */
  const g = fromPoints(dashed && loop && points.length ? [...points, points[0]] : points);
  const line = loop && !dashed ? new THREE.LineLoop(g, mat) : new THREE.Line(g, mat);
  if (dashed) line.computeLineDistances();
  return line;
}

function circlePoints(cx: number, cz: number, r: number, y: number): THREE.Vector3[] {
  const out: THREE.Vector3[] = [];
  for (let i = 0; i < 64; i++) {
    const a = (i / 64) * Math.PI * 2;
    out.push(new THREE.Vector3(cx + Math.sin(a) * r, y, cz + Math.cos(a) * r));
  }
  return out;
}

function quad(x0: number, x1: number, z0: number, z1: number, y: number,
              mat: THREE.Material): THREE.Mesh {
  const g = new THREE.PlaneGeometry(Math.abs(x1 - x0) || 1, Math.abs(z1 - z0) || 1);
  g.rotateX(-Math.PI / 2);
  g.translate((x0 + x1) / 2, y, (z0 + z1) / 2);
  return new THREE.Mesh(g, mat);
}

function disc(cx: number, cz: number, r: number, y: number, mat: THREE.Material): THREE.Mesh {
  const g = new THREE.CircleGeometry(r, 64);
  g.rotateX(-Math.PI / 2);
  g.translate(cx, y, cz);
  return new THREE.Mesh(g, mat);
}

/* Built at radius 1 with `userData.px`: the renderer rescales it to that many screen pixels.
 * Anything without `px` is a world distance and is never rescaled. */
function pointer(geometry: THREE.BufferGeometry, at: THREE.Vector3, px: number,
                 c: THREE.Color): THREE.Mesh {
  const m = new THREE.Mesh(geometry, fillMat(c, 1));
  m.position.copy(at);
  m.userData.px = px;
  return m;
}

function dot(at: THREE.Vector3, px: number, c: THREE.Color): THREE.Mesh {
  return pointer(new THREE.SphereGeometry(1, 12, 8), at, px, c);
}

/** Ground triangles clipped (Sutherland-Hodgman) to the cutting sides, so the bounds lie on the
 *  floor. A cut vertex is set exactly on its plane, so edges along a side are found by equality. */
function layBounds(floor: Float32Array, value: Record<Side, number | null>,
                   cuts: Side[]): {fill: number[]; edge: number[]} {
  const fill: number[] = [];
  const edge: number[] = [];
  const axis: Record<Side, 0 | 2> = {xmin: 0, xmax: 0, zmin: 2, zmax: 2};
  const low: Record<Side, boolean> = {xmin: true, xmax: false, zmin: true, zmax: false};
  const inside = (p: number[], s: Side): boolean =>
    low[s] ? p[axis[s]] >= (value[s] as number) : p[axis[s]] <= (value[s] as number);
  const on = (p: number[], s: Side): boolean => p[axis[s]] === value[s];

  for (let t = 0; t + 8 < floor.length; t += 9) {
    /* Early out for a triangle wholly past one side; this runs on every keystroke. */
    if (cuts.some(s => {
      const v = value[s] as number, k = t + axis[s];
      return low[s] ? floor[k] < v && floor[k + 3] < v && floor[k + 6] < v
                    : floor[k] > v && floor[k + 3] > v && floor[k + 6] > v;
    })) continue;
    let poly: number[][] = [
      [floor[t], floor[t + 1], floor[t + 2]],
      [floor[t + 3], floor[t + 4], floor[t + 5]],
      [floor[t + 6], floor[t + 7], floor[t + 8]],
    ];
    for (const s of cuts) {
      const v = value[s] as number, k = axis[s];
      const out: number[][] = [];
      for (let i = 0; i < poly.length; ++i) {
        const p = poly[i], q = poly[(i + 1) % poly.length];
        const pin = inside(p, s), qin = inside(q, s);
        if (pin) out.push(p);
        if (pin !== qin) {
          const f = (v - p[k]) / (q[k] - p[k]);
          const cut = [p[0] + (q[0] - p[0]) * f, p[1] + (q[1] - p[1]) * f, p[2] + (q[2] - p[2]) * f];
          cut[k] = v;
          out.push(cut);
        }
      }
      poly = out;
      if (poly.length < 3) break;
    }
    if (poly.length < 3) continue;
    for (let i = 1; i + 1 < poly.length; ++i) fill.push(...poly[0], ...poly[i], ...poly[i + 1]);
    for (let i = 0; i < poly.length; ++i) {
      const p = poly[i], q = poly[(i + 1) % poly.length];
      if (cuts.some(s => on(p, s) && on(q, s))) edge.push(...p, ...q);
    }
  }
  return {fill, edge};
}

/**
 * @param box the room's bounding box: where free axes and open sides run to, and the fallback height.
 * @param groundAt the renderer's floor ray, `null` where there is no floor.
 * @param floor the core's ground triangles, nine floats each; empty draws the bounds flat.
 */
export function build(marks: Marks, box: THREE.Box3,
                      groundAt: (x: number, z: number) => number | null = () => null,
                      floor: Float32Array = new Float32Array(0)): THREE.Group {
  const g = new THREE.Group();
  g.renderOrder = 1;
  const empty = box.isEmpty();
  const y = empty ? 0 : box.min.y;
  const stand = (x: number, z: number): number => groundAt(x, z) ?? y;
  const span = empty ? 1000 : Math.max(box.max.x - box.min.x, box.max.z - box.min.z);
  const dash = Math.max(6, span * 0.01);
  const ROOM_PAD = 60;

  const add = (o: THREE.Object3D): void => { g.add(o); };

  /* ── the bounds: where the search may look. Dashed means it runs off that side. ───────────── */
  const b = marks.bounds;
  /* With no room an open side has no edge, so only a fully shut box is drawn. */
  const allShut = b ? (Object.keys(b.open) as Side[]).every(s => !b.open[s]) : false;
  if (b && (!empty || allShut)) {
    const at = (s: Side, fallback: number): number =>
      b.open[s] ? fallback : (b.value[s] ?? fallback);
    const x0 = at('xmin', box.min.x - ROOM_PAD), x1 = at('xmax', box.max.x + ROOM_PAD);
    const z0 = at('zmin', box.min.z - ROOM_PAD), z1 = at('zmax', box.max.z + ROOM_PAD);
    const c = colour('--r-bounds');
    const cuts = (Object.keys(b.open) as Side[]).filter(s => !b.open[s] && b.value[s] !== null);
    /* A min above its max is swapped, as the seam does before a search. */
    const sorted = {...b.value};
    for (const [lo, hi] of [['xmin', 'xmax'], ['zmin', 'zmax']] as Array<[Side, Side]>) {
      const a = sorted[lo], z = sorted[hi];
      if (cuts.includes(lo) && cuts.includes(hi) && a !== null && z !== null && a > z) {
        sorted[lo] = z;
        sorted[hi] = a;
      }
    }
    const laid = floor.length > 0 ? layBounds(floor, sorted, cuts) : null;
    if (laid) {
      if (laid.fill.length) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(laid.fill, 3));
        add(new THREE.Mesh(geo, fillMat(c, 0.07)));
      }
      if (laid.edge.length) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(laid.edge, 3));
        add(new THREE.LineSegments(geo, lineMat(c, 0.75)));
      }
    } else {
      add(quad(x0, x1, z0, z1, y + 1, fillMat(c, 0.07)));
    }
    const edges: Array<[Side, [number, number], [number, number]]> = [
      ['xmin', [x0, z0], [x0, z1]], ['xmax', [x1, z0], [x1, z1]],
      ['zmin', [x0, z0], [x1, z0]], ['zmax', [x0, z1], [x1, z1]],
    ];
    for (const [side, p, q] of edges) {
      /* Already drawn as the laid edge. */
      if (laid && cuts.includes(side)) continue;
      const pts = [new THREE.Vector3(p[0], y + 1, p[1]), new THREE.Vector3(q[0], y + 1, q[1])];
      add(polyline(pts, b.open[side] ? dashMat(c, 0.75, dash) : lineMat(c, 0.75)));
    }
  }

  /* ── the target: its own colour, and it is whatever shape was asked for ───────────────────── */
  const tc = colour('--r-target');

  /* The core's region is drawn by `buildGround`. */
  const gm = marks.ground;

  /** A ring on the floor, a pole, and a fixed-size diamond on top. */
  function marker(x: number, z: number, radius: number): THREE.Vector3 {
    const r = radius > 0 ? radius : 7;
    const ty = stand(x, z);
    add(disc(x, z, r, ty + 2, fillMat(tc, 0.12)));
    add(polyline(circlePoints(x, z, r, ty + 2),
      radius > 0 ? lineMat(tc, 1) : dashMat(tc, 1, Math.max(3, r / 3)), true));
    add(polyline([new THREE.Vector3(x, ty, z), new THREE.Vector3(x, ty + 88, z)], lineMat(tc, 1)));
    add(pointer(new THREE.OctahedronGeometry(1), new THREE.Vector3(x, ty + 88, z), 9, tc));
    return new THREE.Vector3(x, ty, z);
  }

  /* The target's middle, where the facing arc hangs. */
  let aim: THREE.Vector3 | null = null;
  const tm = marks.target;
  if (tm && tm.kind === 'range') {
    /* A freed axis spans the room. */
    const span = (lo: number | null, hi: number | null, min: number, max: number,
                  mid: number): [number, number] =>
      lo === null || hi === null
        ? (empty ? [mid - 900, mid + 900] : [min - 120, max + 120])
        : [Math.min(lo, hi), Math.max(lo, hi)];
    const [rx0, rx1] = span(tm.x0, tm.x1, box.min.x, box.max.x, 0);
    const [rz0, rz1] = span(tm.z0, tm.z1, box.min.z, box.max.z, 0);
    /* Dashed when an edge is freed or the box is an address hull. */
    const loose = tm.address === true
      || tm.x0 === null || tm.x1 === null || tm.z0 === null || tm.z1 === null;
    const mid: [number, number] = [(rx0 + rx1) / 2, (rz0 + rz1) / 2];
    const ry = stand(mid[0], mid[1]);
    /* Once the core's ground arrives the box keeps only its outline, so the fill does not hide it. */
    /* About eight dashes to the shorter side, never longer than the room's dash. */
    const boxDash = Math.min(dash, Math.max(0.25,
      Math.min(Math.abs(rx1 - rx0), Math.abs(rz1 - rz0)) / 14.4));
    const sides: Side[] = ['xmin', 'xmax', 'zmin', 'zmax'];
    const laid = floor.length > 0
      ? layBounds(floor, {xmin: rx0, xmax: rx1, zmin: rz0, zmax: rz1}, sides) : null;
    if (laid && laid.edge.length) {
      if (!gm && !tm.address && laid.fill.length) {
        const geo = new THREE.BufferGeometry();
        geo.setAttribute('position', new THREE.Float32BufferAttribute(laid.fill, 3));
        add(new THREE.Mesh(geo, fillMat(tc, 0.14)));
      }
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(laid.edge, 3));
      const outline = new THREE.LineSegments(geo, loose ? dashMat(tc, 1, boxDash) : lineMat(tc, 1));
      if (loose) {
        /* Distance along the side, not per piece, so dashes run unbroken across pieces. */
        const e = laid.edge, along: number[] = [];
        for (let i = 0; i + 5 < e.length; i += 6) {
          const onX = e[i] === e[i + 3] && (e[i] === rx0 || e[i] === rx1);
          along.push(onX ? e[i + 2] - rz0 : e[i] - rx0, onX ? e[i + 5] - rz0 : e[i + 3] - rx0);
        }
        geo.setAttribute('lineDistance', new THREE.Float32BufferAttribute(along, 1));
      }
      add(outline);
    } else {
      if (!gm && !tm.address) add(quad(rx0, rx1, rz0, rz1, ry + 2, fillMat(tc, 0.14)));
      add(polyline([
        new THREE.Vector3(rx0, ry + 2, rz0), new THREE.Vector3(rx1, ry + 2, rz0),
        new THREE.Vector3(rx1, ry + 2, rz1), new THREE.Vector3(rx0, ry + 2, rz1),
      ], loose ? dashMat(tc, 1, boxDash) : lineMat(tc, 1), true));
    }
    if (!loose) {
      for (const [cx, cz] of [[rx0, rz0], [rx1, rz0], [rx1, rz1], [rx0, rz1]]) {
        const cy = stand(cx, cz);
        add(polyline([new THREE.Vector3(cx, cy, cz), new THREE.Vector3(cx, cy + 34, cz)],
          lineMat(tc, 0.7)));
      }
    }
    if (Math.max(Math.abs(rx1 - rx0), Math.abs(rz1 - rz0)) < 24) marker(mid[0], mid[1], 0);
    aim = new THREE.Vector3(mid[0], ry, mid[1]);
  } else if (tm && tm.kind === 'point') {
    const free = {x: tm.x === null, z: tm.z === null};
    const midX = empty ? 0 : (box.min.x + box.max.x) / 2;
    const midZ = empty ? 0 : (box.min.z + box.max.z) / 2;
    const ends = (lo: number, hi: number, mid: number): [number, number] =>
      empty ? [mid - 900, mid + 900] : [lo - 120, hi + 120];
    /* Both axes free has no shape to draw. */
    if (free.x !== free.z) {
      const e = free.x ? ends(box.min.x, box.max.x, midX) : ends(box.min.z, box.max.z, midZ);
      const at = free.x ? (tm.z as number) : (tm.x as number);
      const by = stand(free.x ? (e[0] + e[1]) / 2 : at, free.x ? at : (e[0] + e[1]) / 2) + 1;
      if (tm.tol > 0) {
        const x0 = free.x ? e[0] : at - tm.tol;
        const x1 = free.x ? e[1] : at + tm.tol;
        const z0 = free.x ? at - tm.tol : e[0];
        const z1 = free.x ? at + tm.tol : e[1];
        add(quad(x0, x1, z0, z1, by, fillMat(tc, 0.16)));
        add(polyline([
          new THREE.Vector3(x0, by, z0), new THREE.Vector3(x1, by, z0),
          new THREE.Vector3(x1, by, z1), new THREE.Vector3(x0, by, z1),
        ], dashMat(tc, 1, dash), true));
      } else {
        /* No tolerance is a line, not a zero-width strip. */
        add(polyline(free.x
          ? [new THREE.Vector3(e[0], by, at), new THREE.Vector3(e[1], by, at)]
          : [new THREE.Vector3(at, by, e[0]), new THREE.Vector3(at, by, e[1])],
          dashMat(tc, 1, dash)));
      }
    }
    if (free.x && free.z) aim = new THREE.Vector3(midX, stand(midX, midZ), midZ);
    else aim = marker(free.x ? midX : (tm.x as number), free.z ? midZ : (tm.z as number), tm.tol);
  }

  /* ── the facing he has to finish on, drawn where he has to finish ─────────────────────────── */
  const fm = marks.facing;
  if (fm && aim) {
    const ay = aim.y;
    const at = (f: number, r: number): THREE.Vector3 =>
      new THREE.Vector3(aim.x + Math.sin(dir(f)) * r, ay + 3, aim.z + Math.cos(dir(f)) * r);
    const a0 = fm.a;
    const arcSpan = fm.b === null ? 0 : ((fm.b - a0 + TURN) % TURN);
    if (arcSpan > 0) {
      const steps = Math.max(1, Math.round((arcSpan / TURN) * 48));
      const pts = [new THREE.Vector3(aim.x, ay + 3, aim.z)];
      for (let i = 0; i <= steps; i++) pts.push(at(a0 + arcSpan * (i / steps), 66));
      const verts: number[] = [];
      for (let i = 1; i < pts.length - 1; i++) {
        verts.push(pts[0].x, pts[0].y, pts[0].z, pts[i].x, pts[i].y, pts[i].z,
          pts[i + 1].x, pts[i + 1].y, pts[i + 1].z);
      }
      const fan = new THREE.BufferGeometry();
      fan.setAttribute('position', new THREE.BufferAttribute(new Float32Array(verts), 3));
      add(new THREE.Mesh(fan, fillMat(tc, 0.2)));
      add(polyline(pts.slice(1), lineMat(tc, 0.8)));
    }
    for (const [i, f] of [a0, a0 + arcSpan].entries()) {
      if (i === 1 && arcSpan === 0) continue;
      const tip = at(f, 74);
      add(polyline([new THREE.Vector3(aim.x, ay + 3, aim.z), tip], lineMat(tc, 1)));
      add(dot(tip, 3.5, tc));
    }
  }

  /* ── the route: where the chosen plan takes him, and where the thing he threw lands ──────── */
  const rt = marks.route;
  if (rt && rt.points.length) {
    const rc = colour('--r-route');
    const at = (p: {x: number; z: number; y?: number | null}): THREE.Vector3 =>
      new THREE.Vector3(p.x, (p.y ?? stand(p.x, p.z)) + 6, p.z);
    if (rt.points.length > 1) {
      add(polyline(rt.points.map(at), dashMat(rc, 0.85, Math.max(4, dash / 3))));
      /* Inner stops are dots; the last is a square. */
      for (const p of rt.points.slice(1, -1)) add(dot(at(p), 4, rc));
    }
    const last = rt.points[rt.points.length - 1];
    const ly = at(last).y;
    const s = 8;
    add(polyline([
      new THREE.Vector3(last.x - s, ly, last.z - s), new THREE.Vector3(last.x + s, ly, last.z - s),
      new THREE.Vector3(last.x + s, ly, last.z + s), new THREE.Vector3(last.x - s, ly, last.z + s),
    ], lineMat(rc, 1), true));
    /* Tie Link's last stop to where the aimed item lands. */
    if (rt.item) {
      add(polyline([at(last), at(rt.item)], lineMat(rc, 0.7)));
      add(dot(at(rt.item), 5, rc));
    }
  }

  /* ── the actors, which are only here at all when a game is attached ───────────────── */
  const am = marks.actors;
  if (am) {
    const ac = colour('--r-actor');
    for (const a of am.its) {
      add(disc(a.x, a.z, a.r, y + 2, fillMat(ac, am.on ? 0.18 : 0.05)));
      add(polyline(circlePoints(a.x, a.z, a.r, y + 2),
        am.on ? lineMat(ac, 1) : dashMat(ac, 0.4, dash), true));
      add(polyline([new THREE.Vector3(a.x, y, a.z), new THREE.Vector3(a.x, y + a.h, a.z)],
        lineMat(ac, am.on ? 0.8 : 0.3)));
    }
  }

  /* ── Link: where he is standing, and which way he is looking ──────────────────────────────── */
  const lm = marks.link;
  if (lm) {
    const lc = colour('--r-link');
    const ly = lm.y ?? stand(lm.x, lm.z);
    add(disc(lm.x, lm.z, 34, ly + 2, fillMat(lc, 0.2)));
    add(polyline(circlePoints(lm.x, lm.z, 34, ly + 2), lineMat(lc, 1), true));
    add(polyline([new THREE.Vector3(lm.x, ly, lm.z), new THREE.Vector3(lm.x, ly + 62, lm.z)],
      lineMat(lc, 1)));
    if (lm.facing !== null) {
      const a = dir(lm.facing);
      const tip = new THREE.Vector3(lm.x + Math.sin(a) * 78, ly + 2, lm.z + Math.cos(a) * 78);
      add(polyline([new THREE.Vector3(lm.x, ly + 2, lm.z), tip], lineMat(lc, 1)));
      add(dot(tip, 3.5, lc));
    }

    /* ── the blade's reach, on the arm he is facing with ─────────────────────────── */
    const sm = marks.sword;
    if (sm && lm.facing !== null) {
      const sc = colour('--r-sword');
      const a0 = dir(lm.facing) - 0.55;
      const arc: THREE.Vector3[] = [new THREE.Vector3(lm.x, ly + 4, lm.z)];
      for (let i = 0; i <= 12; i++) {
        const a = a0 + 1.1 * (i / 12);
        arc.push(new THREE.Vector3(lm.x + Math.sin(a) * 96, ly + 4, lm.z + Math.cos(a) * 96));
      }
      const verts: number[] = [];
      for (let i = 1; i < arc.length - 1; i++) {
        verts.push(arc[0].x, arc[0].y, arc[0].z, arc[i].x, arc[i].y, arc[i].z,
          arc[i + 1].x, arc[i + 1].y, arc[i + 1].z);
      }
      const wedge = new THREE.BufferGeometry();
      wedge.setAttribute('position', new THREE.BufferAttribute(new Float32Array(verts), 3));
      add(new THREE.Mesh(wedge, fillMat(sc, sm.on ? 0.16 : 0.05)));
      add(polyline(arc, sm.on ? lineMat(sc, 0.8) : dashMat(sc, 0.3, dash), true));
    }
  }

  return g;
}

/** The core's ground region as its own group, rebuilt only when `marks.ground` changes, since
 *  `build` re-runs on every edit. One geometry for the whole region. */
export function buildGround(gm: GroundMark): THREE.Group {
  const g = new THREE.Group();
  g.renderOrder = 1;
  const add = (o: THREE.Object3D): void => { g.add(o); };
  const tc = colour('--r-target');

  if (gm.sides.length > 0) {
    const pos: number[] = [];
    const line: number[] = [];
    /* Fixed-size screen dots, for pieces too small to see as geometry. */
    const dot: number[] = [];
    let at = 0;
    for (const n of gm.sides) {
      const v = at;
      at += n;
      /* Twice the signed XZ area; zero is a line (a zero-tolerance target), drawn as one. */
      let area = 0;
      for (let i = 0; i < n; ++i) {
        const a = (v + i) * 3, b = (v + ((i + 1) % n)) * 3;
        area += gm.verts[a] * gm.verts[b + 2] - gm.verts[b] * gm.verts[a + 2];
      }
      const middle = (): void => {
        let x = 0, y = 0, z = 0;
        for (let i = 0; i < n; ++i) {
          x += gm.verts[(v + i) * 3]; y += gm.verts[(v + i) * 3 + 1]; z += gm.verts[(v + i) * 3 + 2];
        }
        dot.push(x / n, y / n, z / n);
      };
      if (n < 3 || Math.abs(area) < 1e-6) {
        middle();
        for (let i = 0; i + 1 < n; ++i) {
          const a = (v + i) * 3, b = (v + i + 1) * 3;
          line.push(gm.verts[a], gm.verts[a + 1], gm.verts[a + 2],
                    gm.verts[b], gm.verts[b + 1], gm.verts[b + 2]);
        }
        continue;
      }
      /* Mean width is 2*area/perimeter (`area` is already doubled); under `THIN` the edges are also
         drawn and dotted so the piece stays visible. Wider pieces are only filled, hiding seams. */
      let perimeter = 0;
      for (let i = 0; i < n; ++i) {
        const a = (v + i) * 3, b = (v + ((i + 1) % n)) * 3;
        perimeter += Math.hypot(gm.verts[b] - gm.verts[a], gm.verts[b + 2] - gm.verts[a + 2]);
      }
      if (perimeter < SMALL) middle();
      if (perimeter > 0 && Math.abs(area) / perimeter < THIN) {
        for (let i = 0; i < n; ++i) {
          const a = (v + i) * 3, b = (v + ((i + 1) % n)) * 3;
          line.push(gm.verts[a], gm.verts[a + 1], gm.verts[a + 2],
                    gm.verts[b], gm.verts[b + 1], gm.verts[b + 2]);
          const len = Math.hypot(gm.verts[b] - gm.verts[a], gm.verts[b + 2] - gm.verts[a + 2]);
          const steps = Math.max(1, Math.ceil(len / THIN));
          for (let k = 0; k < steps; ++k) {
            const f = k / steps;
            dot.push(gm.verts[a] + (gm.verts[b] - gm.verts[a]) * f,
                     gm.verts[a + 1] + (gm.verts[b + 1] - gm.verts[a + 1]) * f,
                     gm.verts[a + 2] + (gm.verts[b + 2] - gm.verts[a + 2]) * f);
          }
        }
      }
      /* Fan from the first vertex; the polygon is convex. */
      for (let i = 1; i + 1 < n; ++i) {
        const a = v * 3, b = (v + i) * 3, c = (v + i + 1) * 3;
        pos.push(gm.verts[a], gm.verts[a + 1], gm.verts[a + 2],
                 gm.verts[b], gm.verts[b + 1], gm.verts[b + 2],
                 gm.verts[c], gm.verts[c + 1], gm.verts[c + 2]);
      }
    }
    if (pos.length) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
      add(new THREE.Mesh(geo, fillMat(tc, 0.3)));
    }
    if (line.length) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(line, 3));
      add(new THREE.LineSegments(geo, lineMat(tc, 0.9)));
    }
    if (dot.length) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.Float32BufferAttribute(dot, 3));
      add(new THREE.Points(geo, new THREE.PointsMaterial({color: tc, size: 8,
                                                          sizeAttenuation: false})));
    }
  }

  return g;
}

/** Disposes every geometry and material in a group. */
export function release(g: THREE.Group): void {
  g.traverse((o: THREE.Object3D) => {
    const m = o as THREE.Mesh;
    if (m.geometry) m.geometry.dispose();
    const mat = m.material as THREE.Material | THREE.Material[] | undefined;
    if (Array.isArray(mat)) mat.forEach(x => x.dispose());
    else if (mat) mat.dispose();
  });
}
