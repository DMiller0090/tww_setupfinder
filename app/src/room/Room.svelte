<script lang="ts">
  import {onMount, untrack} from 'svelte';
  import {t} from '../lib/strings';
  import {say} from '../lib/log.svelte';
  import {mount, type Cam, type Parts, type Point, type RoomView} from './renderer';
  import type {Marks} from './marks';
  import type {Kind, RoomIndex} from '../core/fixtures';
import {ground} from '../core/ground.svelte';

  let {picked, parts = null, roofs = false, marks, solid, phase = 'design', back, fwd,
       canFwd = false, copy, offFloor = () => {}}:
    {picked: RoomIndex | null; parts?: Parts | null; roofs?: boolean; marks: Marks;
     solid: Record<Kind, boolean>;
     phase?: string; back: () => void; fwd: () => void; canFwd?: boolean;
     copy: (text: string) => Promise<boolean>; offFloor?: () => void} = $props();
  let host: HTMLElement;
  let view = $state<RoomView | null>(null);
  /* `null`: off the floor, or no cursor. */
  let at = $state<Point | null>(null);
  let pop = $state<HTMLElement | null>(null);
  /* `null` until the camera has been placed. */
  let cam = $state<Cam | null>(null);
  /* Where the right-click menu opened; `null` is no menu. */
  let menu = $state<Point | null>(null);

  onMount(() => {
    const it = mount(host, (p) => at = p, (c) => cam = c, (p) => menu = p);
    view = it;
    return () => it.dispose();
  });

  async function take(value: number): Promise<void> {
    menu = null;
    if (await copy(String(value))) say('info', t.copied);
  }

  $effect(() => {
    if (!view || !parts) return;
    view.show(parts, roofs);
    /* A start off every floor of the new room is cleared. Untracked so rail edits do not re-run this. */
    const link = untrack(() => marks.link);
    if (link && view.groundAt(link.x, link.z) === null) offFloor();
  });

  /* A camera move or a new room closes the menu. */
  $effect(() => { void parts; void cam; menu = null; });

  /* Separate effects so marks and collision changes do not rebuild the room. */
  $effect(() => { view?.marks(marks); });

  $effect(() => { view?.solid(solid); });

  /* Clamped inside the panel, measured after it is filled. */
  $effect(() => {
    const p = at, box = pop;
    if (!p || !box) return;
    const w = host.clientWidth, h = host.clientHeight;
    box.style.left = Math.min(p.px + 14, Math.max(0, w - box.offsetWidth - 6)) + 'px';
    box.style.top = Math.min(p.py + 14, Math.max(0, h - box.offsetHeight - 6)) + 'px';
  });

  let menuBox = $state<HTMLElement | null>(null);
  $effect(() => {
    const p = menu, box = menuBox;
    if (!p || !box) return;
    const w = host.clientWidth, h = host.clientHeight;
    box.style.left = Math.min(p.px, Math.max(0, w - box.offsetWidth - 6)) + 'px';
    box.style.top = Math.min(p.py, Math.max(0, h - box.offsetHeight - 6)) + 'px';
  });

  const facts = $derived(picked
    ? `${picked.counts.wall.toLocaleString()} ${t.walls} · ` +
      `${picked.counts.ground.toLocaleString()} ${t.floors}` +
      (roofs ? ` · ${picked.counts.roof.toLocaleString()} ${t.roofs}` : '')
    : '');
</script>

<svelte:window
  onpointerdown={(e: PointerEvent) => {
    if (menu && !(e.target as HTMLElement | null)?.closest?.('.menu')) menu = null;
  }}
  onkeydown={(e: KeyboardEvent) => { if (e.key === 'Escape') menu = null; }} />

<section class="panel stage">
  <div class="phead"><h2>{t.room}</h2><span class="sp"></span><span class="r">{facts}</span></div>
  <!-- svelte-ignore a11y_no_noninteractive_tabindex
       Focusable so the room can be orbited from the keyboard. -->
  <div class="view" id="view" bind:this={host} tabindex="0" role="application"
       aria-label={t.roomHelp}>
    <!-- Full precision: nothing here formats the number. -->
    <div class="hover" bind:this={pop} hidden={!at} aria-hidden="true">
      {#if at}<u>{t.x}</u><span>{at.x}</span><u>{t.y}</u><span>{at.y}</span
        ><u>{t.z}</u><span>{at.z}</span>{/if}
    </div>
    {#if menu}
      {@const m = menu}
      <!-- Placed at the click before it is measured, so it does not flicker in the corner. -->
      <div class="menu" bind:this={menuBox} role="menu" tabindex="-1" aria-label={t.copy}
           style:left="{m.px}px" style:top="{m.py}px">
        <button type="button" role="menuitem" onclick={() => void take(m.x)}>{t.x}</button>
        <button type="button" role="menuitem" onclick={() => void take(m.y)}>{t.y}</button>
        <button type="button" role="menuitem" onclick={() => void take(m.z)}>{t.z}</button>
      </div>
    {/if}
    <div class="viewbar">
      <button type="button" class="icon ghost" aria-label={t.zoomIn}
              onclick={() => view?.zoom(0.833)}>+</button>
      <button type="button" class="icon ghost" aria-label={t.zoomOut}
              onclick={() => view?.zoom(1.2)}>−</button>
      <button type="button" class="icon ghost" onclick={() => view?.reset()}>{t.reset}</button>
      <span class="k">{cam ? t.camAt(cam.yaw, cam.zoom.toFixed(2)) : t.dragToOrbit}</span>
    </div>
    <!-- A floor query still with the core. -->
    <div class="waiting" hidden={!ground.waiting} aria-hidden="true"></div>
    <button type="button" class="tab left" onclick={back}>{t.playTab}</button>
    <button type="button" class="tab right" hidden={!(phase === 'design' && canFwd)}
            onclick={fwd}>{t.plansTab}</button>
  </div>
</section>

<style>
  .view :global(canvas) {display:block; width:100%; height:100%}
  /* Reduced motion slows the ring rather than stopping it: a still ring does not read as working. */
  .waiting {position:absolute; right:.5rem; bottom:.5rem; width:2.625rem; height:2.625rem;
            border-radius:50%; border:.375rem solid rgba(86,194,255,.22);
            border-top-color:var(--accent); animation:turn .8s linear infinite;
            pointer-events:none}
  .waiting[hidden] {display:none}
  @keyframes turn {to {transform:rotate(1turn)}}
  @media (prefers-reduced-motion:reduce) {.waiting {animation-duration:2.4s}}
</style>
