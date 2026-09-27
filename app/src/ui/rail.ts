/* Gutters and rail sizing. A panel is sized between its content and the other panels' slack;
 * spare height is shared evenly among panels not dragged by hand. */
const root = document.documentElement;

const RAIL = ['pLoc', 'pTgt', 'pBnd'] as const;
type PanelId = (typeof RAIL)[number];
const RAILVAR: Record<PanelId, string> = {
  pLoc: '--h-loc', pTgt: '--h-tgt', pBnd: '--h-bnd',
};

const pinned: Partial<Record<PanelId, boolean>> = {};

const cssNum = (name: string): number =>
  parseFloat(getComputedStyle(root).getPropertyValue(name)) || 0;

const panelOf = (id: PanelId): HTMLElement | null => document.getElementById(id);

/** Content height; not scrollHeight, which reports the box when the box is taller. */
function natural(pnl: HTMLElement | null): number {
  if (!pnl) return 0;
  const head = pnl.querySelector('.phead') as HTMLElement | null;
  const body = pnl.querySelector('.body') as HTMLElement | null;
  let inside = 0;
  if (body) {
    const cs = getComputedStyle(body);
    inside = parseFloat(cs.paddingTop) + parseFloat(cs.paddingBottom);
    for (const c of Array.from(body.children) as HTMLElement[]) {
      const m = getComputedStyle(c);
      inside += c.offsetHeight + parseFloat(m.marginTop) + parseFloat(m.marginBottom);
    }
  }
  return (head ? head.offsetHeight : 0) + inside + 2;
}

function railEl(): HTMLElement | null {
  return document.querySelector('.rail');
}

/** Size range in rem for a panel, leaving every other panel its content. */
function slack(panel: HTMLElement): {min: number; max: number} {
  const rail = railEl();
  const mine = natural(panel);
  if (!rail) return {min: mine / 16, max: mine / 16};
  const room = rail.clientHeight - parseFloat(getComputedStyle(rail).paddingTop) * 2;
  let taken = 0;
  for (const k of RAIL) { const p = panelOf(k); if (p && p !== panel) taken += natural(p); }
  rail.querySelectorAll('.gutter').forEach(x => { taken += (x as HTMLElement).offsetHeight; });
  return {min: mine / 16, max: Math.max(mine, room - taken) / 16};
}

export function fitRail(): void {
  const rail = railEl();
  if (!rail?.clientHeight) return;
  const pad = parseFloat(getComputedStyle(rail).paddingTop) * 2;
  let room = rail.clientHeight - pad;
  rail.querySelectorAll('.gutter').forEach(x => { room -= (x as HTMLElement).offsetHeight; });
  const free = RAIL.filter(k => !pinned[k]);
  let rest = room;
  for (const k of RAIL) if (pinned[k]) rest -= cssNum(RAILVAR[k]) * 16;
  const nat: Partial<Record<PanelId, number>> = {};
  for (const k of free) { nat[k] = natural(panelOf(k)); rest -= nat[k] as number; }
  const share = free.length ? Math.max(0, rest / free.length) : 0;
  /* Write only on change: any write re-fires the ResizeObserver and can raise a loop error. */
  for (const k of free) {
    const was = ((nat[k] as number) + share) / 16;
    const now = was.toFixed(3) + 'rem';
    if (root.style.getPropertyValue(RAILVAR[k]) !== now) root.style.setProperty(RAILVAR[k], now);
  }
}

export function rail(node: HTMLElement): {destroy(): void} {
  /* Deferred a frame: writing sizes inside the ResizeObserver callback cannot settle. */
  let due = 0;
  const watch = new ResizeObserver(() => {
    if (due) return;
    due = requestAnimationFrame(() => { due = 0; fitRail(); });
  });
  watch.observe(node);
  requestAnimationFrame(() => fitRail());
  return {destroy(): void { if (due) cancelAnimationFrame(due); watch.disconnect(); }};
}

function draggable(node: HTMLElement, at: (e: PointerEvent) => number,
                   step: (d: number) => void, set: (v: number) => void): {destroy(): void} {
  let live = false;
  const down = (e: PointerEvent): void => {
    live = true;
    root.classList.add('dragging');
    /* Throws if the button is already up. */
    try { node.setPointerCapture(e.pointerId); } catch { /* the drag still works without it */ }
    e.preventDefault();
  };
  const move = (e: PointerEvent): void => { if (live) set(at(e)); };
  const up = (): void => { live = false; root.classList.remove('dragging'); };
  const key = (e: KeyboardEvent): void => {
    const d = ({ArrowUp: -1, ArrowDown: 1, ArrowLeft: -1, ArrowRight: 1} as
      Record<string, number>)[e.key];
    if (d === undefined) return;
    e.preventDefault();
    step(d);
  };
  node.addEventListener('pointerdown', down);
  node.addEventListener('pointermove', move);
  node.addEventListener('pointerup', up);
  node.addEventListener('keydown', key);
  return {
    destroy(): void {
      node.removeEventListener('pointerdown', down);
      node.removeEventListener('pointermove', move);
      node.removeEventListener('pointerup', up);
      node.removeEventListener('keydown', key);
    },
  };
}

/** A gutter between two panels; sizes the one above it. */
export function vgutter(node: HTMLElement, id: PanelId): {destroy(): void} {
  const set = (v: number): void => {
    const panel = panelOf(id);
    if (!panel) return;
    const room = slack(panel);
    const put = Math.min(room.max, Math.max(room.min, v));
    root.style.setProperty(RAILVAR[id], put.toFixed(3) + 'rem');
    pinned[id] = true;
    fitRail();
    node.setAttribute('aria-valuenow', String(Math.round(put)));
    node.setAttribute('aria-valuemin', String(Math.round(room.min)));
    node.setAttribute('aria-valuemax', String(Math.round(room.max)));
  };
  const it = draggable(node,
    e => {
      const panel = panelOf(id);
      return panel ? (e.clientY - panel.getBoundingClientRect().top) / 16 : 0;
    },
    d => {
      const panel = panelOf(id);
      set((cssNum(RAILVAR[id]) || (panel ? panel.getBoundingClientRect().height / 16 : 0)) + d);
    },
    set);
  const panel = panelOf(id);
  if (panel) {
    node.setAttribute('aria-valuenow', String(Math.round(panel.getBoundingClientRect().height / 16)));
  }
  return it;
}

/** A gutter over one CSS variable; `from` is the edge the size is measured from. */
export function gutter(node: HTMLElement,
                       o: {name: string; min: number; max: number;
                           from: 'left' | 'right' | 'bottom'}): {destroy(): void} {
  const set = (v: number): void => {
    const put = Math.min(o.max, Math.max(o.min, v));
    root.style.setProperty(o.name, put.toFixed(3) + (o.from === 'bottom' ? '%' : 'rem'));
    node.setAttribute('aria-valuenow', String(Math.round(put)));
  };
  node.setAttribute('aria-valuemin', String(o.min));
  node.setAttribute('aria-valuemax', String(o.max));
  node.setAttribute('aria-valuenow', String(Math.round(cssNum(o.name))));
  return draggable(node,
    e => {
      const box = node.parentElement?.getBoundingClientRect();
      if (!box) return o.min;
      if (o.from === 'right') return (box.right - e.clientX) / 16;
      if (o.from === 'left') return (e.clientX - box.left) / 16;
      return ((box.bottom - e.clientY) / box.height) * 100;
    },
    d => set(cssNum(o.name) + d * (o.from === 'right' ? -2 : 2)),
    set);
}

/** The gutter beside the rail; drags `--rail`. */
export function hgutter(node: HTMLElement): {destroy(): void} {
  const MIN = 14, MAX = 34;
  const set = (v: number): void => {
    const put = Math.min(MAX, Math.max(MIN, v));
    root.style.setProperty('--rail', put.toFixed(3) + 'rem');
    node.setAttribute('aria-valuenow', String(Math.round(put)));
  };
  node.setAttribute('aria-valuemin', String(MIN));
  node.setAttribute('aria-valuemax', String(MAX));
  node.setAttribute('aria-valuenow', String(Math.round(cssNum('--rail'))));
  return draggable(node,
    e => {
      const box = node.parentElement?.getBoundingClientRect();
      return box ? (e.clientX - box.left) / 16 : MIN;
    },
    d => set(cssNum('--rail') + d * 2),
    set);
}
