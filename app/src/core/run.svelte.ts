/* The search, and what comes back from it. Plans are judged against the question captured when the
 * run started; what is chosen is the plan, never its position in the list. */
import {ask, stopRunning, type AskedBounds, type AskedTarget} from './ask';
import {logLine} from '../lib/log.svelte';
import {type Plan, type Stop} from './plans';

export type Phase = 'design' | 'run' | 'done';
export type Hit = 'exact' | 'hit' | 'miss';

/** Read off the controls the moment a search starts. */
export interface Asked {
  /** A freed axis is `null` at both edges. */
  box: {x0: number | null; x1: number | null; z0: number | null; z1: number | null} | null;
  tol: number;
  aim: string;
}

/** As many misses as the core sends. */
const MISSES = 20;

/** Keyed on the moves, so a plan in a fresh report stays chosen. The turn's signed steps are in it
 *  because +95 and -95 cost the same frames, and duplicate keys throw in the keyed table. */
export const planId = (p: Plan): string =>
  p.stops.map(s => `${s.move.id}:${s.frames}:${s.steps}`).join('>');

export const run = $state({
  phase: 'design' as Phase,
  running: false,
  got: [] as Plan[],
  chosen: null as Plan | null,
  pct: 0,
  /** Label and value; a set third field draws it in the warning colour. */
  figures: [] as Array<[string, string, boolean?]>,
  ended: null as 'stopped' | 'empty' | 'done' | 'broke' | null,
  asked: {box: null, tol: 0, aim: 'player'} as Asked,
});

const outside = (v: number, lo: number, hi: number): number =>
  v < lo ? lo - v : v > hi ? v - hi : 0;

export function offDist(p: Plan): number {
  const b = run.asked.box;
  if (!b) return p.d;
  /* A freed axis is not measured, as in `target.cpp`. */
  const dx = b.x0 === null || b.x1 === null ? 0 : outside(p.x, b.x0, b.x1);
  const dz = b.z0 === null || b.z1 === null ? 0 : outside(p.z, b.z0, b.z1);
  return Math.hypot(dx, dz);
}
/** The core's `verify::reaches`, repeated; the two must agree. A zero tolerance is tested on the
 *  float-step count `off`, since equal bytes can be a fraction apart in `d`. The core's own
 *  `reached` wins where sent (an address is judged on its bytes, Y included). */
export function reaches(p: Plan): Hit {
  const near = p.reached ?? (run.asked.tol > 0 ? p.d <= run.asked.tol : p.off === 0);
  return near ? (p.off === 0 ? 'exact' : 'hit') : 'miss';
}

/* `daDitem_c::set_pos`: the item rides a fixed offset turned by his facing. The core applies it. */
export const aimsAtItem = (): boolean => run.asked.aim === 'overhead';

/* By float steps (`off`), distance only breaking a tie; the core picks its misses in this order. */
const nearer = (a: Plan, b: Plan): number => a.off - b.off || offDist(a) - offDist(b);
const byFrames = (a: Plan, b: Plan): number => a.frames - b.frames || nearer(a, b);
const byClosest = (a: Plan, b: Plan): number => nearer(a, b) || a.frames - b.frames;

export function shown(): Plan[] {
  const got = run.got;
  return got.filter(p => reaches(p) !== 'miss').sort(byFrames)
    .concat(got.filter(p => reaches(p) === 'miss').sort(byClosest).slice(0, MISSES));
}

let stopping = false;

export interface Request {
  /** `cam` is the camera yaw, absent rather than zero when unread; land sticks are read in its frame. */
  start: {x: number; y?: number; z: number; f: number; cam?: number};
  target: AskedTarget;
  /** Absent for the panel's `All`. */
  facing?: {a: number; b: number | null};
  asked: Asked;
  steps: number;
  /** Plans of fewer moves are not reported; 0 also reports the start itself. */
  fewest: number;
  frames: number;
  /** Plans that take fewer frames are not reported. */
  leastFrames: number;
  room: {stage: string; room: number; pid?: number; iso?: string};
  moves: string[];
  /** Only the frames that differ from the catalogue. */
  costs: Record<string, number>;
  collision: 'none' | 'floors' | 'solid';
  bounds: AskedBounds;
  cores: number;
  checkRange: number;
  cameraChecks: boolean;
}

/**
 * Start a search and read what comes back as it arrives.
 * @param note `opensLogs` makes the note the way to the Logs.
 */
export async function start(q: Request,
                            note: (sev: 'info' | 'warning' | 'error', text: string,
                                   opensLogs?: boolean, stay?: boolean) => void,
                            words: {reach: (n: number) => string; none: string; stopped: string;
                                    nothing: string; broke: string;
                                    noGround: string;
                                    verified: string; engine: string;
                                    memory: string; memoryFull: string}): Promise<void> {
  stopping = false;
  let broke = false;
  let full = false;
  let noGround = false;
  run.asked = q.asked;
  run.got = [];
  run.chosen = null;
  run.pct = 0;
  run.ended = null;
  run.running = true;
  run.phase = 'run';
  run.figures = [];

  try {
    for await (const line of ask({ask: 'search', start: q.start, target: q.target,
                                  ...(q.facing ? {facing: q.facing} : {}),
                                  steps: q.steps, fewest: q.fewest, frames: q.frames,
                                  leastFrames: q.leastFrames, tol: q.asked.tol,
                                  moves: q.moves, costs: q.costs, collision: q.collision,
                                  bounds: q.bounds,
                                  cores: q.cores, checkRange: q.checkRange,
                                  cameraChecks: q.cameraChecks,
                                  aim: q.asked.aim === 'overhead' ? 'overhead' : 'player',
                                  ...q.room})) {
      /* A stopped run is still read to its `done`; breaking out would leave the core walking. */
      if (line.line === 'data' && line.of === 'plans') {
        const report = line.body as {plans: Plan[]; pct: number; searched: number; rate: number;
                                     verified?: number; engine?: number; memory?: number};
        run.got = report.plans;
        if (run.chosen) {
          const was = planId(run.chosen);
          run.chosen = report.plans.filter(p => planId(p) === was)[0] ?? run.chosen;
        }
        run.pct = report.pct;
        run.figures = [
          ['Searched', report.searched.toLocaleString()],
          ['Rate', `${Math.round(report.rate).toLocaleString()}/s`],
          [words.verified, `${Math.round(report.verified ?? 0).toLocaleString()}/s`],
          [words.engine, `${Math.round(report.engine ?? 0)}%`],
          /* At a hundred the run is no longer exhaustive. */
          ...(report.memory === undefined ? [] : [[words.memory, `${Math.floor(report.memory)}%`,
                                                  report.memory >= 100] as [string, string, boolean]]),
          ['Reach', String(run.got.filter(p => reaches(p) !== 'miss').length)],
        ];
        if (!full && (report.memory ?? 0) >= 100) {
          full = true;
          note('warning', words.memoryFull, false, true);
        }
        const on = shown();
        if (run.chosen && !on.some(p => planId(p) === planId(run.chosen!))) run.chosen = null;
        if (!run.chosen && on.length) run.chosen = on[0];
      }
      if (line.line === 'fail') {
        broke = true;
        noGround = line.code === 'noTargetGround';
        logLine('error', noGround ? words.noGround : line.why);
      }
    }
  } finally {
    run.running = false;
  }

  const found = run.got.filter(p => reaches(p) !== 'miss').length;
  /* Someone who went back to the question stays there. */
  run.phase = (run.phase as Phase) === 'design' ? 'design' : 'done';
  if (stopping) {
    run.ended = 'stopped';
    note('info', words.stopped);
    return;
  }
  if (broke) {
    run.ended = 'broke';
    if (noGround) note('error', words.noGround);
    else note('error', words.broke, true);
    return;
  }
  /* The bar is left where the core's last report put it. */
  if (!run.got.length) {
    run.ended = 'empty';
    note('info', words.nothing);
    return;
  }
  run.ended = 'done';
  run.figures = [
    ['Reach', String(found)], ['Miss', String(run.got.length - found)],
    ...run.figures.filter(([k]) => k === 'Searched' || k === 'Rate' || k === words.verified ||
                                   k === words.engine || k === words.memory),
  ];
  note('info', found ? words.reach(found) : words.none);
}

/** Ends what the window reads and tells the child to stop walking. */
export function stop(): void {
  stopping = true;
  stopRunning();
}

export type {Plan, Stop};
