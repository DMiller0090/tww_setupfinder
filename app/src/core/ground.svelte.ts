/* Where the target meets the floor, asked of the core (`seam.cpp`'s `target_ground`) so the room
 * lights only ground the search would keep. Replies can arrive out of order; only the newest ask
 * is kept. */
import {askAll, leaveGround} from './ask';
import type {AskedTarget, TargetGround} from './ask';
import {logLine} from '../lib/log.svelte';

/** Raw, not deep: an answer is replaced whole and can hold a hundred thousand numbers. */
class Ground {
  at = $state.raw<TargetGround | null>(null);
  /** A question has been with the core longer than `SLOW`. */
  waiting = $state(false);
}
export const ground = new Ground();

/** Every field the core reads, so any change it would answer differently is noticed. */
export interface GroundAsk {
  target: AskedTarget;
  start: {x: number; z: number; f: number};
  tol: number;
  aim: 'player' | 'overhead';
  stage: string;
  room: number;
  pid?: number;
  iso?: string;
}

/** Debounce: a target is typed a byte at a time. */
const SETTLE = 250;

const SLOW = 200;

let timer: ReturnType<typeof setTimeout> | null = null;
let slow: ReturnType<typeof setTimeout> | null = null;

function answered(): void {
  if (slow !== null) { clearTimeout(slow); slow = null; }
  ground.waiting = false;
}
let asked = 0;
let last = '';

/** Ask again, unless the question is the one already answered. Safe to call on every change. */
export function follow(q: GroundAsk | null): void {
  const key = q ? JSON.stringify(q) : '';
  /* Return before touching the timer: the calling effect re-runs when an answer lands and would
     otherwise cancel the ask it is waiting for. */
  if (key === last) return;
  last = key;
  if (timer !== null) { clearTimeout(timer); timer = null; }
  if (!q) {
    ++asked;
    ground.at = null;
    answered();
    return;
  }
  /* Bumping `asked` drops a late answer to the old question. */
  ++asked;
  ground.at = null;
  leaveGround();
  timer = setTimeout(() => { timer = null; void run(q, ++asked); }, SETTLE);
}

async function run(q: GroundAsk, mine: number): Promise<void> {
  if (slow !== null) clearTimeout(slow);
  slow = setTimeout(() => { slow = null; if (mine === asked) ground.waiting = true; }, SLOW);
  try {
    const [got] = await askAll({ask: 'targetGround', ...q}) as TargetGround[];
    if (mine !== asked) return;
    answered();
    ground.at = got ?? null;
  } catch (e) {
    if (mine !== asked) return;
    answered();
    /* A target with no floor is refused by the core; here that just means nothing to light. */
    ground.at = null;
    logLine('info', e instanceof Error ? e.message : String(e));
  }
}
