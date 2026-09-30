/* The real core: the C++ child behind the seam in `ask.ts`. A room question goes down only when
 * it names a pid or an ISO path; one that names neither is answered empty. */
import {invoke} from '@tauri-apps/api/core';
import {listen} from '@tauri-apps/api/event';
import type {Core, Line, PlanRead, Question, SearchRead} from './ask';
import {t} from '../lib/strings';
import {MOVES} from './moves';
import type {Plan, Stop} from './plans';

/** Must match `core.rs`. */
const EVENT = 'core://line';

const MINE: ReadonlySet<string> = new Set(['machine', 'emulators', 'start', 'settings', 'remember']);

const WHEN_ATTACHED: ReadonlySet<string> = new Set(['rooms', 'room', 'targetGround']);

const SEARCHES: ReadonlySet<string> = new Set(['search']);

export const inWindow = (): boolean =>
  typeof window !== 'undefined' && '__TAURI_INTERNALS__' in window;

/* The shell serialises questions; the id only routes a line back to its asker. */
type Sink = (line: Line) => void;
const waiting = new Map<string, Sink>();
let listening: Promise<unknown> | null = null;
let asked = 0;

function ready(): Promise<unknown> {
  if (!listening) {
    listening = listen<{id: string; line: string}>(EVENT, event => {
      const sink = waiting.get(event.payload.id);
      if (!sink) return;
      try {
        sink(JSON.parse(event.payload.line) as Line);
      } catch {
        sink({line: 'fail', why: 'the core wrote something that is not a line'});
      }
    });
  }
  return listening;
}

async function* down(question: Question): AsyncIterable<Line> {
  const id = String(++asked);
  const box = {
    lines: [] as Line[],
    over: false,
    why: null as string | null,
    wake: null as (() => void) | null,
  };
  waiting.set(id, line => {
    box.lines.push(line);
    box.wake?.();
  });
  try {
    await ready();
    /* Settles after every line has arrived; a rejection is the shell failing. */
    void invoke('ask', {id, request: JSON.stringify(question)}).then(
      () => { box.over = true; },
      (e: unknown) => { box.why = String(e); box.over = true; },
    ).finally(() => box.wake?.());

    for (;;) {
      while (box.lines.length) {
        const line = box.lines.shift()!;
        yield line;
        if (line.line === 'done' || line.line === 'fail') return;
      }
      if (box.over) {
        yield {line: 'fail', why: box.why ?? 'the core answered nothing'};
        return;
      }
      /* Re-checked inside: the call can settle before the resolver is set. */
      await new Promise<void>(resolve => {
        box.wake = resolve;
        if (box.lines.length || box.over) resolve();
      });
    }
  } finally {
    waiting.delete(id);
  }
}

/* An unknown id stands in for its name rather than dropping the plan. */
const NAMED: Record<string, {n: string; id: string; f: number}> = (() => {
  const out: Record<string, {n: string; id: string; f: number}> = {};
  for (const k in MOVES) for (const m of MOVES[k]) out[m.id] = {n: m.name, id: m.id, f: m.frames};
  return out;
})();

/** A turn's signed step count and an L chain's taps are part of its input, so they go in the name. */
function named(id: string, steps: number, taps: number,
               frames: number): {n: string; id: string; f: number} {
  const m = NAMED[id] ?? {n: id, id, f: frames};
  if (taps > 0) return {...m, n: t.lTaps(m.n, taps)};
  return steps !== 0 ? {...m, n: t.turnSteps(m.n, steps)} : m;
}

function asPlan(read: PlanRead): Plan {
  const stops: Stop[] = read.stops.map(at => ({
    move: named(at.id, at.steps, at.taps ?? 0, at.frames),
    frames: at.frames, total: at.total, x: at.x, z: at.z, f: at.f, steps: at.steps,
    taps: at.taps ?? 0,
    ...(at.y !== undefined ? {y: at.y} : {}),
  }));
  return {frames: read.frames, off: read.off, x: read.x, z: read.z, stops, d: read.d,
          ...(read.reached !== undefined ? {reached: read.reached} : {}),
          ...(read.ox !== undefined && read.oz !== undefined ? {ox: read.ox, oz: read.oz} : {})};
}

/** Every line of a search, with the plans named and the counters sent to `log`. */
async function* searched(question: Question, log: (line: string) => void): AsyncIterable<Line> {
  for await (const line of down(question)) {
    if (line.line !== 'data' || line.of !== 'plans') {
      yield line;
      continue;
    }
    const read = line.body as SearchRead;
    if (read.counted) log(JSON.stringify(read.counted));
    yield {line: 'data', of: 'plans', body: {
      plans: read.plans.map(asPlan), pct: read.pct, searched: read.searched, rate: read.rate,
      verified: read.verified, engine: read.engine,
      ...(read.memory !== undefined ? {memory: read.memory} : {}),
    }};
  }
}

const live = (question: Question): boolean =>
  'pid' in question && typeof question.pid === 'number';

async function* unanswered(question: Question): AsyncIterable<Line> {
  if (question.ask === 'rooms' || question.ask === 'actors') {
    yield {line: 'data', of: question.ask, body: []};
  }
  yield {line: 'done'};
}

const onDisc = (question: Question): boolean =>
  'iso' in question && typeof question.iso === 'string' && question.iso !== '';

export function realCore(log: (line: string) => void = () => {}): Core {
  return {
    ask(question: Question): AsyncIterable<Line> {
      if (SEARCHES.has(question.ask)) return searched(question, log);
      const mine = MINE.has(question.ask) ||
        (WHEN_ATTACHED.has(question.ask) && (live(question) || onDisc(question)));
      return mine ? down(question) : unanswered(question);
    },
    /* Signals bypass `ask`, which is held for as long as a search runs. Not awaited, so a dead
       core cannot hang the window. */
    stop(): void {
      void invoke('signal', {line: JSON.stringify({signal: 'stop'})});
    },
    keepLog(text: string): void {
      void invoke('signal', {line: JSON.stringify({signal: 'log', text})});
    },
    leaveGround(): void {
      void invoke('signal', {line: JSON.stringify({signal: 'ground'})});
    },
  };
}
