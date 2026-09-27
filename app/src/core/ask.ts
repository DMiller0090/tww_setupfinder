/* The seam: one request in, a stream of lines back, as newline-delimited JSON over the pipe of a
 * core child the window process owns. */

export type Question =
  /* With a pid the core reads the attached game's MEM1; with an ISO path, that disc. */
  | {ask: 'rooms'; pid?: number; iso?: string}
  | {ask: 'room'; stage: string; room: number; pid?: number; iso?: string}
  /* Whether this machine can read another process's memory (macOS cannot). */
  | {ask: 'machine'}
  | {ask: 'emulators'}
  | {ask: 'actors'; stage: string; room: number}
  | {ask: 'start'; pid: number}
  /* The core keeps one opaque blob. `remember` replaces all of it, so a dropped field is cleared.
     A file that will not read comes back as a `fail`. */
  | {ask: 'settings'}
  | {ask: 'remember'; body: Record<string, unknown>}
  /* `y` is absent rather than zero when nothing has read it. `tol` is what a candidate is tested
     against and what the search quantises by. */
  | {ask: 'search'; start: {x: number; y?: number; z: number; f: number; cam?: number};
     target: AskedTarget; facing?: {a: number; b: number | null};
     steps: number; frames: number; tol: number;
     /* Changes how long a run takes, never what comes back. */
     cores: number;
     /* A multiple of the smallest step a position can move by, not a distance. */
     checkRange: number;
     moves: string[]; costs: Record<string, number>; collision: 'none' | 'floors' | 'solid';
     bounds: AskedBounds;
     /* The item's offset turns with the final facing, so only the core can apply it. */
     aim: 'player' | 'overhead';
     stage: string; room: number; pid?: number; iso?: string}
  /* Asked of the core rather than worked out here, so the room lights only ground the search keeps. */
  | {ask: 'targetGround'; start: {x: number; z: number; f: number};
     target: AskedTarget; tol: number; aim: 'player' | 'overhead';
     stage: string; room: number; pid?: number; iso?: string};

/** The room's ground triangles clipped to the target: `sides` vertices each (3 to 9), `verts` x, y, z
 *  for every vertex in turn. Each polygon lies in its triangle's plane. */
export interface TargetGround {
  kept: number;
  sides: number[];
  verts: number[];
}

/** An open side is `null`, not the number left in its greyed-out box. */
export interface AskedBounds {
  xmin: number | null; xmax: number | null; zmin: number | null; zmax: number | null;
}

/** A freed axis is `null`. Only an address carries a height (`y0`/`y1`). */
export type AskedTarget =
  | {shape: 'point'; x: number | null; z: number | null}
  | {shape: 'range'; x0: number | null; x1: number | null;
     z0: number | null; z1: number | null;
     y0?: number; y1?: number;
     /* The typed address bytes, high byte first; the edges above are only their hull. */
     mask?: {x: Array<number | null>; y: Array<number | null>; z: Array<number | null>};
     /* Cuts the region the core keeps, never the box above, which the corridor is laid over. */
     within?: {x0: number; x1: number; z0: number; z1: number}};

/** `done` ends the stream; `fail` ends it badly. Anything else is progress. */
export type Line =
  | {line: 'data'; of: string; body: unknown}
  | {line: 'done'}
  /** `why` goes to the Logs only; a known `code` gets the window's own text. */
  | {line: 'fail'; why: string; code?: string};

export interface Machine {reads: boolean; cores: number}

/** Collision as nine floats a triangle, in the three roles the game's own test uses. */
export interface RoomRead {
  stage: string;
  room: number;
  ground: ArrayLike<number>;
  wall: ArrayLike<number>;
  roof: ArrayLike<number>;
  /** The sea stage's ocean floor, kept out of `ground`: drawn, never stood on. */
  seabed?: ArrayLike<number>;
}

/** `y` is absent rather than zero when nothing has read it. */
export interface StartRead {x: number; y?: number; z: number; facing: number; camera: number}

/** Stops name a move by ID; `real.ts` maps it to a name. */
export interface PlanRead {
  frames: number;
  off: number;
  /** Each axis's share of `off`, signed: positive past the target, negative short of it. */
  ox?: number;
  oz?: number;
  x: number;
  z: number;
  d: number;
  reached?: boolean;
  stops: Array<{id: string; steps: number; taps?: number; frames: number; total: number;
                x: number; z: number; f: number; y?: number}>;
}
export interface SearchRead {
  plans: PlanRead[];
  pct: number;
  searched: number;
  rate: number;
  /** Plans put through the engine a second, and the share of the walkers' time that took. */
  verified?: number;
  engine?: number;
  /** The shortlist's share of its memory, in percent; absent where the machine will not say. */
  memory?: number;
  counted?: Record<string, unknown> & {unstepped?: string[]};
}

export interface Core {
  /** The stream always ends with `done` or `fail`. */
  ask(question: Question): AsyncIterable<Line>;
  /** A signal, not a question: the core would only read a question after the search it meant to stop. */
  stop?(): void;
  /** A signal for the same reason: log lines arrive while a search runs. Nothing comes back. */
  keepLog?(text: string): void;
  /** The room's question changed, so the cut in progress is not wanted. */
  leaveGround?(): void;
}

/** Set once, at start-up, by `main.ts`. */
let core: Core | null = null;

export function useCore(it: Core): void {
  core = it;
}

export function ask(question: Question): AsyncIterable<Line> {
  if (!core) throw new Error('no core is wired up');
  return core.ask(question);
}

export function stopRunning(): void {
  core?.stop?.();
}

export function leaveGround(): void {
  core?.leaveGround?.();
}

export function keepLog(text: string): void {
  core?.keepLog?.(text);
}

/** Read a whole stream into a list; throws on `fail`. */
export async function askAll(question: Question): Promise<unknown[]> {
  const out: unknown[] = [];
  for await (const line of ask(question)) {
    if (line.line === 'fail') throw new Error(line.why);
    if (line.line === 'data') out.push(line.body);
  }
  return out;
}
