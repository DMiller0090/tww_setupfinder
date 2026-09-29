/* What the app remembers. The core keeps one opaque blob; every field name is this file's. Written
 * a beat after each change; a section whose switch is off is left out of the body, which clears it.
 * The search habits are kept once, and each room's question under that room. */
import {ask, askAll} from './ask';
import {logLine} from '../lib/log.svelte';
import {snapF32} from '../lib/f32';
import {base} from '../lib/facing.svelte';
import {catalog, type SwordFilter} from './catalog.svelte';
import {MOVES, TYPES, COMBO_MAX} from './moves';
import {toBytes, type Bytes} from './address';
import {start, target} from '../fixtures/start';
import type {Side} from '../room/marks';

/* Taken at load, because `catalog.moves` is `MOVES` edited in place. */
const DEFAULTS: Record<string, {id: string; frames: number}[]> =
  Object.fromEntries(Object.entries(MOVES)
    .map(([key, row]) => [key, row.map(m => ({id: m.id, frames: m.frames}))]));

const SETTLE = 250;

type Ground = 'none' | 'floors' | 'solid';
type Shape = 'point' | 'range' | 'addr';
type FacingMode = 'any' | 'single' | 'range';

export const settings = $state({
  cores: 1,
  /** A multiple of the smallest position step, not a distance. Kept outside both switches. */
  checkRange: 4,
  /** Whether camera moves are refused where the room could block the camera. */
  cameraChecks: false,
  /** Mirrored into `base.hex` by `tookHex`. */
  hex: false,
  keepMoves: true,
  restore: true,
  keepPlans: true,
  /** Checked in `App.svelte` before it is believed; a disc that no longer reads is cleared. */
  iso: '',

  ground: 'none' as Ground,
  /** Needs walls. */
  sword: true,
  /** Needs something to stand on. */
  actors: false,
  aim: 'player',
  steps: 4,
  frames: 120,

  lastRoom: '',
  sx: start.x,
  /** Never a control: read from the game, and emptied when a spot is typed. */
  sy: '',
  sz: start.z,
  sf: start.facing,
  scam: start.camera,
  shape: 'point' as Shape,
  fmode: 'single' as FacingMode,
  tx: target.tx,
  tz: target.tz,
  tol: target.tol,
  anyx: false,
  anyz: false,
  rx1: '',
  rx2: '',
  rz1: '',
  rz2: '',
  /** Whether the Range tab's box cuts the address. */
  addrRange: false,
  fA: target.fA,
  fB: target.fB,
  addr: [toBytes(Number(target.tx)), toBytes(0), toBytes(Number(target.tz))] as Bytes[],
  bOpen: {xmin: true, xmax: true, zmin: true, zmax: true} as Record<Side, boolean>,
  bValue: {xmin: '', xmax: '', zmin: '', zmax: ''} as Record<Side, string>,
});

/** Deviations from the catalogue only, so a move added later starts on. */
interface KeptMoves {
  sword: SwordFilter;
  cap: number;
  off: string[];
  types: string[];
  frames: Record<string, number>;
}

interface Question {
  sx: string; sy: string; sz: string; sf: string; scam: string;
  shape: Shape; fmode: FacingMode;
  tx: string; tz: string; tol: string;
  anyx: boolean; anyz: boolean;
  rx1: string; rx2: string; rz1: string; rz2: string;
  addrRange: boolean;
  fA: string; fB: string;
  addr: Bytes[];
  bOpen: Record<Side, boolean>;
  bValue: Record<Side, string>;
}

interface KeptSession {
  ground: Ground;
  sword: boolean;
  actors: boolean;
  aim: string;
  steps: number;
  frames: number;
  lastRoom: string;
  rooms: Record<string, Question>;
}

interface Kept {
  cores?: number;
  checkRange?: number;
  cameraChecks?: boolean;
  hex?: boolean;
  keepMoves?: boolean;
  restore?: boolean;
  keepPlans?: boolean;
  iso?: string;
  moves?: KeptMoves;
  session?: KeptSession;
}

/* Whether the start came from the running game, so a kept start does not overwrite it. */
export const fromGame = $state({start: false});

/* Every room but the one on screen, which lives in `settings`. */
let rooms: Record<string, Question> = {};

export const roomKey = (stage: string, room: number): string => stage + '/' + room;

/* A field of the wrong shape is ignored on its own, keeping the default. */
const aNumber = (v: unknown, fallback: number): number =>
  typeof v === 'number' && Number.isFinite(v) ? v : fallback;
const aFlag = (v: unknown, fallback: boolean): boolean =>
  typeof v === 'boolean' ? v : fallback;
const aWord = (v: unknown, fallback: string): string =>
  typeof v === 'string' ? v : fallback;
const oneOf = <T extends string>(v: unknown, of: readonly T[], fallback: T): T =>
  typeof v === 'string' && (of as readonly string[]).includes(v) ? v as T : fallback;
const ids = (v: unknown): string[] =>
  Array.isArray(v) ? v.filter((i): i is string => typeof i === 'string') : [];

const SIDES: readonly Side[] = ['xmin', 'xmax', 'zmin', 'zmax'];

function takeMoves(kept: KeptMoves): void {
  if (kept.sword === 'away' || kept.sword === 'out' || kept.sword === 'both') {
    catalog.sword = kept.sword;
  }
  const cap = aNumber(kept.cap, catalog.cap);
  if (cap >= 0 && cap <= COMBO_MAX) catalog.cap = cap;

  const off = new Set(ids(kept.off));
  for (const row of Object.values(catalog.moves)) {
    for (const move of row) move.on = !off.has(move.id);
  }
  const typesOff = new Set(ids(kept.types));
  for (const type of TYPES) type.on = !typesOff.has(type.k);

  const frames = kept.frames;
  if (frames && typeof frames === 'object') {
    for (const row of Object.values(catalog.moves)) {
      for (const move of row) {
        const was = (frames as Record<string, unknown>)[move.id];
        if (typeof was === 'number' && Number.isInteger(was) && was >= 1) move.frames = was;
      }
    }
  }
}

/** With `keepStart`, a start read from the running game survives. */
function takeQuestion(kept: Question, keepStart: boolean): void {
  if (!keepStart) {
    settings.sx = snapF32(aWord(kept.sx, settings.sx));
    settings.sy = snapF32(aWord(kept.sy, settings.sy));
    settings.sz = snapF32(aWord(kept.sz, settings.sz));
    settings.sf = aWord(kept.sf, settings.sf);
    settings.scam = aWord(kept.scam, settings.scam);
  }
  settings.shape = oneOf(kept.shape, ['point', 'range', 'addr'], settings.shape);
  settings.fmode = oneOf(kept.fmode, ['any', 'single', 'range'], settings.fmode);
  settings.tx = snapF32(aWord(kept.tx, settings.tx));
  settings.tz = snapF32(aWord(kept.tz, settings.tz));
  settings.tol = aWord(kept.tol, settings.tol);
  settings.anyx = aFlag(kept.anyx, settings.anyx);
  settings.anyz = aFlag(kept.anyz, settings.anyz);
  /* Both axes free is a state the panel cannot reach. */
  if (settings.anyx && settings.anyz) settings.anyz = false;
  settings.rx1 = snapF32(aWord(kept.rx1, settings.rx1));
  settings.rx2 = snapF32(aWord(kept.rx2, settings.rx2));
  settings.rz1 = snapF32(aWord(kept.rz1, settings.rz1));
  settings.rz2 = snapF32(aWord(kept.rz2, settings.rz2));
  settings.addrRange = aFlag(kept.addrRange, settings.addrRange);
  settings.fA = aWord(kept.fA, settings.fA);
  settings.fB = aWord(kept.fB, settings.fB);
  /* All or nothing: half an address is not a target. */
  if (Array.isArray(kept.addr) && kept.addr.length === settings.addr.length &&
      kept.addr.every(ax => Array.isArray(ax) && ax.length === 4 &&
        ax.every(b => b === null || (typeof b === 'number' && b >= 0 && b <= 255)))) {
    settings.addr = kept.addr.map(ax => ax.slice());
  }
  for (const side of SIDES) {
    settings.bOpen[side] = aFlag(kept.bOpen?.[side], settings.bOpen[side]);
    settings.bValue[side] = snapF32(aWord(kept.bValue?.[side], settings.bValue[side]));
  }
}

function takeSession(kept: KeptSession): void {
  settings.ground = oneOf(kept.ground, ['none', 'floors', 'solid'], settings.ground);
  settings.sword = aFlag(kept.sword, settings.sword);
  settings.actors = aFlag(kept.actors, settings.actors);
  settings.aim = aWord(kept.aim, settings.aim);
  settings.steps = aNumber(kept.steps, settings.steps);
  settings.frames = aNumber(kept.frames, settings.frames);
  rooms = kept.rooms && typeof kept.rooms === 'object' ? {...kept.rooms} : {};
  settings.lastRoom = aWord(kept.lastRoom, '');
  const had = settings.lastRoom ? rooms[settings.lastRoom] : undefined;
  if (had) takeQuestion(had, false);
}

function movesNow(): KeptMoves {
  const off: string[] = [];
  const frames: Record<string, number> = {};
  for (const [key, row] of Object.entries(catalog.moves)) {
    const was = DEFAULTS[key] ?? [];
    for (const move of row) {
      if (!move.on) off.push(move.id);
      const first = was.find(m => m.id === move.id);
      if (first && move.frames !== first.frames) frames[move.id] = move.frames;
    }
  }
  return {
    sword: catalog.sword,
    cap: catalog.cap,
    off,
    types: TYPES.filter(type => !type.on).map(type => type.k),
    frames,
  };
}

/** Copied out, so the map never shares an array with the live values. */
function questionNow(): Question {
  return {
    sx: settings.sx, sy: settings.sy, sz: settings.sz, sf: settings.sf, scam: settings.scam,
    shape: settings.shape, fmode: settings.fmode,
    tx: settings.tx, tz: settings.tz, tol: settings.tol,
    anyx: settings.anyx, anyz: settings.anyz,
    rx1: settings.rx1, rx2: settings.rx2, rz1: settings.rz1, rz2: settings.rz2,
    addrRange: settings.addrRange,
    fA: settings.fA, fB: settings.fB,
    addr: settings.addr.map(ax => ax.slice()),
    bOpen: {...settings.bOpen},
    bValue: {...settings.bValue},
  };
}

function sessionNow(): KeptSession {
  const all = {...rooms};
  if (settings.lastRoom) all[settings.lastRoom] = questionNow();
  return {
    ground: settings.ground, sword: settings.sword, actors: settings.actors, aim: settings.aim,
    steps: settings.steps, frames: settings.frames,
    lastRoom: settings.lastRoom, rooms: all,
  };
}

function body(): Kept {
  const out: Kept = {
    cores: settings.cores,
    checkRange: settings.checkRange,
    cameraChecks: settings.cameraChecks,
    hex: settings.hex,
    keepMoves: settings.keepMoves,
    restore: settings.restore,
    keepPlans: settings.keepPlans,
    iso: settings.iso,
  };
  if (settings.keepMoves) out.moves = movesNow();
  if (settings.restore) out.session = sessionNow();
  return out;
}

let pending: ReturnType<typeof setTimeout> | null = null;
/* Until the file is read, a write would save the defaults over it. */
let loaded = false;

async function put(): Promise<void> {
  try {
    for await (const line of ask({ask: 'remember', body: body() as Record<string, unknown>})) {
      if (line.line === 'fail') logLine('error', line.why);
    }
  } catch (e) {
    logLine('error', e instanceof Error ? e.message : String(e));
  }
}

export function keep(): void {
  if (!loaded) return;
  if (pending) clearTimeout(pending);
  pending = setTimeout(() => { pending = null; void put(); }, SETTLE);
}

/** The room being left keeps its question; the one entered brings its own back, or leaves the
 *  controls as they are if it has none. */
export function atRoom(key: string): void {
  if (key === settings.lastRoom) return;
  if (settings.lastRoom) rooms[settings.lastRoom] = questionNow();
  settings.lastRoom = key;
  const had = key ? rooms[key] : undefined;
  if (had) takeQuestion(had, fromGame.start);
  keep();
}

/** Read the file, once, at start-up. A failure is the defaults plus a line in the Logs. */
export async function load(): Promise<void> {
  try {
    const [got] = await askAll({ask: 'settings'});
    const kept = (got ?? {}) as Kept;
    settings.cores = aNumber(kept.cores, settings.cores);
    settings.checkRange = aNumber(kept.checkRange, settings.checkRange);
    settings.cameraChecks = aFlag(kept.cameraChecks, settings.cameraChecks);
    settings.hex = aFlag(kept.hex, settings.hex);
    settings.keepMoves = aFlag(kept.keepMoves, settings.keepMoves);
    settings.restore = aFlag(kept.restore, settings.restore);
    settings.keepPlans = aFlag(kept.keepPlans, settings.keepPlans);
    settings.iso = aWord(kept.iso, settings.iso);
    /* A section under a switch that is off is ignored. */
    if (settings.keepMoves && kept.moves && typeof kept.moves === 'object') takeMoves(kept.moves);
    if (settings.restore && kept.session && typeof kept.session === 'object') {
      takeSession(kept.session);
    }
  } catch (e) {
    logLine('error', e instanceof Error ? e.message : String(e));
  } finally {
    loaded = true;
    base.hex = settings.hex;
  }
}

export function tookHex(): void {
  base.hex = settings.hex;
  keep();
}
