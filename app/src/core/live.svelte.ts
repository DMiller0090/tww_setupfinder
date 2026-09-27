/* The connection to a running game. The core attaches per read, so a pid is all that is held. */
import {askAll} from './ask';
import {run} from './run.svelte';
import {logLine} from '../lib/log.svelte';
import type {Machine} from './ask';
import type {Emulator} from './fixtures';

const BEAT = 240;

/* `reads` starts true so `Connect` does not flash grey at launch where it works. */
export const live = $state<{at: Emulator | null; busy: boolean; list: Emulator[]; reads: boolean;
                            cores: number}>({
  at: null,
  busy: false,
  list: [],
  reads: true,
  cores: 1,
});

export async function askMachine(): Promise<void> {
  try {
    const got = (await askAll({ask: 'machine'}))[0] as Machine | undefined;
    live.reads = got ? got.reads : false;
    live.cores = got && got.cores > 0 ? Math.floor(got.cores) : 1;
  } catch (e) {
    live.reads = false;
    live.cores = 1;
    logLine('error', e instanceof Error ? e.message : String(e));
  }
}

export const attached = (): boolean => live.at !== null;

export interface Words {
  noEmulator: string;
  noGame: string;
  pauseFirst: string;
  readFrom: (game: string) => string;
  disconnected: string;
}

type Say = (sev: 'info' | 'warning' | 'error', text: string) => void;

/** Read the start out of the attached game. True when the four numbers landed. */
type Read = () => Promise<boolean>;

const wait = (ms: number): Promise<void> => new Promise(r => setTimeout(r, ms));

/**
 * Press `Connect`, or `Disconnect`.
 *
 * @param open called when several emulators are running; the surface chooses one and calls `connect`.
 */
export async function toggle(say: Say, words: Words, read: Read,
                             open: () => void): Promise<void> {
  if (live.at) { detach(say, words); return; }
  live.busy = true;
  await wait(BEAT);
  live.busy = false;
  if (!(await refresh())) { say('error', words.noGame); return; }
  const it = usable();
  if (!it.length) {
    say('warning', live.list.length ? words.noGame : words.noEmulator);
    return;
  }
  if (it.length === 1) { await connect(it[0], say, words, read); return; }
  open();
}

/* False when the core refused the question; the reason goes to the Logs. */
export async function refresh(): Promise<boolean> {
  try {
    live.list = ((await askAll({ask: 'emulators'}))[0] as Emulator[]) ?? [];
    return true;
  } catch (e) {
    live.list = [];
    logLine('error', e instanceof Error ? e.message : String(e));
    return false;
  }
}

export const usable = (): Emulator[] => live.list.filter(e => e.game && e.reads);

export const canConnect = (): boolean => live.reads && usable().length > 0;

/* Polled while focused and detached, so `Connect` is not greyed on a stale list. */
const BEAT_LIST = 2000;

export function watch(): () => void {
  let timer = 0;
  /* Never mid-run: the core answers one question at a time, and queued polls could starve Stop. */
  const tick = (): void => {
    if (!live.at && !live.busy && !run.running && document.hasFocus()) void refresh();
  };
  const start = (): void => { if (!timer) timer = window.setInterval(tick, BEAT_LIST); };
  const stop = (): void => { window.clearInterval(timer); timer = 0; };
  window.addEventListener('focus', () => { tick(); start(); });
  window.addEventListener('blur', stop);
  if (document.hasFocus()) start();
  void refresh();
  return stop;
}

export async function connect(e: Emulator, say: Say, words: Words, read: Read): Promise<void> {
  /* A running game mixes frames when read. */
  if (!e.paused) { say('warning', words.pauseFirst); return; }
  live.busy = true;
  await wait(BEAT);
  live.at = e;
  const landed = await read();
  live.busy = false;
  if (!landed) { live.at = null; return; }
  say('info', words.readFrom(e.game ?? ''));
}

/* Keeps the start's numbers; only the claim that they are live goes. */
export function detach(say: Say, words: Words): void {
  live.at = null;
  say('warning', words.disconnected);
}

/** The one the environment names comes first. */
export const offered = (): Emulator[] =>
  live.list.filter(e => e.game).slice().sort((a, b) => (b.pref ? 1 : 0) - (a.pref ? 1 : 0));
