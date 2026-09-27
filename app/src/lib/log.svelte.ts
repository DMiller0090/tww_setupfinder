/* The log, the floating messages, and the uncaught-error handler. */
export type Sev = 'info' | 'warning' | 'error';

export interface Entry {sev: Sev; text: string; at: Date}
export interface Note {key: number; sev: Sev; text: string; reach: string | null; stay: boolean;
                      opensLogs: boolean}

const SHOWN = 6000, FADE = 300;

export const log = $state<Entry[]>([]);
export const notes = $state<Note[]>([]);

let next = 0;

/** Optional sink for log lines (the core's log file), set from `main.ts`. */
let onward: ((sev: Sev, text: string) => void) | null = null;

export function alsoKeep(sink: (sev: Sev, text: string) => void): void {
  onward = sink;
}

/** Record a line without a note. A throwing sink is swallowed. */
export function logLine(sev: Sev, text: string): void {
  log.push({sev, text, at: new Date()});
  if (!onward) return;
  try {
    onward(sev, text);
  } catch {
    /* Reporting it would recurse. */
  }
}

/** Move the messages into the open modal dialog (top layer), else over the app. */
export function place(): void {
  const msgs = document.querySelector('.msgs');
  if (!msgs) return;
  const host = document.querySelector('dialog[open]') ?? document.querySelector('.app');
  if (host && msgs.parentElement !== host) host.appendChild(msgs);
}

function drop(key: number): void {
  const at = notes.findIndex(n => n.key === key);
  if (at >= 0) notes.splice(at, 1);
}

/**
 * Log a line and float a note. `reach` is the id of the control a click focuses; `opensLogs`
 * makes a click open the Logs. A note that stays never routes.
 */
export function say(sev: Sev, text: string, reach: string | null = null, stay = false,
                    opensLogs = false): void {
  logLine(sev, text);
  const note: Note = {key: ++next, sev, text, reach: stay ? null : reach, stay,
                      opensLogs: stay ? false : opensLogs};
  notes.push(note);
  queueMicrotask(place);
  if (stay) return;
  setTimeout(() => {
    const at = notes.find(n => n.key === note.key);
    if (at) setTimeout(() => drop(note.key), FADE);
  }, SHOWN);
}

export function close(key: number): void { drop(key); }

/** Every throw is logged; one staying note at a time. */
export function broke(detail: string): void {
  logLine('error', detail);
  if (notes.some(n => n.stay)) return;
  say('error', BROKE, null, true);
}

/* Fallback until `catchThrows` supplies the wording. */
let BROKE = 'Something broke';

export function catchThrows(wording: string): void {
  BROKE = wording;
  addEventListener('error', e => broke(e.message || wording));
  addEventListener('unhandledrejection', e => {
    const why = (e as PromiseRejectionEvent).reason;
    broke(why && why.message ? why.message : String(why));
  });
}
