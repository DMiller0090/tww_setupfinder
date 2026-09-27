/* Which moves the search may use. The presses cap excludes without removing, so raising it again
 * gives back exactly what was checked. */
import {MOVES, TYPES, type Move, type MoveType} from './moves';

export type SwordFilter = 'away' | 'out' | 'both';

export const ALL = '*';

export const catalog = $state({
  moves: MOVES,
  sword: 'both' as SwordFilter,
  cap: 4,
});

export const equipped = (m: Move): boolean =>
  m.sword === 'any' || catalog.sword === 'both' || m.sword === catalog.sword;
export const capped = (m: Move): boolean => (m.presses ?? 0) > catalog.cap;
export const here = (m: Move): boolean => equipped(m) && !capped(m);

export const movesOf = (T: MoveType): Move[] => catalog.moves[T.k] ?? [];
export const shownIn = (T: MoveType): Move[] => movesOf(T).filter(equipped);
export const swordType = (T: MoveType): boolean => movesOf(T).some(m => m.sword === 'out');
/* Counts moves, not rows: one row can stand for many. */
export const weigh = (ms: Move[]): number => ms.reduce((s, m) => s + m.stands, 0);
export const inType = (T: MoveType): Move[] => movesOf(T).filter(here);
export const chosen = (T: MoveType): Move[] => inType(T).filter(m => m.on);

/* Taken at load, because `catalog.moves` is `MOVES` edited in place. */
const MEASURED: Record<string, number> = Object.fromEntries(
  Object.values(MOVES).flatMap(row => row.map(m => [m.id, m.frames] as [string, number])));

/** Only the chosen moves whose frames were retimed; the core prices the rest itself. */
export function costs(): Record<string, number> {
  const out: Record<string, number> = {};
  for (const T of TYPES) {
    for (const m of chosen(T)) if (m.frames !== MEASURED[m.id]) out[m.id] = m.frames;
  }
  return out;
}
export const typeLive = (T: MoveType): number => weigh(inType(T));

export function total(): number {
  return TYPES.reduce((s, T) => s + weigh(chosen(T)), 0);
}
