/* A List target: a delimited file kept as its cells, and which column is X, Z and Facing. */
import {snapF32} from '../lib/f32';

/** A column index per role, -1 for none. `head` is whether the first row names the columns. */
export interface List {cells: string[][]; head: boolean; x: number; z: number; f: number}
export type Role = 'x' | 'z' | 'f';
export const ROLES: readonly Role[] = ['x', 'z', 'f'];

export const noList = (): List => ({cells: [], head: false, x: -1, z: -1, f: -1});

/** A cell's number, `null` for a blank or anything else. */
export function num(cell: string | undefined): number | null {
  const v = (cell ?? '').trim();
  if (v === '') return null;
  const n = Number(v);
  return Number.isFinite(n) ? n : null;
}

/** One line's cells. A quoted cell may hold the delimiter, and `""` inside it is a quote. */
function split(line: string, by: string): string[] {
  const out: string[] = [];
  let cell = '', quoted = false;
  for (let i = 0; i < line.length; ++i) {
    const ch = line[i];
    if (quoted) {
      if (ch === '"' && line[i + 1] === '"') { cell += '"'; ++i; }
      else if (ch === '"') quoted = false;
      else cell += ch;
    } else if (ch === '"' && cell.trim() === '') {
      quoted = true;
      cell = '';
    } else if (ch === by) {
      out.push(cell.trim());
      cell = '';
    } else {
      cell += ch;
    }
  }
  out.push(cell.trim());
  return out;
}

/* Header names each role answers to, compared as lowercase words. */
const NAMES: Record<Role, readonly string[]> = {
  x: ['x', 'posx'],
  z: ['z', 'posz'],
  f: ['facing', 'angle', 'yaw', 'rot', 'rotation', 'dir', 'direction'],
};

/** With a header, a role takes the first column named for it or none; without one, the number
 *  columns in X, Z, Facing order. A header that names no facing has none to guess. */
function guess(cells: string[][], head: boolean): {x: number; z: number; f: number} {
  const at: Record<Role, number> = {x: -1, z: -1, f: -1};
  if (head) {
    for (const r of ROLES) {
      at[r] = cells[0].findIndex(c =>
        c.toLowerCase().split(/[^a-z]+/).some(w => NAMES[r].includes(w)));
    }
    return at;
  }
  const first = cells[head ? 1 : 0] ?? [];
  const free = first.map((_, i) => i)
    .filter(i => num(first[i]) !== null && !ROLES.some(r => at[r] === i));
  for (const r of ROLES) {
    if (at[r] < 0 && free.length) at[r] = free.shift() as number;
  }
  return at;
}

/** Tab-separated when the first line holds a tab, comma-separated otherwise. */
export function read(text: string): List {
  const lines = text.split(/\r?\n/).filter(l => l.trim() !== '');
  const by = lines.length && lines[0].includes('\t') ? '\t' : ',';
  const cells = lines.map(l => split(l, by));
  /* A first row with a word in it names the columns. */
  const head = cells.length > 0 && cells[0].some(c => c !== '' && num(c) === null);
  return {cells, head, ...guess(cells, head)};
}

/** The rows that import: X and Z both numbers, snapped to the float the game holds. */
export function rows(list: List): {x: number; z: number; f: number | null}[] {
  if (list.x < 0 || list.z < 0) return [];
  const out: {x: number; z: number; f: number | null}[] = [];
  for (const r of list.cells.slice(list.head ? 1 : 0)) {
    if (!takes(list, r)) continue;
    out.push({x: Number(snapF32(r[list.x])), z: Number(snapF32(r[list.z])),
              f: list.f >= 0 ? num(r[list.f]) : null});
  }
  return out;
}

/** Whether a row imports. */
export const takes = (list: List, row: string[]): boolean =>
  list.x >= 0 && list.z >= 0 && num(row[list.x]) !== null && num(row[list.z]) !== null;

/** The role a column has now given to it; the column that had it loses it. */
export function give(list: List, column: number, role: Role | null): void {
  for (const r of ROLES) if (list[r] === column) list[r] = -1;
  if (role) list[role] = column;
}
