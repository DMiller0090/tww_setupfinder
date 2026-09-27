/* Coordinates are shown as the float the game holds, e.g. `0.1` becomes 0.100000001490116. */

/** The shortest decimal that reads back as `v`'s nearest float. */
export function f32Text(v: number): string {
  const f = Math.fround(v);
  for (let digits = 1; digits <= 9; ++digits) {
    const back = Number(f.toPrecision(digits));
    if (Math.fround(back) === f) return String(back);
  }
  return String(f);
}

/** A box's text snapped to the nearest float; non-numbers are left as typed. */
export function snapF32(text: string): string {
  const v = text.trim();
  if (v === '') return text;
  const n = Number(v);
  return Number.isFinite(Math.fround(n)) ? f32Text(n) : text;
}
