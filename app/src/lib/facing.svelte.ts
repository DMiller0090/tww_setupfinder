/* Facings on the 65536 circle, shown in one app-wide base. Typed input off the circle is refused,
 * not wrapped. */
export const base = $state({hex: false});

export const wrap = (v: number): number => ((v % 65536) + 65536) % 65536;

export const showFacing = (v: number): string => base.hex
  ? wrap(v).toString(16).toUpperCase().padStart(4, '0')
  : String(wrap(v));

export function readFacing(text: string): number {
  const t = String(text).trim();
  if (!t) return NaN;
  const v = /^[0-9a-fA-F]{1,4}$/.test(t) && base.hex ? parseInt(t, 16)
    : /^\d+$/.test(t) && !base.hex ? parseInt(t, 10) : NaN;
  return (Number.isFinite(v) && v >= 0 && v <= 0xFFFF) ? v : NaN;
}
