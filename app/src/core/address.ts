/* An address as a target: four big-endian float bytes an axis; a blank (`null`) byte is undecided,
 * so every value it could hold stays in the target. */
export const AXES = ['X', 'Y', 'Z'] as const;

export type Bytes = Array<number | null>;

const dv = new DataView(new ArrayBuffer(4));

export const toBytes = (v: number): number[] => {
  dv.setFloat32(0, v);
  return [0, 1, 2, 3].map(i => dv.getUint8(i));
};
export const toFloat = (bs: number[]): number => {
  bs.forEach((b, i) => dv.setUint8(i, b));
  return dv.getFloat32(0);
};

export const hex2 = (b: number): string => b.toString(16).toUpperCase().padStart(2, '0');

export type Span = {exact: number} | {lo: number; hi: number} | {none: true};

const word = new Uint32Array(1);
const wordAsFloat = new Float32Array(word.buffer);

/* Cached per pattern; callers never edit a span. */
const spans = new Map<string, Span>();
const MOST_KEPT = 64;

/** The float the bytes are, or the lowest and highest finite float they can be.
 *  The value is monotonic in the low two bytes, so only their ends are tried; the top two are walked.
 *  The bytes are copied out first so the loop does not read through Svelte's proxy. */
export function spanOf(bytes: Bytes): Span {
  const bs = [bytes[0], bytes[1], bytes[2], bytes[3]];
  const key = bs.join();
  const kept = spans.get(key);
  if (kept) return kept;
  let out: Span;
  if (bs.every(b => b !== null)) {
    out = {exact: toFloat(bs as number[])};
  } else {
    const opt = (i: number): number[] => bs[i] !== null ? [bs[i] as number]
      : (i < 2 ? Array.from({length: 256}, (_, k) => k) : [0, 255]);
    const o0 = opt(0), o1 = opt(1), o2 = opt(2), o3 = opt(3);
    let lo = Infinity, hi = -Infinity, any = false;
    for (const b0 of o0) for (const b1 of o1) {
      const top = (b0 << 24 | b1 << 16) >>> 0;
      for (const b2 of o2) for (const b3 of o3) {
        word[0] = (top | b2 << 8 | b3) >>> 0;
        const v = wordAsFloat[0];
        if (!Number.isFinite(v)) continue;
        any = true;
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    out = any ? {lo, hi} : {none: true};
  }
  if (spans.size >= MOST_KEPT) spans.delete(spans.keys().next().value as string);
  spans.set(key, out);
  return out;
}

/** An axis with no byte typed: every value of that axis. Not the largest finite float, whose box
 *  corners would overflow the GPU's float32. */
export const freeAxis = (bytes: Bytes): boolean => bytes.every(b => b === null);

/** Where the address points, as a box on the floor plane (Y ignored). A freed axis is `null` at both edges. */
export function addrBox(addr: Bytes[], fallback: {x: number; z: number}):
    {x0: number | null; x1: number | null; z0: number | null; z1: number | null} {
  const s = addr.map(spanOf);
  const end = (a: number, fb: number): [number | null, number | null] => {
    if (freeAxis(addr[a])) return [null, null];
    const it = s[a];
    if ('exact' in it) return [it.exact, it.exact];
    if ('none' in it) return [fb, fb];
    return [it.lo, it.hi];
  };
  const x = end(0, fallback.x), z = end(2, fallback.z);
  return {x0: x[0], x1: x[1], z0: z[0], z1: z[1]};
}

type Box = {x0: number | null; x1: number | null; z0: number | null; z1: number | null};

/** The address cut to the Range tab's box, when all four of its edges are typed. A freed axis, or an
 *  address wholly outside the range, takes the range itself; the core still cuts it to the bytes. */
export function withinRange(box: Box, range: Box): Box {
  if (range.x0 === null || range.x1 === null || range.z0 === null || range.z1 === null) return box;
  const cut = (lo: number | null, hi: number | null, r0: number, r1: number):
      [number, number] => {
    const rlo = Math.min(r0, r1), rhi = Math.max(r0, r1);
    if (lo === null || hi === null) return [rlo, rhi];
    const a = Math.min(lo, hi), b = Math.max(lo, hi);
    if (b < rlo || a > rhi) return [rlo, rhi];
    return [Math.max(a, rlo), Math.min(b, rhi)];
  };
  const [x0, x1] = cut(box.x0, box.x1, range.x0, range.x1);
  const [z0, z1] = cut(box.z0, box.z1, range.z0, range.z1);
  return {x0, x1, z0, z1};
}

/** The height span the Y bytes allow; `null` when Y is free or describes no number. */
export function addrY(addr: Bytes[]): {y0: number; y1: number} | null {
  if (freeAxis(addr[1])) return null;
  const it = spanOf(addr[1]);
  if ('exact' in it) return Number.isFinite(it.exact) ? {y0: it.exact, y1: it.exact} : null;
  if ('none' in it) return null;
  return Number.isFinite(it.lo) && Number.isFinite(it.hi) ? {y0: it.lo, y1: it.hi} : null;
}

/** Whether the typed bytes, read as the twelve in memory order, are one unbroken run of at most four.
 *  Nothing typed counts. Matches the core's `Mask::run`. */
export const MOST_RUN = 4;
export function isRun(addr: Bytes[]): boolean {
  const typed = addr.flat().map((b, i) => b === null ? -1 : i).filter(i => i >= 0);
  return typed.length === 0 || (typed.length <= MOST_RUN &&
                                typed[typed.length - 1] - typed[0] === typed.length - 1);
}

/** The bytes themselves, for the core to cut the floor to: the spans are only their hull, which is
 *  exact only while the blank bytes are the low ones. `null` when no byte is typed. */
export function addrMask(addr: Bytes[]): {x: Bytes; y: Bytes; z: Bytes} | null {
  if (addr.every(axis => freeAxis(axis))) return null;
  return {x: [...addr[0]], y: [...addr[1]], z: [...addr[2]]};
}
