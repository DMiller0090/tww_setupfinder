export type Kind = 'ground' | 'wall' | 'roof';

/** In the order the room draws them. */
export const KINDS: Kind[] = ['ground', 'wall', 'roof'];

export interface RoomIndex {
  stage: string;
  room: number;
  tris: number;
  box: [number, number, number, number, number, number];
  counts: Record<Kind, number>;
}

/* One sitting at its own menu is never offered. */
export interface Emulator {
  pid: number;
  game: string | null;
  paused: boolean;
  pref?: boolean;
  /* Whether this build can read it (never on macOS). Unlike `paused`, false means it can never be
     taken, so anything filtering what can be taken must read this. */
  reads: boolean;
}

export interface Actor {x: number; z: number; r: number; h: number}
