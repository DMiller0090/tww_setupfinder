export interface Stop {move: {n: string; id: string; f: number}; frames: number; total: number;
                       x: number; z: number; f: number;
                       /** Absent: the room finds the floor itself. */
                       y?: number;
                       /** Turn steps this stop stands for; zero on a pressed move. */
                       steps: number}
/** `ox` and `oz` are each axis's share of `off`. */
export interface Plan {frames: number; off: number; ox?: number; oz?: number; x: number; z: number;
                      stops: Stop[]; d: number; reached?: boolean}
