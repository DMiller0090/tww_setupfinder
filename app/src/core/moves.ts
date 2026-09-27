/* The move catalogue. Frames are from a standstill on flat ground and match the engine, except the
 * slashes and combos (`changeCutProc` is recorded, not run) and the hand-set C up turn. The ids
 * match `core/src/search/catalogue.*`. `stands` is how many moves a row stands for. */
export type Sword = 'away' | 'out' | 'any';

export interface Move {
  id: string;
  name: string;
  frames: number;
  sword: Sword;
  stands: number;
  presses?: number;
  on: boolean;
  /** Priced by the core per facing; `frames` is then the table's median, which
   *  `core/tests/test_search.cpp` reads from this file. */
  fixed?: boolean;
}

export interface MoveType {k: string; n: string; on: boolean}

export const COMBO_MAX = 4;

export const TYPES: MoveType[] = [
  {k:'jumpslash', n:'Jump slashes', on:true},
  {k:'backflip', n:'Backflips', on:true},
  {k:'sidehop', n:'Sidehops', on:true},
  {k:'roll', n:'Rolls', on:true},
  {k:'slash', n:'Slashes (Target)', on:true},
  {k:'uslash', n:'Untargeted slashes', on:true},
  {k:'crawl', n:'Crawls', on:true},
  {k:'combo', n:'Combos (Target)', on:false},
  {k:'combo_none', n:'Combos (No Target)', on:false},
  {k:'combo_rest', n:'Combos (Mixed)', on:false},
  {k:'turns', n:'Turns', on:true}
];

const COMBOS: Record<string, string> = {
  combo: 'Nt+R:14:1 Nt:16:1 NtLt+R:25:2 NtNt+R:25:2 NtRt+R:25:2 NtUt+R:25:2 NtLt:26:2 NtNt:27:2 NtRt:27:2 NtUt:27:2 NtLtLt+R:36:3 NtLtNt+R:36:3 NtLtRt+R:36:3 NtLtUt+R:36:3 NtNtLt+R:36:3 NtNtNt+R:36:3 NtNtRt+R:36:3 NtNtUt+R:36:3 NtRtLt+R:36:3 NtRtNt+R:36:3 NtRtRt+R:36:3 NtRtUt+R:36:3 NtUtLt+R:36:3 NtUtNt+R:36:3 NtUtRt+R:36:3 NtUtUt+R:36:3 NtLtLt:37:3 NtNtLt:37:3 NtRtLt:37:3 NtUtLt:37:3 NtLtNt:38:3 NtLtRt:38:3 NtLtUt:38:3 NtNtNt:38:3 NtNtRt:38:3 NtNtUt:38:3 NtRtNt:38:3 NtRtRt:38:3 NtRtUt:38:3 NtUtNt:38:3 NtUtRt:38:3 NtUtUt:38:3 Nt+QS:48:1 NtLtLtUt:51:4 NtLtNtUt:51:4 NtLtRtUt:51:4 NtLtUtUt:51:4 NtNtLtUt:51:4 NtNtNtUt:51:4 NtNtRtUt:51:4 NtNtUtUt:51:4 NtRtLtUt:51:4 NtRtNtUt:51:4 NtRtRtUt:51:4 NtRtUtUt:51:4 NtUtLtUt:51:4 NtUtNtUt:51:4 NtUtRtUt:51:4 NtUtUtUt:51:4 NtLtLtNt:53:4 NtLtNtNt:53:4 NtLtRtNt:53:4 NtLtUtNt:53:4 NtNtLtNt:53:4 NtNtNtNt:53:4 NtNtRtNt:53:4 NtNtUtNt:53:4 NtRtLtNt:53:4 NtRtNtNt:53:4 NtRtRtNt:53:4 NtRtUtNt:53:4 NtUtLtNt:53:4 NtUtNtNt:53:4 NtUtRtNt:53:4 NtUtUtNt:53:4 NtLt+QS:64:2 NtNt+QS:64:2 NtRt+QS:64:2 NtUt+QS:64:2 NtLtLt+QS:73:3 NtLtNt+QS:73:3 NtLtRt+QS:73:3 NtLtUt+QS:73:3 NtNtLt+QS:73:3 NtNtNt+QS:73:3 NtNtRt+QS:73:3 NtNtUt+QS:73:3 NtRtLt+QS:73:3 NtRtNt+QS:73:3 NtRtRt+QS:73:3 NtRtUt+QS:73:3 NtUtLt+QS:73:3 NtUtNt+QS:73:3 NtUtRt+QS:73:3 NtUtUt+QS:73:3 NtLtLtNt+QS:82:4 NtLtLtUt+QS:82:4 NtLtNtNt+QS:82:4 NtLtNtUt+QS:82:4 NtLtRtNt+QS:82:4 NtLtRtUt+QS:82:4 NtLtUtNt+QS:82:4 NtLtUtUt+QS:82:4 NtNtLtNt+QS:82:4 NtNtLtUt+QS:82:4 NtNtNtNt+QS:82:4 NtNtNtUt+QS:82:4 NtNtRtNt+QS:82:4 NtNtRtUt+QS:82:4 NtNtUtNt+QS:82:4 NtNtUtUt+QS:82:4 NtRtLtNt+QS:82:4 NtRtLtUt+QS:82:4 NtRtNtNt+QS:82:4 NtRtNtUt+QS:82:4 NtRtRtNt+QS:82:4 NtRtRtUt+QS:82:4 NtRtUtNt+QS:82:4 NtRtUtUt+QS:82:4 NtUtLtNt+QS:82:4 NtUtLtUt+QS:82:4 NtUtNtNt+QS:82:4 NtUtNtUt+QS:82:4 NtUtRtNt+QS:82:4 NtUtRtUt+QS:82:4 NtUtUtNt+QS:82:4 NtUtUtUt+QS:82:4',
  combo_none: 'N+R:14:1 N:15:1 ND+R:25:2 NL+R:25:2 NN+R:25:2 NR+R:25:2 NU+R:25:2 ND:26:2 NL:26:2 NN:26:2 NR:27:2 NU:27:2 NDD+R:36:3 NDL+R:36:3 NDN+R:36:3 NDR+R:36:3 NDU+R:36:3 NLD+R:36:3 NLL+R:36:3 NLN+R:36:3 NLR+R:36:3 NLU+R:36:3 NND+R:36:3 NNL+R:36:3 NNN+R:36:3 NNR+R:36:3 NNU+R:36:3 NRD+R:36:3 NRL+R:36:3 NRN+R:36:3 NRR+R:36:3 NRU+R:36:3 NUD+R:36:3 NUL+R:36:3 NUN+R:36:3 NUR+R:36:3 NUU+R:36:3 NDD:37:3 NDL:37:3 NDN:37:3 NLD:37:3 NLL:37:3 NLN:37:3 NND:37:3 NNL:37:3 NNN:37:3 NRD:37:3 NRL:37:3 NRN:37:3 NUD:37:3 NUL:37:3 NUN:37:3 NDR:38:3 NDU:38:3 NLR:38:3 NLU:38:3 NNR:38:3 NNU:38:3 NRR:38:3 NRU:38:3 NUR:38:3 NUU:38:3 N+QS:48:1 NDDU:51:4 NDLU:51:4 NDNU:51:4 NDRU:51:4 NDUU:51:4 NLDU:51:4 NLLU:51:4 NLNU:51:4 NLRU:51:4 NLUU:51:4 NNDU:51:4 NNLU:51:4 NNNU:51:4 NNRU:51:4 NNUU:51:4 NRDU:51:4 NRLU:51:4 NRNU:51:4 NRRU:51:4 NRUU:51:4 NUDU:51:4 NULU:51:4 NUNU:51:4 NURU:51:4 NUUU:51:4 NDDN:53:4 NDLN:53:4 NDNN:53:4 NDRN:53:4 NDUN:53:4 NLDN:53:4 NLLN:53:4 NLNN:53:4 NLRN:53:4 NLUN:53:4 NNDN:53:4 NNLN:53:4 NNNN:53:4 NNRN:53:4 NNUN:53:4 NRDN:53:4 NRLN:53:4 NRNN:53:4 NRRN:53:4 NRUN:53:4 NUDN:53:4 NULN:53:4 NUNN:53:4 NURN:53:4 NUUN:53:4 ND+QS:64:2 NL+QS:64:2 NN+QS:64:2 NR+QS:64:2 NU+QS:64:2 NDD+QS:73:3 NDL+QS:73:3 NDN+QS:73:3 NDR+QS:73:3 NDU+QS:73:3 NLD+QS:73:3 NLL+QS:73:3 NLN+QS:73:3 NLR+QS:73:3 NLU+QS:73:3 NND+QS:73:3 NNL+QS:73:3 NNN+QS:73:3 NNR+QS:73:3 NNU+QS:73:3 NRD+QS:73:3 NRL+QS:73:3 NRN+QS:73:3 NRR+QS:73:3 NRU+QS:73:3 NUD+QS:73:3 NUL+QS:73:3 NUN+QS:73:3 NUR+QS:73:3 NUU+QS:73:3 NDDN+QS:82:4 NDDU+QS:82:4 NDLN+QS:82:4 NDLU+QS:82:4 NDNN+QS:82:4 NDNU+QS:82:4 NDRN+QS:82:4 NDRU+QS:82:4 NDUN+QS:82:4 NDUU+QS:82:4 NLDN+QS:82:4 NLDU+QS:82:4 NLLN+QS:82:4 NLLU+QS:82:4 NLNN+QS:82:4 NLNU+QS:82:4 NLRN+QS:82:4 NLRU+QS:82:4 NLUN+QS:82:4 NLUU+QS:82:4 NNDN+QS:82:4 NNDU+QS:82:4 NNLN+QS:82:4 NNLU+QS:82:4 NNNN+QS:82:4 NNNU+QS:82:4 NNRN+QS:82:4 NNRU+QS:82:4 NNUN+QS:82:4 NNUU+QS:82:4 NRDN+QS:82:4 NRDU+QS:82:4 NRLN+QS:82:4 NRLU+QS:82:4 NRNN+QS:82:4 NRNU+QS:82:4 NRRN+QS:82:4 NRRU+QS:82:4 NRUN+QS:82:4 NRUU+QS:82:4 NUDN+QS:82:4 NUDU+QS:82:4 NULN+QS:82:4 NULU+QS:82:4 NUNN+QS:82:4 NUNU+QS:82:4 NURN+QS:82:4 NURU+QS:82:4 NUUN+QS:82:4 NUUU+QS:82:4',
  combo_rest: 'NNt+R:25:2 NRt+R:25:2 NUt+R:25:2 NtD+R:25:2 NtL+R:25:2 NtR+R:25:2 NtU+R:25:2 NtD:26:2 NtL:26:2 NNt:27:2 NRt:27:2 NUt:27:2 NtR:27:2 NtU:27:2 NDNt+R:36:3 NDRt+R:36:3 NDUt+R:36:3 NLNt+R:36:3 NLRt+R:36:3 NLUt+R:36:3 NNNt+R:36:3 NNRt+R:36:3 NNUt+R:36:3 NNtD+R:36:3 NNtL+R:36:3 NNtN+R:36:3 NNtNt+R:36:3 NNtR+R:36:3 NNtRt+R:36:3 NNtU+R:36:3 NNtUt+R:36:3 NRNt+R:36:3 NRRt+R:36:3 NRUt+R:36:3 NRtD+R:36:3 NRtL+R:36:3 NRtN+R:36:3 NRtNt+R:36:3 NRtR+R:36:3 NRtRt+R:36:3 NRtU+R:36:3 NRtUt+R:36:3 NUNt+R:36:3 NURt+R:36:3 NUUt+R:36:3 NUtD+R:36:3 NUtL+R:36:3 NUtN+R:36:3 NUtNt+R:36:3 NUtR+R:36:3 NUtRt+R:36:3 NUtU+R:36:3 NUtUt+R:36:3 NtDD+R:36:3 NtDL+R:36:3 NtDN+R:36:3 NtDNt+R:36:3 NtDR+R:36:3 NtDRt+R:36:3 NtDU+R:36:3 NtDUt+R:36:3 NtLD+R:36:3 NtLL+R:36:3 NtLN+R:36:3 NtLNt+R:36:3 NtLR+R:36:3 NtLRt+R:36:3 NtLU+R:36:3 NtLUt+R:36:3 NtND+R:36:3 NtNL+R:36:3 NtNR+R:36:3 NtNU+R:36:3 NtNtD+R:36:3 NtNtL+R:36:3 NtNtR+R:36:3 NtNtU+R:36:3 NtRD+R:36:3 NtRL+R:36:3 NtRN+R:36:3 NtRNt+R:36:3 NtRR+R:36:3 NtRRt+R:36:3 NtRU+R:36:3 NtRUt+R:36:3 NtRtD+R:36:3 NtRtL+R:36:3 NtRtR+R:36:3 NtRtU+R:36:3 NtUD+R:36:3 NtUL+R:36:3 NtUN+R:36:3 NtUNt+R:36:3 NtUR+R:36:3 NtURt+R:36:3 NtUU+R:36:3 NtUUt+R:36:3 NtUtD+R:36:3 NtUtL+R:36:3 NtUtR+R:36:3 NtUtU+R:36:3 NNtD:37:3 NNtL:37:3 NNtN:37:3 NRtD:37:3 NRtL:37:3 NRtN:37:3 NUtD:37:3 NUtL:37:3 NUtN:37:3 NtDD:37:3 NtDL:37:3 NtDN:37:3 NtLD:37:3 NtLL:37:3 NtLN:37:3 NtND:37:3 NtNL:37:3 NtNtD:37:3 NtNtL:37:3 NtRD:37:3 NtRL:37:3 NtRN:37:3 NtRtD:37:3 NtRtL:37:3 NtUD:37:3 NtUL:37:3 NtUN:37:3 NtUtD:37:3 NtUtL:37:3 NDNt:38:3 NDRt:38:3 NDUt:38:3 NLNt:38:3 NLRt:38:3 NLUt:38:3 NNNt:38:3 NNRt:38:3 NNUt:38:3 NNtNt:38:3 NNtR:38:3 NNtRt:38:3 NNtU:38:3 NNtUt:38:3 NRNt:38:3 NRRt:38:3 NRUt:38:3 NRtNt:38:3 NRtR:38:3 NRtRt:38:3 NRtU:38:3 NRtUt:38:3 NUNt:38:3 NURt:38:3 NUUt:38:3 NUtNt:38:3 NUtR:38:3 NUtRt:38:3 NUtU:38:3 NUtUt:38:3 NtDNt:38:3 NtDR:38:3 NtDRt:38:3 NtDU:38:3 NtDUt:38:3 NtLNt:38:3 NtLR:38:3 NtLRt:38:3 NtLU:38:3 NtLUt:38:3 NtNR:38:3 NtNU:38:3 NtNtR:38:3 NtNtU:38:3 NtRNt:38:3 NtRR:38:3 NtRRt:38:3 NtRU:38:3 NtRUt:38:3 NtRtR:38:3 NtRtU:38:3 NtUNt:38:3 NtUR:38:3 NtURt:38:3 NtUU:38:3 NtUUt:38:3 NtUtR:38:3 NtUtU:38:3 NDNtU:51:4 NDRtU:51:4 NDUtU:51:4 NLNtU:51:4 NLRtU:51:4 NLUtU:51:4 NNNtU:51:4 NNRtU:51:4 NNUtU:51:4 NNtDU:51:4 NNtLU:51:4 NNtNU:51:4 NNtNtU:51:4 NNtRU:51:4 NNtRtU:51:4 NNtUU:51:4 NNtUtU:51:4 NRNtU:51:4 NRRtU:51:4 NRUtU:51:4 NRtDU:51:4 NRtLU:51:4 NRtNU:51:4 NRtNtU:51:4 NRtRU:51:4 NRtRtU:51:4 NRtUU:51:4 NRtUtU:51:4 NUNtU:51:4 NURtU:51:4 NUUtU:51:4 NUtDU:51:4 NUtLU:51:4 NUtNU:51:4 NUtNtU:51:4 NUtRU:51:4 NUtRtU:51:4 NUtUU:51:4 NUtUtU:51:4 NtDDU:51:4 NtDLU:51:4 NtDNU:51:4 NtDNtU:51:4 NtDRU:51:4 NtDRtU:51:4 NtDUU:51:4 NtDUtU:51:4 NtLDU:51:4 NtLLU:51:4 NtLNU:51:4 NtLNtU:51:4 NtLRU:51:4 NtLRtU:51:4 NtLUU:51:4 NtLUtU:51:4 NtNDU:51:4 NtNLU:51:4 NtNRU:51:4 NtNUU:51:4 NtNtDU:51:4 NtNtLU:51:4 NtNtRU:51:4 NtNtUU:51:4 NtRDU:51:4 NtRLU:51:4 NtRNU:51:4 NtRNtU:51:4 NtRRU:51:4 NtRRtU:51:4 NtRUU:51:4 NtRUtU:51:4 NtRtDU:51:4 NtRtLU:51:4 NtRtRU:51:4 NtRtUU:51:4 NtUDU:51:4 NtULU:51:4 NtUNU:51:4 NtUNtU:51:4 NtURU:51:4 NtURtU:51:4 NtUUU:51:4 NtUUtU:51:4 NtUtDU:51:4 NtUtLU:51:4 NtUtRU:51:4 NtUtUU:51:4 NDNtN:53:4 NDRtN:53:4 NDUtN:53:4 NLNtN:53:4 NLRtN:53:4 NLUtN:53:4 NNNtN:53:4 NNRtN:53:4 NNUtN:53:4 NNtDN:53:4 NNtLN:53:4 NNtNN:53:4 NNtNtN:53:4 NNtRN:53:4 NNtRtN:53:4 NNtUN:53:4 NNtUtN:53:4 NRNtN:53:4 NRRtN:53:4 NRUtN:53:4 NRtDN:53:4 NRtLN:53:4 NRtNN:53:4 NRtNtN:53:4 NRtRN:53:4 NRtRtN:53:4 NRtUN:53:4 NRtUtN:53:4 NUNtN:53:4 NURtN:53:4 NUUtN:53:4 NUtDN:53:4 NUtLN:53:4 NUtNN:53:4 NUtNtN:53:4 NUtRN:53:4 NUtRtN:53:4 NUtUN:53:4 NUtUtN:53:4 NtDDN:53:4 NtDLN:53:4 NtDNN:53:4 NtDNtN:53:4 NtDRN:53:4 NtDRtN:53:4 NtDUN:53:4 NtDUtN:53:4 NtLDN:53:4 NtLLN:53:4 NtLNN:53:4 NtLNtN:53:4 NtLRN:53:4 NtLRtN:53:4 NtLUN:53:4 NtLUtN:53:4 NtNDN:53:4 NtNLN:53:4 NtNRN:53:4 NtNUN:53:4 NtNtDN:53:4 NtNtLN:53:4 NtNtRN:53:4 NtNtUN:53:4 NtRDN:53:4 NtRLN:53:4 NtRNN:53:4 NtRNtN:53:4 NtRRN:53:4 NtRRtN:53:4 NtRUN:53:4 NtRUtN:53:4 NtRtDN:53:4 NtRtLN:53:4 NtRtRN:53:4 NtRtUN:53:4 NtUDN:53:4 NtULN:53:4 NtUNN:53:4 NtUNtN:53:4 NtURN:53:4 NtURtN:53:4 NtUUN:53:4 NtUUtN:53:4 NtUtDN:53:4 NtUtLN:53:4 NtUtRN:53:4 NtUtUN:53:4 NNt+QS:64:2 NRt+QS:64:2 NUt+QS:64:2 NtD+QS:64:2 NtL+QS:64:2 NtR+QS:64:2 NtU+QS:64:2 NDNt+QS:73:3 NDRt+QS:73:3 NDUt+QS:73:3 NLNt+QS:73:3 NLRt+QS:73:3 NLUt+QS:73:3 NNNt+QS:73:3 NNRt+QS:73:3 NNUt+QS:73:3 NNtD+QS:73:3 NNtL+QS:73:3 NNtN+QS:73:3 NNtNt+QS:73:3 NNtR+QS:73:3 NNtRt+QS:73:3 NNtU+QS:73:3 NNtUt+QS:73:3 NRNt+QS:73:3 NRRt+QS:73:3 NRUt+QS:73:3 NRtD+QS:73:3 NRtL+QS:73:3 NRtN+QS:73:3 NRtNt+QS:73:3 NRtR+QS:73:3 NRtRt+QS:73:3 NRtU+QS:73:3 NRtUt+QS:73:3 NUNt+QS:73:3 NURt+QS:73:3 NUUt+QS:73:3 NUtD+QS:73:3 NUtL+QS:73:3 NUtN+QS:73:3 NUtNt+QS:73:3 NUtR+QS:73:3 NUtRt+QS:73:3 NUtU+QS:73:3 NUtUt+QS:73:3 NtDD+QS:73:3 NtDL+QS:73:3 NtDN+QS:73:3 NtDNt+QS:73:3 NtDR+QS:73:3 NtDRt+QS:73:3 NtDU+QS:73:3 NtDUt+QS:73:3 NtLD+QS:73:3 NtLL+QS:73:3 NtLN+QS:73:3 NtLNt+QS:73:3 NtLR+QS:73:3 NtLRt+QS:73:3 NtLU+QS:73:3 NtLUt+QS:73:3 NtND+QS:73:3 NtNL+QS:73:3 NtNR+QS:73:3 NtNU+QS:73:3 NtNtD+QS:73:3 NtNtL+QS:73:3 NtNtR+QS:73:3 NtNtU+QS:73:3 NtRD+QS:73:3 NtRL+QS:73:3 NtRN+QS:73:3 NtRNt+QS:73:3 NtRR+QS:73:3 NtRRt+QS:73:3 NtRU+QS:73:3 NtRUt+QS:73:3 NtRtD+QS:73:3 NtRtL+QS:73:3 NtRtR+QS:73:3 NtRtU+QS:73:3 NtUD+QS:73:3 NtUL+QS:73:3 NtUN+QS:73:3 NtUNt+QS:73:3 NtUR+QS:73:3 NtURt+QS:73:3 NtUU+QS:73:3 NtUUt+QS:73:3 NtUtD+QS:73:3 NtUtL+QS:73:3 NtUtR+QS:73:3 NtUtU+QS:73:3 NDNtN+QS:82:4 NDNtU+QS:82:4 NDRtN+QS:82:4 NDRtU+QS:82:4 NDUtN+QS:82:4 NDUtU+QS:82:4 NLNtN+QS:82:4 NLNtU+QS:82:4 NLRtN+QS:82:4 NLRtU+QS:82:4 NLUtN+QS:82:4 NLUtU+QS:82:4 NNNtN+QS:82:4 NNNtU+QS:82:4 NNRtN+QS:82:4 NNRtU+QS:82:4 NNUtN+QS:82:4 NNUtU+QS:82:4 NNtDN+QS:82:4 NNtDU+QS:82:4 NNtLN+QS:82:4 NNtLU+QS:82:4 NNtNN+QS:82:4 NNtNU+QS:82:4 NNtNtN+QS:82:4 NNtNtU+QS:82:4 NNtRN+QS:82:4 NNtRU+QS:82:4 NNtRtN+QS:82:4 NNtRtU+QS:82:4 NNtUN+QS:82:4 NNtUU+QS:82:4 NNtUtN+QS:82:4 NNtUtU+QS:82:4 NRNtN+QS:82:4 NRNtU+QS:82:4 NRRtN+QS:82:4 NRRtU+QS:82:4 NRUtN+QS:82:4 NRUtU+QS:82:4 NRtDN+QS:82:4 NRtDU+QS:82:4 NRtLN+QS:82:4 NRtLU+QS:82:4 NRtNN+QS:82:4 NRtNU+QS:82:4 NRtNtN+QS:82:4 NRtNtU+QS:82:4 NRtRN+QS:82:4 NRtRU+QS:82:4 NRtRtN+QS:82:4 NRtRtU+QS:82:4 NRtUN+QS:82:4 NRtUU+QS:82:4 NRtUtN+QS:82:4 NRtUtU+QS:82:4 NUNtN+QS:82:4 NUNtU+QS:82:4 NURtN+QS:82:4 NURtU+QS:82:4 NUUtN+QS:82:4 NUUtU+QS:82:4 NUtDN+QS:82:4 NUtDU+QS:82:4 NUtLN+QS:82:4 NUtLU+QS:82:4 NUtNN+QS:82:4 NUtNU+QS:82:4 NUtNtN+QS:82:4 NUtNtU+QS:82:4 NUtRN+QS:82:4 NUtRU+QS:82:4 NUtRtN+QS:82:4 NUtRtU+QS:82:4 NUtUN+QS:82:4 NUtUU+QS:82:4 NUtUtN+QS:82:4 NUtUtU+QS:82:4 NtDDN+QS:82:4 NtDDU+QS:82:4 NtDLN+QS:82:4 NtDLU+QS:82:4 NtDNN+QS:82:4 NtDNU+QS:82:4 NtDNtN+QS:82:4 NtDNtU+QS:82:4 NtDRN+QS:82:4 NtDRU+QS:82:4 NtDRtN+QS:82:4 NtDRtU+QS:82:4 NtDUN+QS:82:4 NtDUU+QS:82:4 NtDUtN+QS:82:4 NtDUtU+QS:82:4 NtLDN+QS:82:4 NtLDU+QS:82:4 NtLLN+QS:82:4 NtLLU+QS:82:4 NtLNN+QS:82:4 NtLNU+QS:82:4 NtLNtN+QS:82:4 NtLNtU+QS:82:4 NtLRN+QS:82:4 NtLRU+QS:82:4 NtLRtN+QS:82:4 NtLRtU+QS:82:4 NtLUN+QS:82:4 NtLUU+QS:82:4 NtLUtN+QS:82:4 NtLUtU+QS:82:4 NtNDN+QS:82:4 NtNDU+QS:82:4 NtNLN+QS:82:4 NtNLU+QS:82:4 NtNRN+QS:82:4 NtNRU+QS:82:4 NtNUN+QS:82:4 NtNUU+QS:82:4 NtNtDN+QS:82:4 NtNtDU+QS:82:4 NtNtLN+QS:82:4 NtNtLU+QS:82:4 NtNtRN+QS:82:4 NtNtRU+QS:82:4 NtNtUN+QS:82:4 NtNtUU+QS:82:4 NtRDN+QS:82:4 NtRDU+QS:82:4 NtRLN+QS:82:4 NtRLU+QS:82:4 NtRNN+QS:82:4 NtRNU+QS:82:4 NtRNtN+QS:82:4 NtRNtU+QS:82:4 NtRRN+QS:82:4 NtRRU+QS:82:4 NtRRtN+QS:82:4 NtRRtU+QS:82:4 NtRUN+QS:82:4 NtRUU+QS:82:4 NtRUtN+QS:82:4 NtRUtU+QS:82:4 NtRtDN+QS:82:4 NtRtDU+QS:82:4 NtRtLN+QS:82:4 NtRtLU+QS:82:4 NtRtRN+QS:82:4 NtRtRU+QS:82:4 NtRtUN+QS:82:4 NtRtUU+QS:82:4 NtUDN+QS:82:4 NtUDU+QS:82:4 NtULN+QS:82:4 NtULU+QS:82:4 NtUNN+QS:82:4 NtUNU+QS:82:4 NtUNtN+QS:82:4 NtUNtU+QS:82:4 NtURN+QS:82:4 NtURU+QS:82:4 NtURtN+QS:82:4 NtURtU+QS:82:4 NtUUN+QS:82:4 NtUUU+QS:82:4 NtUUtN+QS:82:4 NtUUtU+QS:82:4 NtUtDN+QS:82:4 NtUtDU+QS:82:4 NtUtLN+QS:82:4 NtUtLU+QS:82:4 NtUtRN+QS:82:4 NtUtRU+QS:82:4 NtUtUN+QS:82:4 NtUtUU+QS:82:4'
};

export const MOVES: Record<string, Move[]> = (() => {
  const out: Record<string, Move[]> = {};
  out['jumpslash'] = [
    {id:'jumpslash', name:'Jump slash', frames:35, sword:'out' as const, stands:1, on:true},
    {id:'jumpslash_qs', name:'Jump slash QS', frames:37, sword:'out' as const, stands:1, on:true}
  ];
  out['backflip'] = [
    {id:'backflip', name:'Backflip', frames:22, sword:'any' as const, stands:1, on:true},
    {id:'backflip_qs', name:'Backflip QS', frames:31, sword:'out' as const, stands:1, on:true}
  ];
  out['sidehop'] = [
    {id:'sidehop_l', name:'Sidehop L', frames:21, sword:'any' as const, stands:1, on:true},
    {id:'sidehop_r', name:'Sidehop R', frames:21, sword:'any' as const, stands:1, on:true}
  ];
  out['roll'] = [
    {id:'dry_roll', name:'Roll', frames:21, sword:'away' as const, stands:1, on:true},
    {id:'dry_roll_r', name:'Roll R', frames:21, sword:'away' as const, stands:1, on:true},
    {id:'dry_roll_r_free', name:'Roll R no L', frames:22, sword:'away' as const, stands:1, on:true}
  ];
  out['slash'] = [
    {id:'target_slash_r', name:'Slash R', frames:14, sword:'out' as const, stands:1, on:true},
    {id:'target_slash', name:'Slash', frames:16, sword:'out' as const, stands:1, on:true},
    {id:'target_slash_qs', name:'Slash QS', frames:48, sword:'out' as const, stands:1, on:true}
  ];
  out['uslash'] = [
    {id:'neutral_slash_r', name:'Untargeted slash R', frames:14, sword:'out' as const, stands:1, on:true},
    {id:'neutral_slash', name:'Untargeted slash', frames:15, sword:'out' as const, stands:1, on:true}
  ];
  out['crawl'] = [
    {id:'crawl', name:'Crawl', frames:39, sword:'away' as const, stands:1, on:true},
    {id:'crawl_r', name:'Crawl R', frames:40, sword:'away' as const, stands:1, on:true}
  ];
  out['turns'] = [
    {id:'fine_turn', name:'C Up Turn', frames:46, sword:'any' as const, stands:200, on:true},
    {id:'cup_cdown_turnaround', name:'C Up C Down Turnaround', frames:19, sword:'any' as const, stands:1, on:true, fixed:true},
    {id:'cup_b_settle_turnaround', name:'C Up B Settle Turnaround', frames:21, sword:'any' as const, stands:1, on:true, fixed:true},
    {id:'cup_b_early_turnaround', name:'C Up B Early Turnaround', frames:19, sword:'any' as const, stands:1, on:true, fixed:true},
    {id:'l_cdown_turnaround', name:'L + C Down Turnaround', frames:24, sword:'any' as const, stands:1, on:true, fixed:true}
  ];
  Object.keys(COMBOS).forEach(k => {
    out[k] = COMBOS[k].split(' ').map(s => {
      const p = s.split(':'), n = Number(p[2]);
      return {id:p[0], name:'Combo ' + p[0], frames:Number(p[1]), sword:'out' as const,
              stands:1, presses:n, on:false};
    });
  });
  return out;
})();
