<!-- Plans found and the chosen plan's sequence. Plan columns are percentages so a narrow panel
     does not clip the last ones. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {run, shown, reaches, aimsAtItem, planId, type Plan}
    from '../core/run.svelte';
  import {showFacing} from '../lib/facing.svelte';
  import {gutter} from './rail';

  let {copy}: {copy: (text: string) => Promise<boolean>} = $props();

  let copied = $state(false);

  const rows = $derived(shown());

  /* Off is in float steps to the nearest allowed place, not units, so near misses are not 0. */
  const showOff = (p: Plan): string =>
    reaches(p) === 'miss' && p.ox !== undefined && p.oz !== undefined && (p.ox || p.oz)
      ? t.offAxes([p.ox, p.oz]) : p.off.toLocaleString();

  const num = (v: number): string => String(v);

  const empty = $derived.by(() => {
    if (rows.length) return null;
    if (run.ended === 'empty') return {what: t.nothingFound, why: null};
    if (run.ended === 'stopped') return {what: t.nothingYet, why: t.stoppedBeforeAny};
    return {what: t.nothingYet, why: t.plansArrive};
  });

  const chosenStops = $derived(run.chosen?.stops ?? []);

  /* Space-padded, not tabbed, so columns line up whatever the reader's tab width. */
  function aligned(cells: string[][]): string {
    if (!cells.length) return '';
    const widths = cells[0].map((_, c) => Math.max(...cells.map(r => r[c].length)));
    return cells.map(r => r.map((v, c) => c === 0 ? v.padEnd(widths[c]) : v.padStart(widths[c]))
      .join('  ').trimEnd()).join('\n');
  }

  async function copySeq(): Promise<void> {
    const p = run.chosen;
    if (!p) return;
    const cells: string[][] = p.stops.map(s =>
      [s.move.n, s.frames, s.total, s.x, s.z, showFacing(s.f)].map(String));
    if (aimsAtItem()) cells.push([t.item, '', '', String(p.x), String(p.z), '']);
    const text = aligned(cells);
    copied = await copy(text);
    if (copied) setTimeout(() => { copied = false; }, 1200);
  }
</script>

<div class="answers">
  <section class="panel plans">
    <div class="phead"><h2>{t.plans}</h2>
      <span class="key">
        <span><i style="background:var(--hit)"></i>{t.reachesWord}</span>
        <span><i style="background:var(--miss)"></i>{t.missesWord}</span>
      </span>
    </div>
    <div class="body" style="padding:0">
      <table>
        <colgroup><col style="width:9%"><col style="width:11%"><col style="width:30%"
          ><col style="width:20%"><col style="width:20%"><col style="width:10%"></colgroup>
        <thead><tr>
          <th scope="col" class="n">{t.frames}</th>
          <th scope="col" class="n">{t.off}</th>
          <th scope="col">{t.moves}</th>
          <th scope="col" class="n">{t.x}</th>
          <th scope="col" class="n">{t.z}</th>
          <th scope="col" class="n">{t.facing}</th>
        </tr></thead>
        <tbody>
          <!-- Keyed on planId: each report sends fresh objects, and object keys would rebuild every row. -->
          {#each rows as p (planId(p))}
            {@const names = p.stops.map(s => s.move.n).join(', ')}
            {@const end = p.stops[p.stops.length - 1]}
            <!-- Link's end is the last stop; with the item aimed the plan's x/z is the item's. -->
            {@const he = end ? {x: end.x, z: end.z} : (aimsAtItem() ? null : {x: p.x, z: p.z})}
            <!-- svelte-ignore a11y_click_events_have_key_events
                 It has them: the row takes focus, and Enter or Space chooses it. -->
            <tr data-hit={reaches(p)} aria-selected={run.chosen != null && planId(run.chosen) === planId(p)}
                tabindex="0" title={names}
                onclick={() => run.chosen = p}
                onkeydown={(e) => {
                  if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); run.chosen = p; }
                }}>
              <td class="n">{p.frames}</td>
              <td class="n off">{showOff(p)}</td>
              <td class="mv">{names || t.dash}</td>
              <td class="n">{he ? num(he.x) : t.dash}</td>
              <td class="n">{he ? num(he.z) : t.dash}</td>
              <td class="n">{end ? showFacing(end.f) : t.dash}</td>
            </tr>
          {/each}
        </tbody>
      </table>
      {#if empty}
        <div class="empty" data-run={run.ended === 'empty' ? 'empty' : undefined}
             title={empty.why ?? undefined}><b>{empty.what}</b></div>
      {/if}
    </div>
  </section>

  <!-- svelte-ignore a11y_no_noninteractive_tabindex -->
  <div class="gutter h" id="gutterH" role="separator" aria-orientation="horizontal" tabindex="0"
       aria-label={t.sequenceHeight}
       use:gutter={{name: '--seq', min: 20, max: 75, from: 'bottom'}}></div>

  <section class="panel seq">
    <div class="phead"><h2>{t.sequence}</h2>
      <span class="r">{run.chosen ? t.framesCount(run.chosen.frames) : ''}</span>
      <span class="sp"></span>
      <span class="copied" style:opacity={copied ? 1 : 0}>{t.copied}</span>
      <button type="button" class="ghost icon" aria-label={t.copy} disabled={!chosenStops.length}
              onclick={copySeq}><svg viewBox="0 0 16 16" width="13" height="13"
        aria-hidden="true" fill="none" stroke="currentColor" stroke-width="1.4"
        stroke-linecap="round" stroke-linejoin="round"><rect x="5.75" y="5.75" width="7.5"
        height="8.5" rx="1.5"></rect><path d="M10.25 3.25H4.5a1.25 1.25 0 0 0-1.25 1.25v6.25"
        ></path></svg></button>
    </div>
    <div class="body">
      {#if !run.chosen}
        <div class="empty"><b>{t.nothingChosen}</b></div>
      {:else if !chosenStops.length}
        <div class="empty"><b>{run.running ? t.waiting : t.noMoves}</b></div>
      {:else}
        <table class="seqtab">
          <colgroup><col><col style="width:3.25rem"><col style="width:3.25rem">
            <col style="width:7.75rem"><col style="width:7.75rem"><col style="width:4rem"></colgroup>
          <thead><tr><th scope="col">{t.move}</th><th scope="col" class="n">{t.frames}</th>
            <th scope="col" class="n">{t.totalCol}</th><th scope="col" class="n">{t.x}</th>
            <th scope="col" class="n">{t.z}</th><th scope="col" class="n">{t.facing}</th></tr></thead>
          <tbody>
            {#each chosenStops as s (s)}
              <tr><td class="mv" title={s.move.id}>{s.move.n}</td>
                <td class="n">{s.frames}</td><td class="n">{s.total}</td>
                <td class="n">{num(s.x)}</td><td class="n">{num(s.z)}</td>
                <td class="n">{showFacing(s.f)}</td></tr>
            {/each}
          </tbody>
          {#if aimsAtItem() && run.chosen}
            <tfoot><tr><td class="mv" colspan="3">{t.item}</td>
              <td class="n">{num(run.chosen.x)}</td><td class="n">{num(run.chosen.z)}</td>
              <td></td></tr></tfoot>
          {/if}
        </table>
      {/if}
    </div>
  </section>
</div>
