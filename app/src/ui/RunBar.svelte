<!-- Run bar. Collision is one three-way choice, not two switches, so walls without floors cannot
     be chosen. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {settings as store, keep} from '../core/settings.svelte';
  let {live = false, swings = true, settings = store,
       blocked = null, noDisc = false, reach = undefined, logs, logCount = 0,
       go = () => {}, running = false, pct = 0, figures = [], ended = null}:
       {live?: boolean; swings?: boolean; settings?: typeof store;
        blocked?: string | null; noDisc?: boolean; reach?: (() => void) | undefined;
        logs: () => void; logCount: number;
        go?: () => void; running?: boolean; pct?: number;
        figures?: Array<[string, string, boolean?]>; ended?: 'stopped' | 'empty' | 'done' | 'broke' | null} = $props();
</script>

<div class="runbar">
  <div class="rbrow">
    <!-- Search/Stop; a blocker never disables Stop. -->
    <button type="button" class="go" disabled={!running && (blocked !== null || noDisc)}
            title={!running && blocked ? blocked : undefined}
            onclick={go}>{running ? t.stop : t.search}</button>
    <div class="cfg">
      <div class="grp" role="group" aria-label={t.collision}>
        <span class="seg">
          <button type="button" aria-pressed={settings.ground === 'none'} disabled={running}
                  title={running ? t.stopToChange : undefined}
                  onclick={() => { settings.ground = 'none'; keep(); }}>{t.none}</button>
          <button type="button" aria-pressed={settings.ground === 'floors'} disabled={running}
                  title={running ? t.stopToChange : undefined}
                  onclick={() => { settings.ground = 'floors'; keep(); }}>{t.groundFloors}</button>
          <button type="button" aria-pressed={settings.ground === 'solid'} disabled={running}
                  title={running ? t.stopToChange : undefined}
                  onclick={() => { settings.ground = 'solid'; keep(); }}>{t.solid}</button>
        </span>
        <!-- Actors need floors and sword needs walls (disabled otherwise); each is hidden when it
             cannot exist: actors come from a live game, sword only if a chosen move swings. -->
        <span class="switches">
          <label class="sw" data-role="actors" hidden={!live}
                 title={settings.ground === 'none' ? t.needsFloors
                   : running ? t.stopToChange : undefined}>
            <input type="checkbox" bind:checked={settings.actors} onchange={keep}
                   disabled={running || settings.ground === 'none'}><span>{t.actors}</span></label>
          <label class="sw" data-role="sword" hidden={!swings}
                 title={settings.ground !== 'solid' ? t.needsWalls
                   : running ? t.stopToChange : undefined}>
            <input type="checkbox" bind:checked={settings.sword} onchange={keep}
                   disabled={running || settings.ground !== 'solid'}>
            <span>{t.sword}</span></label>
        </span>
      </div>
      <div class="grp">
        <span class="num"><label for="lLen">{t.steps}</label>
          <input type="number" id="lLen" bind:value={settings.steps} disabled={running}
                 onchange={keep} title={running ? t.stopToChange : undefined}></span>
        <span class="num"><label for="lFr">{t.frames}</label>
          <input type="number" id="lFr" bind:value={settings.frames} disabled={running}
                 onchange={keep} title={running ? t.stopToChange : undefined}></span>
      </div>
    </div>
    <span class="sp"></span>
  </div>

  <div class="rbrow two report" aria-live="polite">
    <div class="prog"><i style:width="{pct}%"></i></div>
    <span class="pct">{ended === 'stopped' ? t.stopped
      : ended && pct === 100 ? t.doneWord : t.percent(pct)}</span>
    {#if blocked}
      <!-- A button: it focuses the control that blocks the search. -->
      <button type="button" class="blk" title={t.goToIt} onclick={() => reach?.()}
        >{blocked}</button>
    {:else if figures.length}
      <!-- Inline colour: the frozen stylesheet has no rule for a flagged figure. -->
      <p class="live">{#each figures as [what, value, warn] (what)}<span
        ><u style:color={warn ? 'var(--miss)' : undefined}>{what}</u
        ><b style:color={warn ? 'var(--miss)' : undefined}>{value}</b></span>{/each}</p>
    {:else}
      <p class="live"><span>{t.notStarted}</span></p>
    {/if}
    <span class="sp"></span>
    <button type="button" class="logsbtn" onclick={logs}>{t.logs} <b>{logCount}</b></button>
  </div>
</div>
