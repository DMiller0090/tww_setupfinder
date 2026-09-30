<!-- Bounds. Sides start open; closing one never fills in a default value. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {snapF32} from '../lib/f32';
  import {settings as store, keep} from '../core/settings.svelte';
  import type {Side} from '../room/marks';

  let {settings = store}: {settings?: typeof store} = $props();

  const label: Record<Side, string> = {xmin: t.xMin, xmax: t.xMax, zmin: t.zMin, zmax: t.zMax};
  const edgeLabel: Record<Side, string> = {
    xmin: t.edgeXMin, xmax: t.edgeXMax, zmin: t.edgeZMin, zmax: t.edgeZMax,
  };
  const shut = $derived((Object.keys(settings.bOpen) as Side[]).filter(s => !settings.bOpen[s]));
</script>

<section class="panel" id="pBnd">
  <div class="phead"><h2>{t.bounds}</h2><span class="sp"></span>
    <span class="r">{shut.length === 0 ? t.anywhere : shut.map(s => label[s]).join(' · ')}</span></div>
  <div class="body">
    <!-- Each label sits above its field, so a full float fits within the rail. -->
    <div class="bounds">
      {#each ['zmax', 'xmin'] as const as side}
        <div class="bside {side}" style:flex-direction="column" style:align-items="center"
             style:gap=".125rem"><label for="b{side}">{label[side]}</label>
          <input type="text" id="b{side}" bind:value={settings.bValue[side]}
                 style:width="calc(12ch + .75rem)" style:flex="none"
                 disabled={settings.bOpen[side]} oninput={keep}
                 onchange={() => { settings.bValue[side] = snapF32(settings.bValue[side]); keep(); }}
                 title={settings.bOpen[side] ? t.sideOpen : label[side]} autocomplete="off"
                 spellcheck="false"
                 aria-invalid={!settings.bOpen[side] && settings.bValue[side].trim() === ''}></div>
      {/each}
      <div class="box">
        <div class="fill"></div>
        <span class="who">{t.start}</span>
        {#each ['zmax', 'zmin'] as const as side}
          <button type="button" class="edge h {side}" data-open={settings.bOpen[side]}
                  aria-label={edgeLabel[side]} title={t.closeSide}
                  onclick={() => { settings.bOpen[side] = !settings.bOpen[side]; keep(); }}
          ></button>
        {/each}
        {#each ['xmin', 'xmax'] as const as side}
          <button type="button" class="edge v {side}" data-open={settings.bOpen[side]}
                  aria-label={edgeLabel[side]} title={t.closeSide}
                  onclick={() => { settings.bOpen[side] = !settings.bOpen[side]; keep(); }}
          ></button>
        {/each}
      </div>
      {#each ['xmax', 'zmin'] as const as side}
        <div class="bside {side}" style:flex-direction="column" style:align-items="center"
             style:gap=".125rem"><label for="b{side}">{label[side]}</label>
          <input type="text" id="b{side}" bind:value={settings.bValue[side]}
                 style:width="calc(12ch + .75rem)" style:flex="none"
                 disabled={settings.bOpen[side]} oninput={keep}
                 onchange={() => { settings.bValue[side] = snapF32(settings.bValue[side]); keep(); }}
                 title={settings.bOpen[side] ? t.sideOpen : label[side]} autocomplete="off"
                 spellcheck="false"
                 aria-invalid={!settings.bOpen[side] && settings.bValue[side].trim() === ''}></div>
      {/each}
    </div>
    <p class="hint">{t.clickAnEdge}</p>
  </div>
</section>
