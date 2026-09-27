<!-- Target. `Bomb/Small Pot` is disabled: the sim has one overhead offset for every held item. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {snapF32} from '../lib/f32';
  import Address from './Address.svelte';
  import {settings as store, keep} from '../core/settings.svelte';
  let {settings = store}: {settings?: typeof store} = $props();

  /* The address range switch needs all four Range edges as numbers. */
  const rangeTyped = $derived([settings.rx1, settings.rx2, settings.rz1, settings.rz2]
    .every(v => v.trim() !== '' && Number.isFinite(Number(v.trim()))));

  /* X and Z cannot both be free (no distance to order by); the last one checked wins. */
  function freesX(): void {
    if (settings.anyx) settings.anyz = false;
    keep();
  }
  function freesZ(): void {
    if (settings.anyz) settings.anyx = false;
    keep();
  }
</script>

<section class="panel" id="pTgt">
  <div class="phead"><h2>{t.target}</h2></div>
  <div class="body">
    <div class="f" style="margin-bottom:.4375rem">
      <span class="seg" role="group" aria-label={t.shape}>
        <button type="button" aria-pressed={settings.shape === 'point'}
                onclick={() => { settings.shape = 'point'; keep(); }}>{t.point}</button>
        <button type="button" aria-pressed={settings.shape === 'range'}
                onclick={() => { settings.shape = 'range'; keep(); }}>{t.range}</button>
        <button type="button" aria-pressed={settings.shape === 'addr'}
                onclick={() => { settings.shape = 'addr'; keep(); }}>{t.address}</button>
      </span>
      <select aria-label={t.aim} bind:value={settings.aim} onchange={keep}>
        <option value="player">{t.aimPlayer}</option>
        <option value="overhead">{t.aimOverhead}</option>
        <option value="bomb" disabled>{t.aimBomb}</option>
      </select>
    </div>

    {#if settings.shape === 'range'}
      <div>
        <div class="f"><label for="rx1">{t.x}</label>
          <input type="text" id="rx1" bind:value={settings.rx1} autocomplete="off"
                 spellcheck="false" oninput={keep}
                 onchange={() => { settings.rx1 = snapF32(settings.rx1); keep(); }} aria-invalid={settings.rx1.trim() === ''}>
          <input type="text" id="rx2" bind:value={settings.rx2} aria-label={t.xMax}
                 autocomplete="off" spellcheck="false" oninput={keep}
                 onchange={() => { settings.rx2 = snapF32(settings.rx2); keep(); }}
                 aria-invalid={settings.rx2.trim() === ''}></div>
        <div class="f"><label for="rz1">{t.z}</label>
          <input type="text" id="rz1" bind:value={settings.rz1} autocomplete="off"
                 spellcheck="false" oninput={keep}
                 onchange={() => { settings.rz1 = snapF32(settings.rz1); keep(); }} aria-invalid={settings.rz1.trim() === ''}>
          <input type="text" id="rz2" bind:value={settings.rz2} aria-label={t.zMaxBox}
                 autocomplete="off" spellcheck="false" oninput={keep}
                 onchange={() => { settings.rz2 = snapF32(settings.rz2); keep(); }}
                 aria-invalid={settings.rz2.trim() === ''}></div>
      </div>
    {/if}

    {#if settings.shape === 'addr'}
      <Address {settings} />
    {/if}

    {#if settings.shape === 'point'}
      <div>
        <div class="f"><label for="tx">{t.x}</label>
          <input type="text" id="tx" bind:value={settings.tx} disabled={settings.anyx}
                 autocomplete="off" spellcheck="false" oninput={keep}
                 onchange={() => { settings.tx = snapF32(settings.tx); keep(); }}
                 aria-invalid={!settings.anyx && settings.tx.trim() === ''}>
          <span class="check"><input type="checkbox" id="anyx" bind:checked={settings.anyx}
              onchange={freesX}>
            <label for="anyx">{t.all}</label></span></div>
        <div class="f"><label for="tz">{t.z}</label>
          <input type="text" id="tz" bind:value={settings.tz} disabled={settings.anyz}
                 autocomplete="off" spellcheck="false" oninput={keep}
                 onchange={() => { settings.tz = snapF32(settings.tz); keep(); }}
                 aria-invalid={!settings.anyz && settings.tz.trim() === ''}>
          <span class="check"><input type="checkbox" id="anyz" bind:checked={settings.anyz}
              onchange={freesZ}>
            <label for="anyz">{t.all}</label></span></div>
        <div class="f"><label for="tol">{t.tolerance}</label>
          <input type="text" id="tol" bind:value={settings.tol} autocomplete="off"
                 spellcheck="false" oninput={keep}></div>
      </div>
    {/if}

    <fieldset style="margin-top:.5rem">
      <legend>{t.facing}</legend>
      <!-- A row wrapper, or the block `.check` drops below the buttons. -->
      <div class="f" style="margin-bottom:0">
        <span class="seg" role="group" aria-label={t.facing}>
          <button type="button" aria-pressed={settings.fmode === 'any'}
                  onclick={() => { settings.fmode = 'any'; keep(); }}>{t.all}</button>
          <button type="button" aria-pressed={settings.fmode === 'single'}
                  onclick={() => { settings.fmode = 'single'; keep(); }}>{t.single}</button>
          <button type="button" aria-pressed={settings.fmode === 'range'}
                  onclick={() => { settings.fmode = 'range'; keep(); }}>{t.range}</button>
        </span>
        <!-- Whether the Range box also limits the address. -->
        {#if settings.shape === 'addr'}
          <span class="check" style="margin-left:.75rem"><input type="checkbox" id="addrRange"
              bind:checked={settings.addrRange} onchange={keep} disabled={!rangeTyped}>
            <label for="addrRange">{t.range}</label></span>
        {/if}
      </div>
      <!-- Space reserved so the panel height does not change with the mode. -->
      <div class="reserve" style="margin-top:.375rem">
        {#if settings.fmode !== 'any'}
          <div class="f">
            <input type="text" id="fa" bind:value={settings.fA} aria-label={t.facing}
                   autocomplete="off" spellcheck="false" oninput={keep}
                   aria-invalid={settings.fA.trim() === ''}>
            {#if settings.fmode === 'range'}
              <input type="text" id="fb" bind:value={settings.fB} aria-label={t.facingMax}
                     autocomplete="off" spellcheck="false" oninput={keep}
                     aria-invalid={settings.fB.trim() === ''}>
            {/if}
          </div>
        {/if}
      </div>
    </fieldset>
  </div>
</section>
