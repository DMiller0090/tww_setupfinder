<!-- Location. Exact parsing, no defaults: an emptied box is refused, never refilled. -->
<script lang="ts">
  import {t} from '../lib/strings';
  import {snapF32} from '../lib/f32';
  import {settings as store, keep, fromGame} from '../core/settings.svelte';
  let {live = false, update = () => {}, settings = store}:
    {live?: boolean; update?: () => void; settings?: typeof store} = $props();

  /* A typed X or Z drops the height read from the game, and the start stops being the game's. */
  function moved(): void {
    settings.sy = '';
    fromGame.start = false;
    keep();
  }
</script>

<section class="panel" id="pLoc">
  <div class="phead"><h2>{t.location}</h2><span class="chip ok">{t.standing}</span>
    <span class="sp"></span>
    <!-- Reads position and both facings from the game; hidden, not disabled, with no game. -->
    <button type="button" class="mini" hidden={!live} onclick={update}>{t.update}</button></div>
  <div class="body">
    <div class="f"><label for="sx">{t.x}</label>
      <input type="text" id="sx" bind:value={settings.sx} autocomplete="off" spellcheck="false"
             oninput={moved} onchange={() => { settings.sx = snapF32(settings.sx); keep(); }}
             aria-invalid={settings.sx.trim() === ''}></div>
    <div class="f"><label for="sz">{t.z}</label>
      <input type="text" id="sz" bind:value={settings.sz} autocomplete="off" spellcheck="false"
             oninput={moved} onchange={() => { settings.sz = snapF32(settings.sz); keep(); }}
             aria-invalid={settings.sz.trim() === ''}></div>
    <fieldset style="margin-top:.4375rem">
      <legend>{t.facing}</legend>
      <div class="f"><label for="sf">{t.link}</label>
        <input type="text" id="sf" bind:value={settings.sf} autocomplete="off" spellcheck="false"
               oninput={keep} aria-invalid={settings.sf.trim() === ''}></div>
      <div class="f"><label for="scam">{t.camera}</label>
        <input type="text" id="scam" bind:value={settings.scam} autocomplete="off"
               spellcheck="false" oninput={keep}
               aria-invalid={settings.scam.trim() === ''}></div>
    </fieldset>
  </div>
</section>
