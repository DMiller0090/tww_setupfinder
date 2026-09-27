<script lang="ts">
  import {untrack} from 'svelte';
  import {t} from './lib/strings';
  import Bar from './ui/Bar.svelte';
  import Location from './ui/Location.svelte';
  import Target from './ui/Target.svelte';
  import Bounds from './ui/Bounds.svelte';
  import RunBar from './ui/RunBar.svelte';
  import Room from './room/Room.svelte';
  import Msgs from './ui/Msgs.svelte';
  import Logs from './ui/win/Logs.svelte';
  import Stage from './ui/win/Stage.svelte';
  import Options from './ui/win/Options.svelte';
  import Moves from './ui/win/Moves.svelte';
  import Ask from './ui/win/Ask.svelte';
  import Emulators from './ui/win/Emulators.svelte';
  import {askAll} from './core/ask';
  import type {RoomRead, StartRead, AskedTarget} from './core/ask';
  import {log, catchThrows, say, logLine} from './lib/log.svelte';
  import {chosen, costs, total} from './core/catalog.svelte';
  import {TYPES} from './core/moves';
  import {live, toggle, connect, askMachine, canConnect, watch} from './core/live.svelte';
  import {canChoose, chooseDisc} from './core/disc';
  import type {Actor, Emulator} from './core/fixtures';
  import {run, start as startRun, stop as stopRun, shown, aimsAtItem}
    from './core/run.svelte';
  import {readFacing, wrap} from './lib/facing.svelte';
  import Answers from './ui/Answers.svelte';
  import {gutter} from './ui/rail';
  import type {Marks, Side} from './room/marks';
  import type {Parts} from './room/renderer';
  import {addrBox, addrMask, addrY, isRun, withinRange} from './core/address';
  import {ground, follow as followGround} from './core/ground.svelte';
  import {settings, keep, atRoom, roomKey, fromGame} from './core/settings.svelte';
  import {rail, vgutter, hgutter} from './ui/rail';
  import type {RoomIndex} from './core/fixtures';
  import {target} from './fixtures/start';

  /* The disc's rooms when there is a disc, else the game's one room. */
  let discRooms = $state<RoomIndex[]>([]);
  let gameRoom = $state<RoomIndex | null>(null);
  const rooms = $derived<RoomIndex[]>(
    discRooms.length ? discRooms : (gameRoom ? [gameRoom] : []));
  let picked = $state<RoomIndex | null>(null);
  /* The disc path came from the settings file; if it reads no rooms it is cleared. A typed path is
     never cleared. */
  let fromFile = $state(true);

  /** Every way of choosing, typing or clearing a disc ends here. */
  function tookDisc(): void {
    fromFile = false;
    say('info', settings.iso ? t.gameAttached : t.gameRemoved);
  }

  /* The OS file dialog, since a WebView file input gives no path. Cancel (`null`) changes nothing. */
  async function browseDisc(): Promise<void> {
    try {
      const got = await chooseDisc();
      if (got !== null) {
        settings.iso = got;
        keep();
        tookDisc();
      }
    } catch (e) {
      logLine('error', e instanceof Error ? e.message : String(e));
    }
  }
  let parts = $state<Parts | null>(null);
  let logsOpen = $state(false), stageOpen = $state(false), optionsOpen = $state(false);
  let movesOpen = $state(false);
  /* One confirm dialog, opened with its question by `askTo`. */
  let askOpen = $state(false);
  let instOpen = $state(false);
  let roomActors = $state<Actor[]>([]);
  let asked = $state({title: '', line: '', yes: '', run: (): void => {}});
  function askTo(title: string, line: string, yes: string, run: () => void): void {
    asked = {title, line, yes, run};
    askOpen = true;
  }

  /* Every box and switch lives in `settings`. `settings.sy` is Link's height: read from the game,
     emptied when X or Z is typed. */

  catchThrows(t.somethingBroke);

  const words = {
    noEmulator: t.noEmulator, noGame: t.noGameLoaded, pauseFirst: t.pauseFirst,
    readFrom: t.readFrom, disconnected: t.disconnected,
  };
  async function readStart(): Promise<boolean> {
    const pid = live.at?.pid;
    if (pid === undefined) { say('warning', t.noGame); return false; }
    try {
      const got = (await askAll({ask: 'start', pid}))[0] as StartRead;
      /* `String` is the shortest text that reads back as the same number. */
      settings.sx = String(got.x);
      /* No height: the room finds the floor. */
      settings.sy = got.y === undefined ? '' : String(got.y);
      settings.sz = String(got.z);
      settings.sf = String(got.facing);
      settings.scam = String(got.camera);
      fromGame.start = true;
      keep();
      return true;
    } catch (e) {
      /* The game went away between the list and the read. */
      logLine('error', e instanceof Error ? e.message : String(e));
      say('warning', t.noGame);
      return false;
    }
  }
  const connected = $derived(live.at !== null);
  /* The game's own room is read from MEM1, the only read with moving collision; any other room
     from the disc. `null` is nothing to read yet. */
  const source = $derived.by((): {pid?: number; iso?: string} | null => {
    const p = picked, at = live.at, disc = settings.iso.trim();
    if (!p) return null;
    if (at && gameRoom && p.stage === gameRoom.stage && p.room === gameRoom.room) {
      return {pid: at.pid};
    }
    return disc ? {iso: disc} : null;
  });
  /* Machine capabilities once; running emulators while watched, so `Connect` is never stale. */
  void askMachine();
  $effect(() => watch());
  const swings = $derived(TYPES.some(T => chosen(T).some(m => m.sword === 'out')));

  /* Actors only for the room the game is in; a failed ask is an empty list and a log line. */
  $effect(() => {
    const at = live.at, p = picked, g = gameRoom;
    const here = !!g && !!p && p.stage === g.stage && p.room === g.room;
    if (!at || !p || !here) { roomActors = []; return; }
    void (async () => {
      try {
        const [list] = (await askAll({ask: 'actors', stage: p.stage, room: p.room})) as Actor[][];
        roomActors = list ?? [];
      } catch (e) {
        roomActors = [];
        logLine('error', e instanceof Error ? e.message : String(e));
      }
    })();
  });

  $effect(() => {
    const disc = settings.iso.trim();
    if (!disc) { discRooms = []; return; }
    const kept = untrack(() => fromFile);
    void (async () => {
      try {
        const [list] = (await askAll({ask: 'rooms', iso: disc})) as RoomIndex[][];
        discRooms = list ?? [];
      } catch (e) {
        discRooms = [];
        logLine('error', e instanceof Error ? e.message : String(e));
      }
      /* A kept path that reads nothing has moved since; clear it silently. */
      fromFile = false;
      if (!discRooms.length && kept) {
        settings.iso = '';
        keep();
        return;
      }
      if (!discRooms.length) say('error', t.noRoom);
    })();
  });

  /* The room the attached game is in. Connecting selects it; disconnecting leaves the choice. */
  $effect(() => {
    const at = live.at;
    if (!at) { gameRoom = null; return; }
    void (async () => {
      try {
        const [list] = (await askAll({ask: 'rooms', pid: at.pid})) as RoomIndex[][];
        gameRoom = list?.[0] ?? null;
      } catch (e) {
        gameRoom = null;
        logLine('error', e instanceof Error ? e.message : String(e));
      }
      if (gameRoom) picked = gameRoom;
    })();
  });

  /* A chosen room no longer in the list is replaced. */
  $effect(() => {
    const list = rooms;
    const p = untrack(() => picked);
    if (!list.length) { picked = null; return; }
    if (!p || !list.some(r => r.stage === p.stage && r.room === p.room)) {
      /* The last room, when restore is on and it is on this disc; else the first. */
      const back = untrack(() => settings.restore && settings.lastRoom
        ? list.find(r => roomKey(r.stage, r.room) === settings.lastRoom) : undefined);
      picked = back ?? list[0];
    }
  });

  /* Each room keeps its own start, target and bounds. */
  $effect(() => {
    const p = picked;
    if (!p) return;
    untrack(() => atRoom(roomKey(p.stage, p.room)));
  });

  $effect(() => {
    const from = source, p = picked;
    if (!p || !from) { parts = null; return; }
    void (async () => {
      try {
        const got = (await askAll({ask: 'room', stage: p.stage, room: p.room,
                                   ...from}))[0] as RoomRead;
        parts = {ground: new Float32Array(got.ground), wall: new Float32Array(got.wall),
                 roof: new Float32Array(got.roof), seabed: new Float32Array(got.seabed ?? [])};
      } catch (e) {
        parts = null;
        logLine('error', e instanceof Error ? e.message : String(e));
      }
    })();
  });

  /* `null` for empty or not a number. */
  const n = (s: string): number | null => {
    const v = s.trim();
    if (v === '') return null;
    const f = Number(v);
    return Number.isFinite(f) ? f : null;
  };

  const rangeBox = () =>
    ({x0: n(settings.rx1), x1: n(settings.rx2), z0: n(settings.rz1), z1: n(settings.rz2)});

  /** The hull of the address's bytes, each axis ordered, a freed axis `null` at both ends. */
  function addrHull(): {x0: number | null; x1: number | null; z0: number | null; z1: number | null} {
    const b = addrBox(settings.addr, {x: Number(target.tx) || 0, z: Number(target.tz) || 0});
    const pair = (lo: number | null, hi: number | null): [number | null, number | null] =>
      lo === null || hi === null ? [null, null]
        : (Number.isFinite(lo) && Number.isFinite(hi)
            ? [Math.min(lo, hi), Math.max(lo, hi)] : [null, null]);
    const [x0, x1] = pair(b.x0, b.x1);
    const [z0, z1] = pair(b.z0, b.z1);
    return {x0, x1, z0, z1};
  }

  /* Kept apart from `marks`: the ground effect reads this, and reading `marks` (which holds
     `ground.at`) there would be a cycle that stops the app mounting. */
  const targetMark: Marks['target'] = $derived.by(() => {
    let mark: Marks['target'] = null;
    if (settings.shape === 'point') {
      const x = settings.anyx ? null : n(settings.tx), z = settings.anyz ? null : n(settings.tz);
      if ((settings.anyx || x !== null) && (settings.anyz || z !== null)) {
        mark = {kind: 'point' as const, x, z, tol: Math.max(0, n(settings.tol) ?? 0)};
      }
    } else if (settings.shape === 'range') {
      const a = n(settings.rx1), b = n(settings.rx2);
      const c = n(settings.rz1), d = n(settings.rz2);
      if (a !== null && b !== null && c !== null && d !== null) {
        mark = {kind: 'range' as const,
          x0: Math.min(a, b), x1: Math.max(a, b), z0: Math.min(c, d), z1: Math.max(c, d)};
      }
    } else {
      /* Cut to the Range box when checked, as the core cuts the region. */
      const whole = addrHull();
      const {x0, x1, z0, z1} = settings.addrRange ? withinRange(whole, rangeBox()) : whole;
      /* Any typed byte makes a target, even height alone. */
      if (addrMask(settings.addr) !== null) {
        mark = {kind: 'range' as const, x0, x1, z0, z1, address: true};
      }
    }

    return mark;
  });

  const marks: Marks = $derived.by(() => {
    const sxv = n(settings.sx), szv = n(settings.sz);
    const link = sxv !== null && szv !== null
      ? {x: sxv, z: szv, y: n(settings.sy), facing: n(settings.sf)} : null;

    const a = n(settings.fA);
    const facing = settings.fmode === 'any' || a === null
      ? null : {a, b: settings.fmode === 'range' ? n(settings.fB) : null};

    const value = {xmin: n(settings.bValue.xmin), xmax: n(settings.bValue.xmax),
                   zmin: n(settings.bValue.zmin), zmax: n(settings.bValue.zmax)};
    const p = run.chosen;
    const route = p && p.stops.length
      ? {points: [{x: sxv ?? 0, z: szv ?? 0, y: n(settings.sy)},
                  ...p.stops.map(s => ({x: s.x, z: s.z, y: s.y ?? null}))],
         item: aimsAtItem() ? {x: p.x, z: p.z} : null}
      : null;

    const sword = swings ? {on: settings.sword && settings.ground === 'solid'} : null;
    const actors = live.at
      ? {on: settings.actors && settings.ground !== 'none', its: roomActors} : null;

    return {link, route, target: targetMark, ground: ground.at, facing,
            bounds: {open: settings.bOpen, value}, sword, actors};
  });

  /* The first empty box: [label, value, element id]. */
  const empty = $derived([
    [t.location + ' ' + t.x, settings.sx, 'sx'], [t.location + ' ' + t.z, settings.sz, 'sz'],
    [t.location + ' ' + t.link, settings.sf, 'sf'],
    [t.location + ' ' + t.camera, settings.scam, 'scam'],
    ...(settings.shape === 'point' && !settings.anyx
      ? [[t.target + ' ' + t.x, settings.tx, 'tx']] : []),
    ...(settings.shape === 'point' && !settings.anyz
      ? [[t.target + ' ' + t.z, settings.tz, 'tz']] : []),
    ...(settings.shape === 'range'
      ? [[t.target + ' ' + t.x, settings.rx1, 'rx1'],
         [t.target + ' ' + t.xMax, settings.rx2, 'rx2'],
         [t.target + ' ' + t.z, settings.rz1, 'rz1'],
         [t.target + ' ' + t.zMax, settings.rz2, 'rz2']] : []),
    ...((Object.keys(settings.bOpen) as Side[]).filter(s => !settings.bOpen[s])
      .map(s => [t.bounds + ' ' + boundLabel[s], settings.bValue[s], 'b' + s])),
    ...(settings.fmode !== 'any' ? [[t.target + ' ' + t.facing, settings.fA, 'fa']] : []),
  ].find(([, v]) => (v as string).trim() === ''));
  const blocked = $derived(
    empty ? t.blockedEmpty(empty[0] as string)
      : settings.shape === 'addr' && !isRun(settings.addr) ? t.blockedAddress
      : total() === 0 ? t.blockedNoMoves : null);
  /* Apart from `blocked` because it dims `Search` without a message. */
  const noDisc = $derived(settings.iso.trim() === '');
  function reach(): void {
    const box = empty ? document.getElementById(empty[2] as string) : null;
    box?.focus();
  }

  /** `null` for an open or empty side. */
  const sideOf = (side: Side): number | null =>
    settings.bOpen[side] ? null : (n(settings.bValue[side]) ?? null);

  /* The target as the room draws it, plus an address's height, mask and range. Shared by the
     search and the ground query so both use one target. */
  function askedTarget(): AskedTarget {
    /* Not `marks.target`: that would subscribe the ground effect to its own answer. */
    const tm = targetMark;
    const height = settings.shape === 'addr' ? addrY(settings.addr) : null;
    /* The hull alone over-covers when a high byte is blank. */
    const mask = settings.shape === 'addr' ? addrMask(settings.addr) : null;
    /* Sent apart from the uncut box, which the corridor is laid over; the core cuts only the region. */
    const r = rangeBox();
    const within = settings.shape === 'addr' && settings.addrRange
      && r.x0 !== null && r.x1 !== null && r.z0 !== null && r.z1 !== null
      ? {x0: r.x0, x1: r.x1, z0: r.z0, z1: r.z1} : null;
    const box = within ? addrHull() : tm;
    return tm && tm.kind === 'range' && box && 'x0' in box
      ? {shape: 'range' as const, x0: box.x0, x1: box.x1, z0: box.z0, z1: box.z1,
         ...(height ?? {}), ...(mask ? {mask} : {}), ...(within ? {within} : {})}
      : {shape: 'point' as const, x: tm && tm.kind === 'point' ? tm.x : null,
         z: tm && tm.kind === 'point' ? tm.z : null};
  }

  /* Where the target meets the floor, re-asked as the question changes. */
  $effect(() => {
    const disc = settings.iso.trim();
    followGround(picked && targetMark && disc ? {
      target: askedTarget(),
      start: {x: n(settings.sx) ?? 0, z: n(settings.sz) ?? 0,
              f: wrap(readFacing(settings.sf) || 0)},
      tol: Math.max(0, n(settings.tol) ?? 0),
      aim: settings.aim === 'overhead' ? 'overhead' : 'player',
      stage: picked.stage, room: picked.room,
      ...(disc ? {iso: disc} : {}), ...(live.at ? {pid: live.at.pid} : {}),
    } : null);
  });

  /* The target floor needs Link's data from the disc; warned once per target set without one. */
  let warnedNoDisc = false;
  $effect(() => {
    const standing = !!targetMark && noDisc;
    if (standing && !warnedNoDisc) say('warning', t.noDiscTargets);
    warnedNoDisc = standing;
  });

  function go(): void {
    if (run.running) { stopRun(); return; }
    if (noDisc) return;
    if (blocked) { reach(); return; }
    const tm = targetMark;
    const box = tm && tm.kind === 'range'
      ? {x0: tm.x0, x1: tm.x1, z0: tm.z0, z1: tm.z1}
      : null;
    const aimAt = askedTarget();
    /* The disc always goes: verification needs all six collision tables, which only it has. */
    const disc = settings.iso.trim();
    void startRun({
      /* Turnarounds aim at `camera + k * 0x4000`. An empty box omits `cam`, since `readFacing` reads
         "" as 0. */
      start: {x: n(settings.sx) ?? 0, y: n(settings.sy) ?? undefined, z: n(settings.sz) ?? 0,
              f: wrap(readFacing(settings.sf) || 0),
              ...(settings.scam.trim() === ''
                    ? {} : {cam: wrap(readFacing(settings.scam) || 0)})},
      target: aimAt,
      /* Absent means any facing. */
      ...(marks.facing ? {facing: marks.facing} : {}),
      asked: {box, tol: Math.max(0, n(settings.tol) ?? 0), aim: settings.aim},
      steps: Math.max(0, settings.steps), frames: Math.max(0, settings.frames),
      room: {stage: picked?.stage ?? '', room: picked?.room ?? 0,
             ...(disc ? {iso: disc} : {}), ...(live.at ? {pid: live.at.pid} : {})},
      moves: TYPES.flatMap(T => chosen(T).map(m => m.id)),
      costs: costs(),
      collision: settings.ground,
      /* Capped at this machine's cores; a kept value may come from a bigger one. */
      cores: Math.max(1, Math.min(live.cores, Math.floor(settings.cores) || 1)),
      /* 4 is the core's own default. */
      checkRange: Number.isFinite(settings.checkRange) && settings.checkRange >= 0
                    ? settings.checkRange : 4,
      bounds: {
        xmin: sideOf('xmin'), xmax: sideOf('xmax'),
        zmin: sideOf('zmin'), zmax: sideOf('zmax'),
      },
    }, (sev, text, opensLogs, stay) => say(sev, text, null, stay ?? false, opensLogs),
       {reach: t.reachCount, none: t.noneReach, stopped: t.stopped, nothing: t.nothingFound,
        broke: t.searchBroke, noGround: t.noValidTargets,
        verified: t.verifiedFigure, engine: t.engineFigure,
        memory: t.memoryFigure, memoryFull: t.memoryFull});
  }

  $effect(() => {
    document.body.className = run.phase + (run.running ? ' running' : '');
  });

  /* Enter presses `Search` unless the focus has its own Enter; Escape goes back to design. */
  function onKey(e: KeyboardEvent): void {
    const inWindow = !!document.querySelector('dialog[open]');
    if (e.key === 'Escape' && run.phase !== 'design' && !inWindow) { run.phase = 'design'; return; }
    if (e.key === 'Enter' && !e.isComposing && !e.ctrlKey && !e.metaKey && !e.altKey && !inWindow &&
        !(e.target as HTMLElement | null)?.closest?.('button, a, tr, textarea')) {
      e.preventDefault();
      go();
    }
  }

  /* Link's start is off every floor of the new room, so the whole start goes. */
  function clearLocation(): void {
    settings.sx = '';
    settings.sy = '';
    settings.sz = '';
    settings.sf = '';
    settings.scam = '';
    fromGame.start = false;
    keep();
  }

  async function copyText(text: string): Promise<boolean> {
    try { await navigator.clipboard.writeText(text); return true; } catch { return false; }
  }

  const boundLabel: Record<Side, string> = {
    xmin: t.xMin, xmax: t.xMax, zmin: t.zMin, zmax: t.zMax};

  const where = $derived(picked ? t.roomAt(picked.stage, picked.room) : t.noRoom);

  /* What stops Link; everything is drawn either way. */
  const solid = $derived({
    ground: settings.ground !== 'none',
    wall: settings.ground === 'solid',
    /* A roof stops Link where walls do. */
    roof: settings.ground === 'solid',
  });
</script>

<svelte:window onkeydown={onKey} />

<a class="skip" href="#view">{t.skipToRoom}</a>

<div class="app">
  <Bar moveCount={total()} {where} {connected} busy={live.busy}
       canConnect={connected || canConnect()} canStage={discRooms.length > 0}
       canAttach={!settings.iso && canChoose()} attach={() => void browseDisc()}
       conn={() => void toggle(say, words, readStart, () => instOpen = true)}
       stage={() => stageOpen = true}
       options={() => optionsOpen = true} moves={() => movesOpen = true} />
  <div class="work">
    <!-- svelte-ignore a11y_no_noninteractive_tabindex
         Gutters are drag handles that also take keyboard focus. -->
    <div class="rail" use:rail>
      <Location live={connected}
                update={() => void readStart().then(ok => { if (ok) say('info', t.updated); })} />
      <div class="gutter h" id="gLoc" role="separator" aria-orientation="horizontal" tabindex="0"
           aria-label={t.locationHeight} use:vgutter={'pLoc'}></div>
      <Target />
      <div class="gutter h" id="gTgt" role="separator" aria-orientation="horizontal" tabindex="0"
           aria-label={t.targetHeight} use:vgutter={'pTgt'}></div>
      <Bounds />
    </div>
    <!-- svelte-ignore a11y_no_noninteractive_tabindex -->
    <div class="gutter" id="gRail" role="separator" aria-orientation="vertical" tabindex="0"
         aria-label={t.questionWidth} use:hgutter></div>
    <Room {picked} {parts} {marks} {solid} phase={run.phase} canFwd={shown().length > 0}
          copy={copyText} offFloor={clearLocation}
          back={() => run.phase = 'design'} fwd={() => run.phase = 'done'} />

    <!-- svelte-ignore a11y_no_noninteractive_tabindex -->
    <div class="gutter" id="gutterV" role="separator" aria-orientation="vertical" tabindex="0"
         aria-label={t.answersWidth} use:gutter={{name: '--answers', min: 18, max: 70, from: 'right'}}
    ></div>
    <Answers copy={copyText} />
  </div>
  <RunBar {blocked} {noDisc} {reach} logs={() => logsOpen = true} logCount={log.length}
          live={connected} {swings}
          {go} running={run.running} pct={run.pct} figures={run.figures} ended={run.ended} />
  <Msgs logs={() => logsOpen = true} />
  <Logs bind:open={logsOpen} />
  <Stage bind:open={stageOpen} {rooms} {picked} choose={(it) => picked = it} />
  <Options bind:open={optionsOpen} browse={() => void browseDisc()} took={tookDisc}
           {askTo} />
  <Moves bind:open={movesOpen} />
  <Ask bind:open={askOpen} title={asked.title} line={asked.line} yes={asked.yes}
       confirm={asked.run} />
  <Emulators bind:open={instOpen}
             take={(e: Emulator) => void connect(e, say, words, readStart)} />
</div>
