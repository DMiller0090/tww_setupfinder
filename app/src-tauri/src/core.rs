//! A pipe to the C++ core: one child, one line-delimited request at a time, no parsing beyond
//! spotting the end of an answer. The child exits when its stdin closes with this process.

use std::io::{BufRead, BufReader, Write};
use std::path::PathBuf;
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Mutex;

use serde::Serialize;
use tauri::{AppHandle, Emitter, State};

/// Every core line is emitted here; `id` names the request.
const EVENT: &str = "core://line";

#[derive(Clone, Serialize)]
struct Line {
    id: String,
    line: String,
}

struct Running {
    child: Child,
    stdout: BufReader<ChildStdout>,
}

/// Started lazily on the first request. `running` serialises requests (the protocol has no ids);
/// `sending` is separate so `signal` can write while a request is waiting on its answer.
/// Lock order is `running` then `sending`.
#[derive(Default)]
pub struct Core {
    running: Mutex<Option<Running>>,
    sending: Mutex<Option<ChildStdin>>,
}

/// Where `setupcore` may be: beside the app first (release), then `core/build/<config>/` under
/// any ancestor (development).
fn candidates() -> Vec<PathBuf> {
    let exe_name = if cfg!(windows) { "setupcore.exe" } else { "setupcore" };
    let mut out = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            out.push(dir.join(exe_name));
            for up in dir.ancestors() {
                for config in ["Release", "Debug", ""] {
                    out.push(up.join("core").join("build").join(config).join(exe_name));
                }
            }
        }
    }
    out
}

fn spawn() -> Result<(Running, ChildStdin), String> {
    let tried = candidates();
    let found = tried
        .iter()
        .find(|p| p.is_file())
        .ok_or_else(|| "the core is not built - run `core\\build.bat`".to_string())?;

    let mut command = Command::new(found);
    command.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::inherit());
    #[cfg(windows)]
    {
        // CREATE_NO_WINDOW: the core is a console binary.
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x0800_0000);
    }
    let mut child = command.spawn().map_err(|e| format!("could not start the core: {e}"))?;
    let stdin = child.stdin.take().ok_or("the core took no stdin")?;
    let stdout = child.stdout.take().ok_or("the core gave no stdout")?;
    Ok((Running { child, stdout: BufReader::new(stdout) }, stdin))
}

/// Send one request and emit each reply line as it arrives, returning on `done` or `fail`.
/// Blocks on the pipe; `async` only moves it off the main thread.
#[tauri::command(async)]
pub fn ask(app: AppHandle, core: State<'_, Core>, id: String, request: String) -> Result<(), String> {
    if request.contains('\n') {
        return Err("a request cannot contain a newline".into());
    }
    let mut held = core.running.lock().map_err(|_| "the core's lock is poisoned".to_string())?;
    if held.is_none() {
        let (mut running, stdin) = spawn()?;
        // Kill the child if its stdin cannot be stored, or it is orphaned.
        match core.sending.lock() {
            Ok(mut sending) => *sending = Some(stdin),
            Err(_) => {
                let _ = running.child.kill();
                return Err("the way in is poisoned".into());
            }
        }
        *held = Some(running);
    }

    // Released before reading, so `signal` can write meanwhile.
    let wrote = {
        let mut sending = core.sending.lock().map_err(|_| "the way in is poisoned".to_string())?;
        match sending.as_mut() {
            Some(pipe) => writeln!(pipe, "{request}").and_then(|_| pipe.flush()),
            None => Err(std::io::Error::new(std::io::ErrorKind::BrokenPipe, "no way in")),
        }
    };
    let running = held.as_mut().expect("just started");
    if let Err(e) = wrote {
        // The core died; forget it so the next request respawns.
        let _ = running.child.kill();
        *held = None;
        if let Ok(mut sending) = core.sending.lock() {
            *sending = None;
        }
        return Err(format!("the core stopped listening: {e}"));
    }

    loop {
        let mut line = String::new();
        match running.stdout.read_line(&mut line) {
            Ok(0) => {
                let _ = running.child.kill();
                *held = None;
                if let Ok(mut sending) = core.sending.lock() {
                    *sending = None;
                }
                return Err("the core stopped answering".into());
            }
            Ok(_) => {}
            Err(e) => {
                let _ = running.child.kill();
                *held = None;
                if let Ok(mut sending) = core.sending.lock() {
                    *sending = None;
                }
                return Err(format!("could not read the core: {e}"));
            }
        }
        let text = line.trim_end_matches(['\r', '\n']).to_string();
        if text.is_empty() {
            continue;
        }
        // Parsed, not substring-matched: a `fail` reason may quote `"line":"done"`.
        let ended = serde_json::from_str::<serde_json::Value>(&text)
            .ok()
            .and_then(|v| v.get("line").and_then(|k| k.as_str()).map(|k| k == "done" || k == "fail"))
            .unwrap_or(false);
        let _ = app.emit(EVENT, Line { id: id.clone(), line: text });
        if ended {
            return Ok(());
        }
    }
}

/// Write one line with no reply (how `Stop` reaches a running search); a no-op with no child.
/// Not `async`: blocked `ask` calls can fill the async pool.
#[tauri::command]
pub fn signal(core: State<'_, Core>, line: String) -> Result<(), String> {
    if line.contains('\n') {
        return Err("a signal cannot contain a newline".into());
    }
    let mut sending = core.sending.lock().map_err(|_| "the way in is poisoned".to_string())?;
    match sending.as_mut() {
        Some(pipe) => writeln!(pipe, "{line}")
            .and_then(|_| pipe.flush())
            .map_err(|e| format!("the core stopped listening: {e}")),
        None => Ok(()),
    }
}
