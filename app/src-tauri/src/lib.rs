mod core;

use tauri::Manager;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
  tauri::Builder::default()
    /* Native file dialog: a WebView file input gives only a filename, not a path. */
    .plugin(tauri_plugin_dialog::init())
    .manage(core::Core::default())
    .invoke_handler(tauri::generate_handler![core::ask, core::signal])
    .setup(|app| {
      /* Sets only ICON_SMALL, so the title bar draws the source PNG instead of a resampled .ico
         entry; on failure the exe icon stays. */
      if let Some(window) = app.get_webview_window("main") {
        match tauri::image::Image::from_bytes(include_bytes!("../icons/titlebar.png")) {
          Ok(mark) => { let _ = window.set_icon(mark); }
          Err(e) => log::warn!("the title bar mark did not decode: {e}"),
        }
      }
      if cfg!(debug_assertions) {
        app.handle().plugin(
          tauri_plugin_log::Builder::default()
            .level(log::LevelFilter::Info)
            .build(),
        )?;
      }
      Ok(())
    })
    .run(tauri::generate_context!())
    .expect("error while running tauri application");
}
