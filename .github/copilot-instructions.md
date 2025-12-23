# GitHub Copilot Instructions for termviz

These notes give an AI coding agent the minimum, project-specific context to be productive when editing or extending termviz (a terminal-based ROS visualizer written in Rust).

Keep instructions short, concrete and tied to real files. Do not assume any GUI; this is a TUI app using `tui` + `crossterm` and ROS bindings.

1) Big-picture architecture
- **Single binary TUI app**: entry point is `src/main.rs` which initializes ROS (`rosrust`), a TF listener (`rustros_tf::TfListener`), loads config (`src/config.rs`) and runs the `App` loop.
- **App + modes**: `src/app.rs` builds an `App` that composes multiple modes (in `src/app_modes/`). Modes implement the `BaseMode` trait in `src/app_modes/mod.rs`. Modes include `send_pose`, `teleoperate`, `image_view`, `topic_managment`, and `viewport`.
- **Listeners as adapters**: `src/listeners.rs` constructs topic-specific listener objects (e.g., `laser::LaserListener`, `map::MapListener`, `pointcloud::PointCloud2Listener`). Each listener converts ROS messages into in-memory structures used by the `viewport` and modes.
- **Rendering and input flow**: `App::run()` advances mode state; `App::draw()` delegates to the active mode. `main.rs` reads terminal events and maps keys to `app_modes::input` strings, which modes handle via `handle_input`.

2) Key files to inspect when changing behavior
- `src/main.rs` — process lifecycle, TF waiting logic, rate loop, key-mapping setup.
- `src/config.rs` — configuration loading, defaults, and `confy`-based user config persistence. Useful when adding new config options.
- `src/app.rs` — terminal initialization, mode switching, and help display generation.
- `src/listeners.rs` and any `src/*/listener` modules (e.g. `laser.rs`, `map.rs`, `pointcloud.rs`, `marker.rs`, `pose.rs`) — where ROS subscriptions are created and translated.
- `src/app_modes/` — mode implementations. Use `viewport.rs` for coordinate transforms and frame-related logic.

3) Build / test / CI notes (how maintainers run things)
- Build: `cargo build --release` (output: `target/release/termviz`).
- Tests: `cargo test`.
- CI in `.github/workflows/rust.yml` installs ROS message Debian packages before building. Locally you may need `ROSRUST_MSG_PATH=/usr/share/` in the environment when building/tests if system ROS message crates are required.
- Formatting check used in CI: `cargo fmt --all -- --check`.
- Release packaging uses `cargo-dist` in `.github/workflows/release.yml`.

4) Runtime and integration points
- Requires a running ROS master; `rosrust::init("termviz")` is called in `main.rs`.
- TF: relies on `rustros_tf::TfListener`. On startup `main.rs` waits up to `--tf-wait-time` seconds for a transform between `fixed_frame` and `robot_frame`. If not present the app uses origin and prompts the user.
- Config files: default user path is `~/.config/termviz/termviz.yml` (via `confy`) and global default `/etc/termviz/termviz.yml`. `config::get_config` will store an updated user config unless a custom path was provided.

5) Project-specific conventions and patterns
- Key mapping: `config.key_mapping` maps `app_modes::input` semantic strings to physical keys. `main.rs` translates characters to `KeyCode` and maps to the app input strings. When adding new commands add the input name to `src/app_modes/mod.rs` and provide a default in `config.rs`.
- Modes are boxed and stored in `App` as `Vec<Box<dyn BaseMode<B>>>`. When adding a mode follow the pattern in `src/app.rs::new()` to construct listeners/viewport and pass shared `Rc<RefCell<...>>` or `Arc` where appropriate.
- Concurrency & ownership: TF listener and ROS subscriptions use `Arc` for shared ownership. UI uses `Rc<RefCell<...>>` for interior mutability in single-threaded TUI context.
- Colors: colors are defined in `src/config.rs` as an RGB struct with `to_tui()` helper returning `tui::style::Color::Rgb(r,g,b)`.

6) Common small tasks and examples
- Add a new config option: update `TermvizConfig` in `src/config.rs`, add default in `Default` implementation and read it in the consumer module. `confy` will auto-load/store the user config.
- Add a new mode: implement `BaseMode` (see `app_modes/viewport.rs` for examples), add construction in `App::new()` and an entry in the `key_mapping` defaults if it needs a key.
- Add a new ROS listener: create a listener module (see `laser.rs` or `map.rs`) and register it in `listeners.rs::new()` and pass topics via `TermvizConfig`.

7) Tests and side-effects to watch for
- Many modules interact with ROS and TF. Unit tests that depend on message types may require `ROSRUST_MSG_PATH` or installed system `ros-*-msgs` packages (see CI). Mocking ROS may be necessary for pure-unit tests.
- Terminal state: tests that call `App::init_terminal()` affect the terminal (raw mode). Prefer testing logic without initializing the terminal.

8) Editing guidance for AI agents
- Keep changes minimal and follow existing ownership (Arc vs Rc) conventions.
- When adding public APIs, prefer small focused changes and update `README.md` or default config where applicable.
- Preserve `confy` persistence behavior: do not remove the user config write unless explicitly requested.
- For UI changes, follow the `tui` widget patterns used in `app.rs::show_help`.

If anything above is unclear or you'd like more examples (e.g., a new mode scaffold), tell me which area to expand and I will iterate.
