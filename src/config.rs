use crate::app_modes::input;
use confy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use strum_macros::Display;
use tui::style::Color as TuiColor;

const DEFAULT_PATH: &str = "/etc/termviz/termviz.yml";

const fn default_int() -> i64 {
    0
}

const fn default_map_threshold() -> i8 {
    1
}

const fn default_pose_length() -> f64 {
    0.2
}

const fn default_teleop_max_vel() -> f64 {
    0.2
}

const fn default_teleop_increment() -> f64 {
    0.1
}

const fn default_teleop_increment_step() -> f64 {
    0.1
}

fn default_teleop_cmd_vel_topic() -> String {
    "cmd_vel".to_string()
}

const fn default_publish_cmd_vel_when_idle() -> bool {
    true
}

const fn color_white() -> Color {
    Color {
        r: 255,
        g: 255,
        b: 255,
    }
}

const fn color_red() -> Color {
    Color { r: 255, g: 0, b: 0 }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Color {
    pub r: u8,
    pub b: u8,
    pub g: u8,
}

impl Color {
    pub fn to_tui(&self) -> TuiColor {
        return TuiColor::Rgb(self.r, self.g, self.b);
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfig {
    pub topic: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PointCloud2ListenerConfig {
    pub topic: String,
    #[serde(default = "bool::default")]
    pub use_rgb: bool,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PoseListenerConfig {
    pub topic: String,
    pub style: String,
    #[serde(default = "color_red")]
    pub color: Color,
    #[serde(default = "default_pose_length")]
    pub length: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ImageListenerConfig {
    pub topic: String,
    #[serde(default = "default_int")]
    pub rotation: i64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SendPoseConfig {
    pub topic: String,
    pub msg_type: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfigColor {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MapListenerConfig {
    pub topic: String,
    #[serde(default = "color_white")]
    pub color: Color,
    #[serde(default = "default_map_threshold")]
    pub threshold: i8,
}

#[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq, Eq, Display)]
pub enum TeleopMode {
    // Node will keep speed button press only increment decrement
    Classic,
    // Speed will be reduced every iteration by one increment
    // To keep driving a button needs to be pressed every increment
    #[default]
    Safe,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TeleopConfig {
    #[serde(default = "default_teleop_increment")]
    pub default_increment: f64,
    #[serde(default = "default_teleop_increment_step")]
    pub increment_step: f64,
    #[serde(default = "default_teleop_cmd_vel_topic")]
    pub cmd_vel_topic: String,
    #[serde(default = "default_publish_cmd_vel_when_idle")]
    pub publish_cmd_vel_when_idle: bool,
    #[serde(default)]
    pub mode: TeleopMode,
    #[serde(default = "default_teleop_max_vel")]
    pub max_vel: f64,
}

impl Default for TeleopConfig {
    fn default() -> TeleopConfig {
        TeleopConfig {
            default_increment: 0.1,
            increment_step: 0.1,
            cmd_vel_topic: "cmd_vel".to_string(),
            publish_cmd_vel_when_idle: true,
            mode: TeleopMode::default(),
            max_vel: 0.2,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(from = "TermvizConfigPartial")]
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub robot_frame: String,
    pub map_topics: Vec<MapListenerConfig>,
    pub laser_topics: Vec<ListenerConfigColor>,
    pub marker_topics: Vec<ListenerConfig>,
    pub image_topics: Vec<ImageListenerConfig>,
    pub marker_array_topics: Vec<ListenerConfig>,
    pub path_topics: Vec<PoseListenerConfig>,
    pub pointcloud2_topics: Vec<PointCloud2ListenerConfig>,
    pub polygon_stamped_topics: Vec<ListenerConfigColor>,
    pub pose_array_topics: Vec<PoseListenerConfig>,
    pub pose_stamped_topics: Vec<PoseListenerConfig>,
    pub send_pose_topics: Vec<SendPoseConfig>,
    pub target_framerate: i64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    pub zoom_factor: f64,
    pub key_mapping: HashMap<String, String>,
        #[serde(default)]
    pub teleop: TeleopConfig,
}

#[derive(Debug, Deserialize, Default)]
struct TermvizConfigPartial {
    fixed_frame: Option<String>,
    robot_frame: Option<String>,
    map_topics: Option<Vec<MapListenerConfig>>,
    laser_topics: Option<Vec<ListenerConfigColor>>,
    marker_topics: Option<Vec<ListenerConfig>>,
    image_topics: Option<Vec<ImageListenerConfig>>,
    marker_array_topics: Option<Vec<ListenerConfig>>,
    path_topics: Option<Vec<PoseListenerConfig>>,
    pointcloud2_topics: Option<Vec<PointCloud2ListenerConfig>>,
    polygon_stamped_topics: Option<Vec<ListenerConfigColor>>,
    pose_array_topics: Option<Vec<PoseListenerConfig>>,
    pose_stamped_topics: Option<Vec<PoseListenerConfig>>,
    send_pose_topics: Option<Vec<SendPoseConfig>>,
    target_framerate: Option<i64>,
    axis_length: Option<f64>,
    visible_area: Option<Vec<f64>>,
    zoom_factor: Option<f64>,
    key_mapping: Option<HashMap<String, String>>,
    teleop: Option<TeleopConfig>,
}

impl From<TermvizConfigPartial> for TermvizConfig {
    fn from(partial: TermvizConfigPartial) -> Self {
        let mut cfg = TermvizConfig::default();

        if let Some(v) = partial.fixed_frame {
            cfg.fixed_frame = v;
        }
        if let Some(v) = partial.robot_frame {
            cfg.robot_frame = v;
        }
        if let Some(v) = partial.map_topics {
            cfg.map_topics = v;
        }
        if let Some(v) = partial.laser_topics {
            cfg.laser_topics = v;
        }
        if let Some(v) = partial.marker_topics {
            cfg.marker_topics = v;
        }
        if let Some(v) = partial.image_topics {
            cfg.image_topics = v;
        }
        if let Some(v) = partial.marker_array_topics {
            cfg.marker_array_topics = v;
        }
        if let Some(v) = partial.path_topics {
            cfg.path_topics = v;
        }
        if let Some(v) = partial.pointcloud2_topics {
            cfg.pointcloud2_topics = v;
        }
        if let Some(v) = partial.polygon_stamped_topics {
            cfg.polygon_stamped_topics = v;
        }
        if let Some(v) = partial.pose_array_topics {
            cfg.pose_array_topics = v;
        }
        if let Some(v) = partial.pose_stamped_topics {
            cfg.pose_stamped_topics = v;
        }
        if let Some(v) = partial.send_pose_topics {
            cfg.send_pose_topics = v;
        }
        if let Some(v) = partial.target_framerate {
            cfg.target_framerate = v;
        }
        if let Some(v) = partial.axis_length {
            cfg.axis_length = v;
        }
        if let Some(v) = partial.visible_area {
            cfg.visible_area = v;
        }
        if let Some(v) = partial.zoom_factor {
            cfg.zoom_factor = v;
        }
        if let Some(v) = partial.key_mapping {
            cfg.key_mapping = v;
        }
        if let Some(v) = partial.teleop {
            cfg.teleop = v;
        }

        cfg
    }
}

#[cfg(test)]
mod tests {
        use super::*;
        use std::io::Write;
        use tempfile::NamedTempFile;

        fn write_config(contents: &str) -> NamedTempFile {
                let mut file = NamedTempFile::new().expect("temp file");
                file.write_all(contents.as_bytes()).expect("write yaml");
                file
        }

        #[test]
        fn loads_old_yaml_missing_new_teleop_field() {
                // Intentionally omit `teleop.publish_cmd_vel_when_idle` to verify backward compat.
            // Also omit `pointcloud2_topics` to verify newly-added top-level fields don't break older configs.
                                let yaml = r#"
fixed_frame: map
robot_frame: base_link

map_topics:
-
    topic: map
    color:
        r: 255
        g: 255
        b: 255
    threshold: 1
laser_topics: []
marker_topics: []
image_topics: []
marker_array_topics: []
path_topics: []
polygon_stamped_topics: []
pose_array_topics: []
pose_stamped_topics: []
send_pose_topics:
-
    topic: initialpose
    msg_type: PoseWithCovarianceStamped

target_framerate: 10
axis_length: 0.5
visible_area: [ -5.0, 5.0, -5.0, 5.0 ]
zoom_factor: 0.1

key_mapping: {}

teleop:
    default_increment: 0.1
    increment_step: 0.1
    cmd_vel_topic: cmd_vel
    mode: Safe
    max_vel: 0.2
"#;

                let file = write_config(yaml);
                let cfg: TermvizConfig = confy::load_path(file.path()).expect("load config");
                assert!(cfg.teleop.publish_cmd_vel_when_idle);
                assert!(!cfg.pointcloud2_topics.is_empty());
                assert_eq!(cfg.pointcloud2_topics[0].topic, "pointcloud2");
        }

        #[test]
        fn loads_yaml_without_teleop_block() {
                                let yaml = r#"
fixed_frame: map
robot_frame: base_link

map_topics:
-
    topic: map
    color:
        r: 255
        g: 255
        b: 255
    threshold: 1
laser_topics: []
marker_topics: []
image_topics: []
marker_array_topics: []
path_topics: []
pointcloud2_topics: []
polygon_stamped_topics: []
pose_array_topics: []
pose_stamped_topics: []
send_pose_topics:
-
    topic: initialpose
    msg_type: PoseWithCovarianceStamped

target_framerate: 10
axis_length: 0.5
visible_area: [ -5.0, 5.0, -5.0, 5.0 ]
zoom_factor: 0.1

key_mapping: {}
"#;

                let file = write_config(yaml);
                let cfg: TermvizConfig = confy::load_path(file.path()).expect("load config");

                let default = TeleopConfig::default();
                assert_eq!(cfg.teleop.default_increment, default.default_increment);
                assert_eq!(cfg.teleop.increment_step, default.increment_step);
                assert_eq!(cfg.teleop.cmd_vel_topic, default.cmd_vel_topic);
                assert_eq!(cfg.teleop.publish_cmd_vel_when_idle, default.publish_cmd_vel_when_idle);
                assert_eq!(cfg.teleop.mode, default.mode);
                assert_eq!(cfg.teleop.max_vel, default.max_vel);
        }
}

impl Default for TermvizConfig {
    fn default() -> Self {
        TermvizConfig {
            fixed_frame: "map".to_string(),
            robot_frame: "base_link".to_string(),
            map_topics: vec![MapListenerConfig {
                topic: "map".to_string(),
                color: Color {
                    r: 255,
                    b: 255,
                    g: 255,
                },
                threshold: 1,
            }],
            laser_topics: vec![ListenerConfigColor {
                topic: "scan".to_string(),
                color: Color { r: 200, b: 0, g: 0 },
            }],
            marker_array_topics: vec![ListenerConfig {
                topic: "marker_array".to_string(),
            }],
            marker_topics: vec![ListenerConfig {
                topic: "marker".to_string(),
            }],
            image_topics: vec![ImageListenerConfig {
                topic: "image_rect".to_string(),
                rotation: 0,
            }],
            pose_stamped_topics: vec![PoseListenerConfig {
                topic: "pose_stamped".to_string(),
                style: "axis".to_string(),
                color: Color { r: 255, g: 0, b: 0 },
                length: 0.2,
            }],
            pose_array_topics: vec![PoseListenerConfig {
                topic: "pose_array".to_string(),
                style: "arrow".to_string(),
                color: Color { r: 255, g: 0, b: 0 },
                length: 0.2,
            }],
            path_topics: vec![PoseListenerConfig {
                topic: "path".to_string(),
                style: "line".to_string(),
                color: Color { r: 0, g: 255, b: 0 },
                length: 0.2,
            }],
            pointcloud2_topics: vec![PointCloud2ListenerConfig {
                topic: "pointcloud2".to_string(),
                use_rgb: false,
            }],
            polygon_stamped_topics: vec![ListenerConfigColor {
                topic: "footprint".to_string(),
                color: Color { r: 200, b: 0, g: 0 },
            }],
            send_pose_topics: vec![SendPoseConfig {
                topic: "initialpose".to_string(),
                msg_type: "PoseWithCovarianceStamped".to_string(),
            }],
            target_framerate: 30,
            axis_length: 0.5,
            visible_area: vec![-5., 5., -5., 5.],
            zoom_factor: 0.1,
            key_mapping: HashMap::from([
                (input::UP.to_string(), "w".to_string()),
                (input::DOWN.to_string(), "s".to_string()),
                (input::LEFT.to_string(), "a".to_string()),
                (input::RIGHT.to_string(), "d".to_string()),
                (input::ROTATE_LEFT.to_string(), "q".to_string()),
                (input::ROTATE_RIGHT.to_string(), "e".to_string()),
                (input::CANCEL.to_string(), "Esc".to_string()),
                (input::CONFIRM.to_string(), "Enter".to_string()),
                (input::ZOOM_IN.to_string(), "=".to_string()),
                (input::ZOOM_OUT.to_string(), "-".to_string()),
                (input::INCREMENT_STEP.to_string(), "k".to_string()),
                (input::DECREMENT_STEP.to_string(), "j".to_string()),
                (input::NEXT.to_string(), "n".to_string()),
                (input::PREVIOUS.to_string(), "b".to_string()),
                (input::SHOW_HELP.to_string(), "h".to_string()),
                (input::MODE_2.to_string(), "t".to_string()),
                (input::MODE_3.to_string(), "i".to_string()),
            ]),
            teleop: TeleopConfig::default(),
        }
    }
}

/// Loads the config with the following priority:
/// 1. from a custom path, if provided;
/// 2. from the default user config path, if it exists;
/// 3. from the default global config path, if it exists;
/// 4. or with default values.
pub fn get_config(custom_path: Option<&PathBuf>) -> Result<TermvizConfig, confy::ConfyError> {
    let default_path = PathBuf::from(DEFAULT_PATH);
    let user_path = confy::get_configuration_file_path("termviz", "termviz")?;

    let load_config_path = custom_path.unwrap_or_else(|| {
        if user_path.exists() {
            &user_path
        } else {
            &default_path
        }
    });

    let cfg = if load_config_path.exists() {
        println!("Loading config from: {:?}", load_config_path);
        confy::load_path(&load_config_path)?
    } else {
        println!("No config found at {:?}, using defaults.", load_config_path);
        TermvizConfig::default()
    };

    // Always update user config, unless a custom path was used.
    if custom_path.is_none() {
        match confy::store_path(&user_path.as_path(), &cfg) {
            Ok(_) => println!("Updated {:?}", user_path),
            Err(e) => println!("Error updating config at user path: {:?}", e),
        }
    }
    Ok(cfg)
}
