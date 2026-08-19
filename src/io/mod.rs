//! Data sources: offline rosbag2 (MCAP) reading and live ROS 2 topics.
//!
//! Both produce the shared protocol-neutral [`model::LogFile`] that the DSP /
//! pipeline / GUI layers consume.

pub mod cdr;
pub mod convert;
// The live store is consumed only by the feature-gated rclrs client; it is
// still compiled (and its protocol-independent tests run) in default builds,
// so silence dead-code warnings there.
#[cfg_attr(not(feature = "ros2"), allow(dead_code))]
pub mod live_store;
pub mod model;
pub mod rosbag_loader;

// The rclrs-based live client requires a ROS 2 distro; it is only compiled
// with the `ros2` cargo feature.
#[cfg(feature = "ros2")]
pub mod ros2_live;

#[cfg(feature = "ros2")]
pub use ros2_live::{LiveStatus, LiveTopicConfig, Ros2LiveClient};
