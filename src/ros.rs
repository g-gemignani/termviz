#[cfg(all(feature = "ros1", feature = "ros2"))]
compile_error!("Features 'ros1' and 'ros2' are mutually exclusive");

#[cfg(feature = "ros2")]
compile_error!(
    "ROS 2 backend not implemented yet. Build with default features (ROS1) for now."
);

#[cfg(feature = "ros1")]
pub use rosrust;

#[cfg(feature = "ros1")]
#[allow(unused_imports)]
pub use rosrust_msg;

#[cfg(feature = "ros1")]
pub use rustros_tf;

#[cfg(feature = "ros1")]
pub type Time = rosrust::Time;

#[cfg(feature = "ros1")]
pub fn init(name: &str) {
    rosrust::init(name);
}

#[cfg(feature = "ros1")]
pub fn now() -> Time {
    Time::new()
}

#[cfg(feature = "ros1")]
pub fn subscribe<M, F>(topic: &str, queue_size: usize, callback: F) -> rosrust::Subscriber
where
    M: rosrust::Message,
    F: Fn(M) + Send + 'static,
{
    rosrust::subscribe(topic, queue_size, callback).unwrap()
}

#[cfg(feature = "ros1")]
pub fn publish<M>(topic: &str, queue_size: usize) -> rosrust::Publisher<M>
where
    M: rosrust::Message,
{
    rosrust::publish(topic, queue_size).unwrap()
}

#[cfg(feature = "ros1")]
pub fn topics() -> Vec<(String, String)> {
    // rosrust exposes ROS1 master topic list as `TopicInfo { name, datatype }`.
    match rosrust::topics() {
        Ok(list) => list
            .into_iter()
            .map(|t| (t.name.to_string(), t.datatype.to_string()))
            .collect(),
        Err(_) => Vec::new(),
    }
}

#[cfg(feature = "ros1")]
pub fn param(name: &str) -> Option<rosrust::Parameter> {
    rosrust::param(name)
}
