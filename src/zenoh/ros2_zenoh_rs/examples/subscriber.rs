/// Example subscriber using ros2_zenoh_rs

use ros2_zenoh_rs::Node;
use ros2_interfaces_rs::geometry_msgs::Twist;
use std::sync::Arc;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("Starting ROS 2 Zenoh Rust Subscriber");

    // Create node
    let node = Arc::new(Node::new("rust_subscriber", "/")?);
    println!("Node created: {}", node.name());

    // Create subscriber with callback
    let _subscriber = node.create_raw_subscriber(
        "/turtle1/cmd_vel",
        "geometry_msgs/msg/Twist",
        |data: &[u8]| {
            // Deserialize CDR data
            match Twist::deserialize_cdr(data) {
                Ok(twist) => {
                    println!(
                        "Received Twist: linear.x={:.2}, angular.z={:.2}",
                        twist.linear.x, twist.angular.z
                    );
                }
                Err(e) => {
                    eprintln!("Failed to deserialize message: {}", e);
                }
            }
        },
    )?;

    println!("Subscriber waiting for messages on /turtle1/cmd_vel...");
    println!("Press Ctrl+C to exit");

    // Keep running
    loop {
        std::thread::sleep(Duration::from_secs(1));
    }
}

