/// Example publisher using ros2_zenoh_rs

use ros2_zenoh_rs::Node;
use ros2_interfaces_rs::geometry_msgs::Twist;
use ros2_interfaces_rs::geometry_msgs::Vector3;
use std::time::Duration;

fn main() -> anyhow::Result<()> {
    println!("Starting ROS 2 Zenoh Rust Publisher");

    // Create node
    let node = Node::new("rust_publisher", "/")?;
    println!("Node created: {}", node.name());

    // Create publisher
    let publisher = node.create_raw_publisher("/turtle1/cmd_vel", "geometry_msgs/msg/Twist")?;
    println!("Publisher created for topic: {}", publisher.topic());

    // Create a Twist message
    let twist = Twist {
        linear: Vector3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        },
        angular: Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.5,
        },
    };

    println!("Publishing Twist messages...");
    for i in 0..10 {
        // Serialize to CDR
        let cdr_bytes = twist.serialize_cdr()
            .map_err(|e| anyhow::anyhow!("Failed to serialize: {}", e))?;
        
        // Publish
        publisher.publish(&cdr_bytes)?;
        println!("Published message {}: linear.x={}, angular.z={}", 
                 i, twist.linear.x, twist.angular.z);
        
        std::thread::sleep(Duration::from_secs(1));
    }

    println!("Publisher example completed");
    Ok(())
}

