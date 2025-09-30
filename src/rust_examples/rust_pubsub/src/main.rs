use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();

    let node = executor.create_node("minimal_publisher")?;

    let publisher = node.create_publisher::<rust_interfaces::msg::Test>("topic")?;

    let mut message = rust_interfaces::msg::Test::default();

    let mut publish_count: u32 = 1;

    while context.ok() {
        message.message = format!("Hello, world! {}", publish_count);
        println!("Publishing: [{}]", message.message);
        publisher.publish(&message)?;
        publish_count += 1;
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    Ok(())
}