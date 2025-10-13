use anyhow::{Error, Result};
use rclrs::*;

fn main() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();

    let node = executor.create_node("minimal_subscriber")?;

    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_subscription::<geometry_msgs::msg::Twist, _>(
        "/turtle1/safe_cmd_vel",
        move |num_messages: &mut usize, msg: geometry_msgs::msg::Twist| {
            *num_messages += 1;
            println!(
                "#{} | Twist linear=({:.3}, {:.3}, {:.3}) angular=({:.3}, {:.3}, {:.3})",
                *num_messages,
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
            );
        },
    )?;

    println!("Waiting for messages...");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}