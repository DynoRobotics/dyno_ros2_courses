use anyhow::{Error, Result};
use rclrs::*;
use std::sync::{Arc, Mutex};
use std::thread;
use tauri::Emitter;

// Global state to hold the publisher
// We use Arc<Mutex<>> to allow safe shared access across threads
static PUBLISHER: once_cell::sync::Lazy<Arc<Mutex<Option<PublisherState>>>> =
    once_cell::sync::Lazy::new(|| Arc::new(Mutex::new(None)));

struct PublisherState {
    publisher: rclrs::Publisher<rust_interfaces::msg::Test>,
    publish_count: u32,
}

pub fn init_ros2_publisher() -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let executor = context.create_basic_executor();
    let node = executor.create_node("minimal_publisher")?;
    let publisher = node.create_publisher::<rust_interfaces::msg::Test>("topic")?;
    
    let state = PublisherState {
        publisher,
        publish_count: 0,
    };
    
    *PUBLISHER.lock().unwrap() = Some(state);
    println!("[ros2] Publisher initialized");
    Ok(())
}

pub fn publish_message(message_content: &str) -> Result<(), Error> {
    let mut guard = PUBLISHER.lock().unwrap();
    
    if let Some(state) = guard.as_mut() {
        state.publish_count += 1;
        let mut message = rust_interfaces::msg::Test::default();
        message.message = format!("{} (count: {})", message_content, state.publish_count);
        
        println!("Publishing: [{}]", message.message);
        state.publisher.publish(&message)?;
        Ok(())
    } else {
        Err(Error::msg("ROS2 publisher not initialized"))
    }
}

pub fn start_ros2_subscriber(app_handle: tauri::AppHandle) {
    thread::spawn(move || {
        if let Err(e) = run_subscriber(app_handle) {
            eprintln!("[ros2 subscriber] Error: {e:?}");
        }
    });
}

fn run_subscriber(app_handle: tauri::AppHandle) -> Result<(), Error> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("tauri_subscriber")?;

    let worker = node.create_worker::<usize>(0);
    let _subscription = worker.create_subscription::<std_msgs::msg::String, _>(
        "chatter",
        move |num_messages: &mut usize, msg: std_msgs::msg::String| {
            *num_messages += 1;
            println!("#{} | Received: '{}'", *num_messages, msg.data);
            
            // Emit event to frontend
            if let Err(e) = app_handle.emit("ros2-message", msg.data.clone()) {
                eprintln!("[ros2 subscriber] Failed to emit event: {}", e);
            }
        },
    )?;

    println!("[ros2] Subscriber initialized, waiting for messages on 'chatter' topic...");
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}
