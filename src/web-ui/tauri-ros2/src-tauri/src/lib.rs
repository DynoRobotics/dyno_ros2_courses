mod ros2;

// Learn more about Tauri commands at https://tauri.app/develop/calling-rust/
#[tauri::command]
fn greet(name: &str) -> String {
    // Publish a ROS2 message when greet is invoked
    if let Err(e) = ros2::publish_message(&format!("Hello, {}!", name)) {
        eprintln!("[greet] Failed to publish ROS2 message: {}", e);
    }
    
    format!("Hello, {}! You've been greeted from Rust!", name)
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .setup(|app| {
                // Initialize the ROS 2 publisher once at startup
                if let Err(e) = ros2::init_ros2_publisher() {
                    eprintln!("[setup] Failed to initialize ROS2 publisher: {}", e);
                }
                
                // Start the ROS 2 subscriber in a background thread
                ros2::start_ros2_subscriber(app.handle().clone());
                
                Ok(())
        })
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![greet])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
