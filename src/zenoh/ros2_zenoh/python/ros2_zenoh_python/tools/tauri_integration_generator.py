#!/usr/bin/env python3
"""
Tauri ROS 2 Integration Generator

This script generates complete Tauri integration files for ROS 2 message types,
including Rust backend types, TypeScript frontend types, and conversion functions.
"""

import os
import sys
import json
from pathlib import Path
from multi_lang_generator import MultiLangGenerator, Language, MessageInfo


class TauriIntegrationGenerator:
    """Generates complete Tauri integration for ROS 2 messages."""
    
    def __init__(self):
        self.generator = MultiLangGenerator()
    
    def generate_rust_backend(self, messages: dict) -> str:
        """Generate complete Rust backend with Tauri commands."""
        lines = []
        lines.append("use serde::{Deserialize, Serialize};")
        lines.append("use tauri::command;")
        lines.append("use std::sync::{Arc, Mutex};")
        lines.append("use anyhow::{Error, Result};")
        lines.append("")
        lines.append("// Generated ROS 2 message types")
        lines.append("")
        
        # Generate all message types
        for message_name, message in messages.items():
            lines.extend(self.generator.generate_rust(message).split('\n'))
            lines.append("")
        
        # Generate Tauri commands
        lines.extend([
            "// Tauri command functions",
            "",
            "#[command]",
            "pub async fn publish_twist(msg: Twist) -> Result<String, String> {",
            "    // TODO: Implement ROS 2 publishing",
            "    Ok(format!(\"Published Twist: linear=({}, {}, {}), angular=({}, {}, {})\",",
            "        msg.linear.x, msg.linear.y, msg.linear.z,",
            "        msg.angular.x, msg.angular.y, msg.angular.z))",
            "}",
            "",
            "#[command]",
            "pub async fn subscribe_twist() -> Result<Twist, String> {",
            "    // TODO: Implement ROS 2 subscription",
            "    Err(\"Not implemented\".to_string())",
            "}",
            "",
            "#[command]",
            "pub async fn publish_vector3(msg: Vector3) -> Result<String, String> {",
            "    // TODO: Implement ROS 2 publishing",
            "    Ok(format!(\"Published Vector3: ({}, {}, {})\", msg.x, msg.y, msg.z))",
            "}",
            "",
            "#[command]",
            "pub async fn subscribe_vector3() -> Result<Vector3, String> {",
            "    // TODO: Implement ROS 2 subscription",
            "    Err(\"Not implemented\".to_string())",
            "}",
            "",
        ])
        
        return '\n'.join(lines)
    
    def generate_typescript_frontend(self, messages: dict) -> str:
        """Generate complete TypeScript frontend with React hooks."""
        lines = []
        lines.append("// Generated ROS 2 message types for Tauri frontend")
        lines.append("import { invoke } from '@tauri-apps/api/core';")
        lines.append("import { listen } from '@tauri-apps/api/event';")
        lines.append("import { useState, useEffect, useCallback } from 'react';")
        lines.append("")
        
        # Generate all message interfaces
        for message_name, message in messages.items():
            lines.extend(self.generator.generate_typescript(message).split('\n'))
            lines.append("")
        
        # Generate Tauri API functions
        lines.extend([
            "// Tauri API functions",
            "",
            "export async function publishTwist(msg: Twist): Promise<string> {",
            "  return await invoke('publish_twist', { msg });",
            "}",
            "",
            "export async function subscribeTwist(): Promise<Twist> {",
            "  return await invoke('subscribe_twist');",
            "}",
            "",
            "export async function publishVector3(msg: Vector3): Promise<string> {",
            "  return await invoke('publish_vector3', { msg });",
            "}",
            "",
            "export async function subscribeVector3(): Promise<Vector3> {",
            "  return await invoke('subscribe_vector3');",
            "}",
            "",
        ])
        
        # Generate React hooks
        lines.extend([
            "// React hooks for ROS 2 messages",
            "",
            "export function useTwist() {",
            "  const [messages, setMessages] = useState<Twist[]>([]);",
            "  const [isConnected, setIsConnected] = useState(false);",
            "",
            "  const publish = useCallback(async (msg: Twist) => {",
            "    try {",
            "      const result = await publishTwist(msg);",
            "      console.log('Published:', result);",
            "    } catch (error) {",
            "      console.error('Failed to publish Twist:', error);",
            "    }",
            "  }, []);",
            "",
            "  useEffect(() => {",
            "    const unlisten = listen<Twist>('ros2-twist', (event) => {",
            "      setMessages(prev => [...prev.slice(-9), event.payload]);",
            "    });",
            "",
            "    return () => {",
            "      unlisten.then(fn => fn());",
            "    };",
            "  }, []);",
            "",
            "  return { messages, publish, isConnected };",
            "}",
            "",
            "export function useVector3() {",
            "  const [messages, setMessages] = useState<Vector3[]>([]);",
            "  const [isConnected, setIsConnected] = useState(false);",
            "",
            "  const publish = useCallback(async (msg: Vector3) => {",
            "    try {",
            "      const result = await publishVector3(msg);",
            "      console.log('Published:', result);",
            "    } catch (error) {",
            "      console.error('Failed to publish Vector3:', error);",
            "    }",
            "  }, []);",
            "",
            "  useEffect(() => {",
            "    const unlisten = listen<Vector3>('ros2-vector3', (event) => {",
            "      setMessages(prev => [...prev.slice(-9), event.payload]);",
            "    });",
            "",
            "    return () => {",
            "      unlisten.then(fn => fn());",
            "    };",
            "  }, []);",
            "",
            "  return { messages, publish, isConnected };",
            "}",
            "",
        ])
        
        return '\n'.join(lines)
    
    def generate_react_component(self, messages: dict) -> str:
        """Generate a complete React component using the generated hooks."""
        lines = []
        lines.append("import React, { useState } from 'react';")
        lines.append("import { useTwist, useVector3 } from './ros2Types';")
        lines.append("import { Twist, Vector3 } from './ros2Types';")
        lines.append("")
        lines.append("export function Ros2ControlPanel() {")
        lines.append("  const { messages: twistMessages, publish: publishTwist } = useTwist();")
        lines.append("  const { messages: vector3Messages, publish: publishVector3 } = useVector3();")
        lines.append("")
        lines.append("  const [twistInput, setTwistInput] = useState<Twist>({")
        lines.append("    linear: { x: 0, y: 0, z: 0 },")
        lines.append("    angular: { x: 0, y: 0, z: 0 }")
        lines.append("  });")
        lines.append("")
        lines.append("  const [vector3Input, setVector3Input] = useState<Vector3>({")
        lines.append("    x: 0, y: 0, z: 0")
        lines.append("  });")
        lines.append("")
        lines.append("  const handleTwistPublish = () => {")
        lines.append("    publishTwist(twistInput);")
        lines.append("  };")
        lines.append("")
        lines.append("  const handleVector3Publish = () => {")
        lines.append("    publishVector3(vector3Input);")
        lines.append("  };")
        lines.append("")
        lines.append("  return (")
        lines.append("    <div style={{ padding: '20px', fontFamily: 'Arial, sans-serif' }}>")
        lines.append("      <h1>ROS 2 Control Panel</h1>")
        lines.append("")
        lines.append("      <div style={{ marginBottom: '30px' }}>")
        lines.append("        <h2>Twist Publisher</h2>")
        lines.append("        <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>")
        lines.append("          <div>")
        lines.append("            <label>Linear X: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.linear.x}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                linear: { ...twistInput.linear, x: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Linear Y: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.linear.y}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                linear: { ...twistInput.linear, y: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Linear Z: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.linear.z}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                linear: { ...twistInput.linear, z: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("        </div>")
        lines.append("        <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>")
        lines.append("          <div>")
        lines.append("            <label>Angular X: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.angular.x}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                angular: { ...twistInput.angular, x: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Angular Y: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.angular.y}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                angular: { ...twistInput.angular, y: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Angular Z: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={twistInput.angular.z}")
        lines.append("              onChange={(e) => setTwistInput({")
        lines.append("                ...twistInput,")
        lines.append("                angular: { ...twistInput.angular, z: parseFloat(e.target.value) }")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("        </div>")
        lines.append("        <button onClick={handleTwistPublish}>Publish Twist</button>")
        lines.append("      </div>")
        lines.append("")
        lines.append("      <div style={{ marginBottom: '30px' }}>")
        lines.append("        <h2>Vector3 Publisher</h2>")
        lines.append("        <div style={{ display: 'flex', gap: '10px', marginBottom: '10px' }}>")
        lines.append("          <div>")
        lines.append("            <label>X: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={vector3Input.x}")
        lines.append("              onChange={(e) => setVector3Input({")
        lines.append("                ...vector3Input,")
        lines.append("                x: parseFloat(e.target.value)")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Y: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={vector3Input.y}")
        lines.append("              onChange={(e) => setVector3Input({")
        lines.append("                ...vector3Input,")
        lines.append("                y: parseFloat(e.target.value)")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <label>Z: </label>")
        lines.append("            <input")
        lines.append("              type=\"number\"")
        lines.append("              value={vector3Input.z}")
        lines.append("              onChange={(e) => setVector3Input({")
        lines.append("                ...vector3Input,")
        lines.append("                z: parseFloat(e.target.value)")
        lines.append("              })}")
        lines.append("            />")
        lines.append("          </div>")
        lines.append("        </div>")
        lines.append("        <button onClick={handleVector3Publish}>Publish Vector3</button>")
        lines.append("      </div>")
        lines.append("")
        lines.append("      <div style={{ marginBottom: '30px' }}>")
        lines.append("        <h2>Received Messages</h2>")
        lines.append("        <div style={{ display: 'flex', gap: '20px' }}>")
        lines.append("          <div>")
        lines.append("            <h3>Twist Messages</h3>")
        lines.append("            <ul style={{ maxHeight: '200px', overflowY: 'auto' }}>")
        lines.append("              {twistMessages.map((msg, idx) => (")
        lines.append("                <li key={idx}>")
        lines.append("                  Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}) | ")
        lines.append("                  Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")
        lines.append("                </li>")
        lines.append("              ))}")
        lines.append("            </ul>")
        lines.append("          </div>")
        lines.append("          <div>")
        lines.append("            <h3>Vector3 Messages</h3>")
        lines.append("            <ul style={{ maxHeight: '200px', overflowY: 'auto' }}>")
        lines.append("              {vector3Messages.map((msg, idx) => (")
        lines.append("                <li key={idx}>({msg.x}, {msg.y}, {msg.z})</li>")
        lines.append("              ))}")
        lines.append("            </ul>")
        lines.append("          </div>")
        lines.append("        </div>")
        lines.append("      </div>")
        lines.append("    </div>")
        lines.append("  );")
        lines.append("}")
        
        return '\n'.join(lines)
    
    def generate_tauri_config(self, messages: dict) -> str:
        """Generate Tauri configuration for ROS 2 commands."""
        commands = []
        for message_name in messages.keys():
            commands.extend([
                f"publish_{message_name.lower()}",
                f"subscribe_{message_name.lower()}"
            ])
        
        config = {
            "commands": commands,
            "permissions": {
                "allow": ["core:default"]
            }
        }
        
        return json.dumps(config, indent=2)


def main():
    if len(sys.argv) != 3:
        print("Usage: python3 tauri_integration_generator.py <input_msg_file_or_dir> <output_dir>")
        sys.exit(1)
    
    input_path = sys.argv[1]
    output_dir = sys.argv[2]
    
    generator = TauriIntegrationGenerator()
    
    # Parse the message files
    messages = {}
    if os.path.isfile(input_path):
        # Single file
        message = generator.generator.parse_msg_file(input_path)
        messages[message.name] = message
    else:
        # Directory
        for root, dirs, files in os.walk(input_path):
            for file in files:
                if file.endswith('.msg'):
                    file_path = os.path.join(root, file)
                    message = generator.generator.parse_msg_file(file_path)
                    messages[message.name] = message
    
    # Create output directory
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate Rust backend
    rust_content = generator.generate_rust_backend(messages)
    rust_path = os.path.join(output_dir, "ros2_backend.rs")
    with open(rust_path, 'w') as f:
        f.write(rust_content)
    print(f"✅ Generated Rust backend: {rust_path}")
    
    # Generate TypeScript frontend
    ts_content = generator.generate_typescript_frontend(messages)
    ts_path = os.path.join(output_dir, "ros2Types.ts")
    with open(ts_path, 'w') as f:
        f.write(ts_content)
    print(f"✅ Generated TypeScript frontend: {ts_path}")
    
    # Generate React component
    react_content = generator.generate_react_component(messages)
    react_path = os.path.join(output_dir, "Ros2ControlPanel.tsx")
    with open(react_path, 'w') as f:
        f.write(react_content)
    print(f"✅ Generated React component: {react_path}")
    
    # Generate Tauri config
    config_content = generator.generate_tauri_config(messages)
    config_path = os.path.join(output_dir, "tauri_config.json")
    with open(config_path, 'w') as f:
        f.write(config_content)
    print(f"✅ Generated Tauri config: {config_path}")
    
    print(f"\n🎉 Complete Tauri integration generated successfully!")
    print(f"📁 Output directory: {output_dir}")
    print(f"\n📋 Next steps:")
    print(f"1. Copy the Rust backend code to your Tauri src/ directory")
    print(f"2. Copy the TypeScript types to your frontend src/ directory")
    print(f"3. Use the React component in your App.tsx")
    print(f"4. Update your tauri.conf.json with the generated commands")


if __name__ == '__main__':
    main()