#!/bin/bash

echo "🧪 Testing Zenoh C Examples End-to-End Communication"
echo "=================================================="

# Kill any existing processes
pkill -f subscriber_example_zenoh 2>/dev/null
pkill -f publisher_example_zenoh 2>/dev/null

echo "📥 Starting subscriber in background..."
cd /home/ubuntu/ws/src/ros2_zenoh_c
./bin/subscriber_example_zenoh turtle1/cmd_vel > subscriber_output.log 2>&1 &
SUBSCRIBER_PID=$!

echo "⏳ Waiting for subscriber to initialize..."
sleep 2

echo "📤 Running publisher..."
./bin/publisher_example_zenoh turtle1/cmd_vel "Test message from C!"

echo "⏳ Waiting for messages to be processed..."
sleep 3

echo "🛑 Stopping subscriber..."
kill $SUBSCRIBER_PID 2>/dev/null
wait $SUBSCRIBER_PID 2>/dev/null

echo "📋 Subscriber output:"
echo "===================="
cat subscriber_output.log

echo ""
echo "✅ Test completed!"
