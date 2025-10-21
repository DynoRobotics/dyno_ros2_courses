# ros2_zenoh_python Tests

Comprehensive test suite for ros2_zenoh_python package.

## Test Categories

### Basic Pub/Sub (`test_basic_pubsub.py`)
- Single publisher to single subscriber
- Multiple messages
- Multiple subscribers
- Basic message delivery verification

### Timing Tests (`test_timing.py`)
- Message latency measurements
- Throughput testing
- Multi-node timing characteristics
- Performance benchmarks

### Interop Tests (`test_interop.py`)
- Optional rclpy interoperability (if available)
- Message hash compatibility
- Cross-implementation communication
- ROS2 standard message compatibility

## Running Tests

### Run all tests:
```bash
cd /path/to/ros2_zenoh_python
pytest tests/ -v
```

### Run specific test file:
```bash
pytest tests/test_basic_pubsub.py -v
pytest tests/test_timing.py -v
pytest tests/test_interop.py -v
```

### Run with output:
```bash
pytest tests/ -v -s  # -s shows print statements
```

### Run only timing tests:
```bash
pytest tests/test_timing.py -v -s
```

### Skip interop tests (if rclpy not available):
```bash
pytest tests/ -v -m "not interop"
```

## Requirements

### Core tests:
- pytest
- pytest-asyncio

### Interop tests (optional):
- rclpy
- geometry_msgs (ROS2 package)

Install pytest dependencies:
```bash
pip install pytest pytest-asyncio
```

## Test Output

Tests will show:
- Message delivery success/failure
- Latency statistics (average, min, max)
- Throughput measurements
- Interop compatibility results

Example output:
```
tests/test_timing.py::TestTiming::test_message_latency 
Latency stats:
  Average: 2.34 ms
  Max: 5.12 ms
  Min: 1.23 ms
PASSED

tests/test_timing.py::TestTiming::test_throughput 
Throughput stats:
  Messages: 100
  Duration: 0.156 s
  Throughput: 641.0 msg/s
PASSED
```

## Notes

- Timing tests may vary based on system load
- Interop tests are skipped if rclpy is not available
- Tests use localhost Zenoh configuration
- Each test creates fresh node instances

