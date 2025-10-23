# Testing Guide for ros2_zenoh_python

This guide explains how to write fast, reliable tests for ROS2 applications using simulation time and mocking.

## Table of Contents

1. [Simulation Time](#simulation-time)
2. [Mocking & Dependency Injection](#mocking--dependency-injection)
3. [Best Practices](#best-practices)
4. [Common Patterns](#common-patterns)

---

## Simulation Time

Simulation time allows you to test time-dependent behavior without waiting for real wall time to pass. This makes tests **dramatically faster** and **deterministic**.

### Basic Usage

```python
from ros2_zenoh_python.time import Clock, Time

# Create a simulation clock
clock = Clock(use_sim_time=True)
clock.set_time(Time.from_sec(0.0))

# Sleep in simulation time
await clock.sleep(3600.0)  # "Sleep" for 1 hour

# But tests run instantly because we control time!
clock.set_time(Time.from_sec(3600.0))
await clock.tick()  # Wake up any sleepers
```

### Timers

Create timers that respect simulation time:

```python
fire_count = 0

async def on_timer():
    global fire_count
    fire_count += 1
    print(f"Timer fired! Count: {fire_count}")

# Create a 1-second periodic timer
timer = clock.create_timer(1.0, on_timer)
timer.start()

# Advance time and check results
for i in range(1, 11):
    clock.set_time(Time.from_sec(i))
    await clock.tick()
    await asyncio.sleep(0.001)  # Tiny delay for timer to fire

# Timer fired 10 times in < 1 second of wall time!
assert fire_count >= 9
```

### Testing Async Code with Time

```python
async def long_running_task(clock):
    """Task that takes 'hours' in simulation time."""
    results = []
    for hour in range(24):
        await clock.sleep(3600.0)  # Sleep 1 hour
        results.append(f"Hour {hour} complete")
    return results

# Run the test
clock = Clock(use_sim_time=True)
clock.set_time(Time.from_sec(0.0))

task = asyncio.create_task(long_running_task(clock))

# Simulate 24 hours instantly
for hour in range(24):
    clock.set_time(Time.from_sec((hour + 1) * 3600))
    await clock.tick()

results = await asyncio.wait_for(task, timeout=1.0)
assert len(results) == 24
```

---

## Mocking & Dependency Injection

Use `MockNode` to test classes without requiring actual network communication.

### Dependency Injection Pattern

**Write your classes to accept a node as a parameter:**

```python
class MyRobotController:
    """Good: accepts node via dependency injection."""
    
    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel')
        self.odom_sub = node.create_subscription(
            Odometry, '/odom', self.on_odometry
        )
        self.current_position = None
    
    def on_odometry(self, msg):
        self.current_position = msg.pose.pose.position
        
    def move_forward(self, speed: float):
        cmd = Twist()
        cmd.linear.x = speed
        self.cmd_pub.publish(cmd)
```

### Testing with MockNode

```python
import pytest
from ros2_zenoh_python.testing import MockNode

@pytest.mark.asyncio
async def test_robot_controller():
    # Create mock node
    mock_node = MockNode('test_node')
    
    # Create controller with mock
    controller = MyRobotController(mock_node)
    
    # Inject odometry data
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = 5.0
    await mock_node.inject_to_subscription('/odom', odom_msg)
    
    # Verify odometry was processed
    assert controller.current_position.x == 5.0
    
    # Test movement command
    controller.move_forward(1.5)
    
    # Verify command was published
    commands = mock_node.get_published_messages('/cmd_vel')
    assert len(commands) == 1
    assert commands[0].linear.x == 1.5
```

### Mocking Services

```python
class MyServiceUser:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(AddTwoInts, '/add')
    
    async def compute(self, a: int, b: int) -> int:
        request = AddTwoInts.Request(a=a, b=b)
        response = await self.client.call_async(request)
        return response.sum

# Test
@pytest.mark.asyncio
async def test_service_user():
    mock_node = MockNode('test')
    
    # Set canned response
    mock_response = AddTwoInts.Response(sum=42)
    mock_node.set_client_response('/add', mock_response)
    
    # Test
    user = MyServiceUser(mock_node)
    result = await user.compute(1, 2)
    
    # Verify result (from mock)
    assert result == 42
    
    # Verify request was made
    requests = mock_node.get_client_requests('/add')
    assert len(requests) == 1
    assert requests[0].a == 1
    assert requests[0].b == 2
```

---

## Best Practices

### 1. **Always Use Dependency Injection**

❌ **Bad:**
```python
class MyNode:
    def __init__(self):
        # Creates real node - can't be tested in isolation
        self.node = Node('my_node')
```

✅ **Good:**
```python
class MyNode:
    def __init__(self, node):
        # Accepts node - can inject mock for testing
        self.node = node
```

### 2. **Test Edge Cases Explicitly, Not with Delays**

❌ **Bad:**
```python
# Start background task
task = asyncio.create_task(my_function())
await asyncio.sleep(2.0)  # Hope it finishes?
assert task.done()
```

✅ **Good:**
```python
# Use synchronization primitives
ready = threading.Event()

async def my_function():
    ready.set()  # Signal we started
    # ... do work ...

task = asyncio.create_task(my_function())
await asyncio.get_event_loop().run_in_executor(None, ready.wait, 1.0)
assert ready.is_set()
```

### 3. **Use Simulation Time for Time-Dependent Tests**

❌ **Bad:**
```python
async def test_timeout():
    start = time.time()
    await some_operation(timeout=5.0)
    duration = time.time() - start
    assert duration >= 5.0  # Test takes 5+ seconds!
```

✅ **Good:**
```python
async def test_timeout():
    clock = Clock(use_sim_time=True)
    clock.set_time(Time.from_sec(0.0))
    
    # Test runs instantly
    clock.set_time(Time.from_sec(10.0))
    await clock.tick()
```

### 4. **Clear Mock State Between Tests**

```python
@pytest.fixture
def mock_node():
    node = MockNode('test')
    yield node
    node.clear_all()  # Clean up
```

### 5. **Verify Behavior, Not Implementation**

❌ **Bad:**
```python
# Testing implementation details
assert len(controller._internal_buffer) == 3
```

✅ **Good:**
```python
# Testing observable behavior
commands = mock_node.get_published_messages('/cmd_vel')
assert len(commands) == 3
assert commands[0].linear.x == 1.0
```

---

## Common Patterns

### Pattern 1: Testing Periodic Publishers

```python
class PeriodicPublisher:
    def __init__(self, node):
        self.node = node
        self.pub = node.create_publisher(String, '/periodic')
        clock = node.get_clock()
        self.timer = clock.create_timer(1.0, self.on_timer)
        self.timer.start()
        self.count = 0
    
    def on_timer(self):
        msg = String(data=f"Message {self.count}")
        self.pub.publish(msg)
        self.count += 1

# Test
@pytest.mark.asyncio
async def test_periodic_publisher():
    clock = Clock(use_sim_time=True)
    clock.set_time(Time.from_sec(0.0))
    
    mock_node = MockNode('test')
    mock_node.set_clock(clock)
    
    publisher = PeriodicPublisher(mock_node)
    
    # Advance time by 5 seconds
    for i in range(1, 6):
        clock.set_time(Time.from_sec(i))
        await clock.tick()
        await asyncio.sleep(0.01)
    
    # Verify messages were published
    messages = mock_node.get_published_messages('/periodic')
    assert len(messages) >= 4  # Should fire ~5 times
```

### Pattern 2: Testing State Machines

```python
class StateMachine:
    def __init__(self, node):
        self.node = node
        self.state = 'IDLE'
        self.sub = node.create_subscription(String, '/trigger', self.on_trigger)
        self.pub = node.create_publisher(String, '/state')
    
    def on_trigger(self, msg):
        if msg.data == 'start' and self.state == 'IDLE':
            self.state = 'RUNNING'
            self.pub.publish(String(data=self.state))
        elif msg.data == 'stop' and self.state == 'RUNNING':
            self.state = 'IDLE'
            self.pub.publish(String(data=self.state))

# Test
@pytest.mark.asyncio
async def test_state_machine():
    mock_node = MockNode('test')
    sm = StateMachine(mock_node)
    
    # Trigger state transition
    await mock_node.inject_to_subscription('/trigger', String(data='start'))
    
    # Verify state changed
    assert sm.state == 'RUNNING'
    states = mock_node.get_published_messages('/state')
    assert len(states) == 1
    assert states[0].data == 'RUNNING'
    
    # Stop
    await mock_node.inject_to_subscription('/trigger', String(data='stop'))
    assert sm.state == 'IDLE'
```

### Pattern 3: Testing Multi-Node Scenarios

```python
from ros2_zenoh_python.testing import TestScenario

@pytest.mark.asyncio
async def test_multi_node_communication():
    scenario = TestScenario(use_sim_time=True)
    
    # Create nodes
    sender_node = scenario.create_mock_node('sender')
    receiver_node = scenario.create_mock_node('receiver')
    
    # Set up communication
    sender_pub = sender_node.create_publisher(String, '/topic')
    
    received = []
    receiver_sub = receiver_node.create_subscription(
        String, '/topic', lambda msg: received.append(msg.data)
    )
    
    # Simulate message from sender
    msg = String(data="cross-node")
    sender_pub.publish(msg)
    
    # Inject to receiver (simulates Zenoh routing)
    await receiver_node.inject_to_subscription('/topic', msg)
    
    # Verify
    assert len(received) == 1
    assert received[0] == "cross-node"
```

---

## Performance Comparison

### Without Simulation Time
```python
# Test that waits for real time
async def test_slow():
    await asyncio.sleep(5.0)  # Wait 5 seconds
    # ... assertions ...

# Runtime: 5+ seconds per test
```

### With Simulation Time
```python
# Same test with simulation time
async def test_fast():
    clock = Clock(use_sim_time=True)
    clock.set_time(Time.from_sec(0.0))
    clock.set_time(Time.from_sec(5.0))
    await clock.tick()
    # ... assertions ...

# Runtime: < 0.01 seconds per test (500x faster!)
```

---

## Summary

1. **Use `MockNode`** for dependency injection and isolated testing
2. **Use `Clock(use_sim_time=True)`** for time-dependent tests
3. **Test behavior, not implementation**
4. **Use synchronization primitives** instead of blind sleeps
5. **Clear mock state** between tests

These patterns enable:
- ✅ **Fast tests** (no waiting for real time)
- ✅ **Deterministic tests** (no race conditions)
- ✅ **Isolated tests** (no network dependencies)
- ✅ **Maintainable tests** (clear intent)

See `tests/test_simulation_time.py` and `tests/test_mocking.py` for complete examples.

