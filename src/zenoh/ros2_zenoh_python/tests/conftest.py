"""
Pytest configuration for ros2_zenoh_python tests with shared Zenoh session
"""

import pytest
import asyncio
import zenoh
import sys
from pathlib import Path

# Note: Tests use bundled messages from ros2_zenoh_python._bundled_msgs
# No need to add external paths


@pytest.fixture(scope="session")
def event_loop():
    """Create an event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    
    # Cancel any remaining tasks
    pending = asyncio.all_tasks(loop)
    for task in pending:
        task.cancel()
    
    # Give tasks a chance to clean up
    if pending:
        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
    
    loop.close()


@pytest.fixture(scope="session")
def zenoh_session():
    """
    Create a shared Zenoh session for all tests.
    
    This significantly speeds up tests by reusing the same session.
    """
    import time
    
    # Create session in peer mode for fast local testing
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    session = zenoh.open(config)
    
    yield session
    
    # Aggressive cleanup
    try:
        session.close()
        # Give Zenoh threads time to shut down
        time.sleep(0.2)
    except Exception as e:
        print(f"Warning: Error closing zenoh_session: {e}")


@pytest.fixture(scope="session")
def zenoh_session_client():
    """
    Create a shared Zenoh session in client mode for interop tests.
    
    Connects to the ROS2 Zenoh router at localhost:7447.
    """
    import time
    
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    session = zenoh.open(config)
    
    yield session
    
    # Aggressive cleanup
    try:
        session.close()
        # Give Zenoh threads time to shut down
        time.sleep(0.2)
    except Exception as e:
        print(f"Warning: Error closing zenoh_session_client: {e}")


@pytest.fixture(scope="session")
def rclpy_session():
    """
    Initialize rclpy once per test session.
    
    rclpy can only be initialized once per process, so this fixture
    ensures all interop tests share the same rclpy context.
    """
    try:
        import rclpy
        import threading
        
        rclpy.init()
        yield None
        
        # Aggressive cleanup
        # First, try to shutdown any remaining executors
        try:
            # Get all threads and try to identify rclpy executor threads
            for thread in threading.enumerate():
                if 'rclpy' in thread.name.lower() or 'executor' in thread.name.lower():
                    # These should clean up when we shutdown rclpy
                    pass
            
            # Shutdown rclpy context
            if rclpy.ok():
                rclpy.shutdown()
        except RuntimeError:
            # Context already shutdown, that's fine
            pass
        except Exception as e:
            print(f"Warning: Error during rclpy cleanup: {e}")
            
    except ImportError:
        # rclpy not available, skip
        yield None


def pytest_sessionfinish(session, exitstatus):
    """Clean up after all tests are done."""
    import threading
    import time
    import sys
    import os
    
    # Give threads a moment to clean up
    time.sleep(0.5)
    
    # Log remaining threads
    remaining_threads = [t for t in threading.enumerate() if t.daemon is False and t != threading.current_thread()]
    if remaining_threads:
        print(f"\n⚠️  Warning: {len(remaining_threads)} non-daemon threads still running:")
        for thread in remaining_threads:
            print(f"   - {thread.name} (daemon={thread.daemon}, alive={thread.is_alive()})")
        
        # If there are Zenoh pyo3-closure threads, they're internal threads that won't clean up
        # Force exit to prevent hanging
        zenoh_threads = [t for t in remaining_threads if 'pyo3' in t.name or 'zenoh' in t.name.lower()]
        if zenoh_threads:
            print(f"   Note: {len(zenoh_threads)} Zenoh internal threads detected - will force exit")
            time.sleep(0.1)
            os._exit(exitstatus)  # Force exit without waiting for threads


def pytest_configure(config):
    """Configure pytest with custom markers."""
    # Register custom markers to avoid warnings
    config.addinivalue_line(
        "markers", 
        "interop: mark test as requiring rclpy interoperability (may fail without rclpy)"
    )
    
    # Set environment variable to skip signal handlers in tests
    import os
    os.environ['PYTEST_CURRENT_TEST'] = 'true'
