"""
Pytest configuration for ros2_zenoh_python tests with shared Zenoh session
"""

import pytest
import asyncio
import zenoh
import sys
from pathlib import Path

# Add unified_output to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "tools" / "unified_output" / "python"))


@pytest.fixture(scope="session")
def event_loop():
    """Create an event loop for async tests."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


@pytest.fixture(scope="session")
def zenoh_session():
    """
    Create a shared Zenoh session for all tests.
    
    This significantly speeds up tests by reusing the same session.
    """
    # Create session in peer mode for fast local testing
    config = zenoh.Config()
    config.insert_json5("mode", '"peer"')
    session = zenoh.open(config)
    
    yield session
    
    session.close()


@pytest.fixture(scope="session")
def zenoh_session_client():
    """
    Create a shared Zenoh session in client mode for interop tests.
    
    Connects to the ROS2 Zenoh router at localhost:7447.
    """
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", '["tcp/localhost:7447"]')
    session = zenoh.open(config)
    
    yield session
    
    session.close()


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
