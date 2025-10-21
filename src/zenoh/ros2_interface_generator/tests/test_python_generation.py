"""
Test Python code generation.
"""

import pytest
import tempfile
import sys
from pathlib import Path

from ros2_interface_generator.generator import MessageInfo, FieldInfo
from ros2_interface_generator.languages.python import PythonGenerator


def test_python_generator_init():
    """Test Python generator initialization."""
    gen = PythonGenerator(encoding='cdr')
    assert gen.encoding == 'cdr'


def test_simple_message_generation():
    """Test generating a simple message with builtin types."""
    gen = PythonGenerator()
    
    # Create a simple message (like geometry_msgs/Vector3)
    vector3 = MessageInfo(
        name='Vector3',
        package='geometry_msgs'
    )
    vector3.fields = [
        FieldInfo(name='x', type='float64', is_builtin=True),
        FieldInfo(name='y', type='float64', is_builtin=True),
        FieldInfo(name='z', type='float64', is_builtin=True),
    ]
    vector3.type_hash = 'RIHS01_test123'
    
    messages_by_package = {
        'geometry_msgs': {'Vector3': vector3}
    }
    
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = Path(tmpdir)
        gen.generate(messages_by_package, output_dir)
        
        # Check that files were created
        pkg_dir = output_dir / "ros2_interfaces_py"
        assert pkg_dir.exists()
        assert (pkg_dir / "geometry_msgs").exists()
        assert (pkg_dir / "geometry_msgs" / "msg").exists()
        assert (pkg_dir / "geometry_msgs" / "msg" / "vector3.py").exists()
        assert (output_dir / "setup.py").exists()


def test_nested_message_generation():
    """Test generating a message with nested types."""
    gen = PythonGenerator()
    
    # Create Vector3
    vector3 = MessageInfo(name='Vector3', package='geometry_msgs')
    vector3.fields = [
        FieldInfo(name='x', type='float64', is_builtin=True),
        FieldInfo(name='y', type='float64', is_builtin=True),
        FieldInfo(name='z', type='float64', is_builtin=True),
    ]
    vector3.type_hash = 'RIHS01_vector3'
    
    # Create Twist (uses Vector3)
    twist = MessageInfo(name='Twist', package='geometry_msgs')
    twist.fields = [
        FieldInfo(name='linear', type='Vector3', ros2_package='geometry_msgs', is_builtin=False),
        FieldInfo(name='angular', type='Vector3', ros2_package='geometry_msgs', is_builtin=False),
    ]
    twist.type_hash = 'RIHS01_twist'
    twist.dependencies.add('geometry_msgs.Vector3')
    
    messages_by_package = {
        'geometry_msgs': {
            'Vector3': vector3,
            'Twist': twist,
        }
    }
    
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = Path(tmpdir)
        gen.generate(messages_by_package, output_dir)
        
        # Check that both messages were generated
        msg_dir = output_dir / "ros2_interfaces_py" / "geometry_msgs" / "msg"
        assert (msg_dir / "vector3.py").exists()
        assert (msg_dir / "twist.py").exists()
        
        # Check that Twist imports Vector3
        twist_content = (msg_dir / "twist.py").read_text()
        assert 'from .vector3 import Vector3' in twist_content
        assert 'TYPE_HASH' in twist_content
        assert 'DDS_TYPE_NAME' in twist_content


def test_array_field_generation():
    """Test generating messages with array fields."""
    gen = PythonGenerator()
    
    # Create a message with array field
    msg = MessageInfo(name='TestArray', package='test_msgs')
    msg.fields = [
        FieldInfo(name='data', type='float64', is_builtin=True, is_array=True),
        FieldInfo(name='fixed_data', type='int32', is_builtin=True, is_array=True, is_bounded_array=True, array_size=10),
    ]
    msg.type_hash = 'RIHS01_testarray'
    
    messages_by_package = {
        'test_msgs': {'TestArray': msg}
    }
    
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = Path(tmpdir)
        gen.generate(messages_by_package, output_dir)
        
        # Check array type hints
        msg_file = output_dir / "ros2_interfaces_py" / "test_msgs" / "msg" / "testarray.py"
        content = msg_file.read_text()
        assert 'List[float64]' in content
        assert 'List[int32]' in content


def test_get_python_type():
    """Test Python type mapping."""
    gen = PythonGenerator()
    
    # Test builtin types
    field = FieldInfo(name='test', type='float64', is_builtin=True)
    assert gen._get_python_type(field, 'test_pkg') == 'float64'
    
    field = FieldInfo(name='test', type='string', is_builtin=True)
    assert gen._get_python_type(field, 'test_pkg') == 'str'
    
    # Test array
    field = FieldInfo(name='test', type='int32', is_builtin=True, is_array=True)
    assert gen._get_python_type(field, 'test_pkg') == 'List[int32]'
    
    # Test custom type from same package
    field = FieldInfo(name='test', type='Vector3', ros2_package='geometry_msgs', is_builtin=False)
    assert gen._get_python_type(field, 'geometry_msgs') == 'Vector3'
    
    # Test custom type from different package
    field = FieldInfo(name='test', type='Vector3', ros2_package='geometry_msgs', is_builtin=False)
    result = gen._get_python_type(field, 'std_msgs')
    assert 'geometry_msgs' in result


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

