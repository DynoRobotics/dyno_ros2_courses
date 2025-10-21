"""
Basic tests for ros2_interface_generator.
"""

import pytest
from ros2_interface_generator import Generator
from ros2_interface_generator.generator import FieldInfo, MessageInfo


def test_generator_init():
    """Test generator initialization."""
    gen = Generator(language='python', encoding='cdr')
    assert gen.language == 'python'
    assert gen.encoding == 'cdr'


def test_unsupported_language():
    """Test that unsupported languages raise NotImplementedError."""
    with pytest.raises(NotImplementedError):
        Generator(language='fortran', encoding='cdr')


def test_field_info():
    """Test FieldInfo dataclass."""
    field = FieldInfo(
        name='x',
        type='float32',
        is_builtin=True
    )
    assert field.name == 'x'
    assert field.type == 'float32'
    assert field.is_builtin is True
    assert field.is_array is False


def test_message_info():
    """Test MessageInfo dataclass."""
    msg = MessageInfo(
        name='Vector3',
        package='geometry_msgs'
    )
    assert msg.name == 'Vector3'
    assert msg.package == 'geometry_msgs'
    assert len(msg.fields) == 0
    assert len(msg.dependencies) == 0


def test_builtin_types():
    """Test that builtin types are defined."""
    gen = Generator(language='python', encoding='cdr')
    assert 'float32' in Generator.BUILTIN_TYPES
    assert 'int32' in Generator.BUILTIN_TYPES
    assert 'string' in Generator.BUILTIN_TYPES
    assert 'bool' in Generator.BUILTIN_TYPES


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

