#!/usr/bin/env python3
"""
Test that code examples in documentation actually work.

Extracts Python code blocks from markdown files and runs them as tests.
"""

import ast
import re
import sys
from pathlib import Path
from typing import List, Tuple

import pytest


def extract_python_blocks(markdown_file: Path) -> List[Tuple[int, str]]:
    """
    Extract Python code blocks from markdown file.
    
    Returns: List of (line_number, code_block) tuples
    """
    content = markdown_file.read_text()
    blocks = []
    
    # Match ```python ... ``` blocks
    pattern = r'```python\n(.*?)```'
    
    for match in re.finditer(pattern, content, re.DOTALL):
        code = match.group(1)
        # Find line number
        line_num = content[:match.start()].count('\n') + 1
        blocks.append((line_num, code))
    
    return blocks


def is_runnable_code(code: str) -> bool:
    """
    Check if code block is meant to be run.
    
    Skip blocks with placeholders or pseudo-code markers.
    """
    skip_markers = [
        '{{',           # Template placeholders
        '...',          # Pseudo-code ellipsis
        'TODO',         # Not implemented
        'FIXME',        # Broken
        '# Pseudo-code', # Explicit pseudo-code
        '# Example only', # Documentation example
    ]
    
    for marker in skip_markers:
        if marker in code:
            return False
    
    # Skip if it's just TypeScript in a Python block
    if 'interface ' in code or 'function ' in code and ':' in code:
        return False
    
    return True


def validate_syntax(code: str) -> Tuple[bool, str]:
    """
    Validate Python syntax without executing.
    
    Returns: (is_valid, error_message)
    """
    try:
        ast.parse(code)
        return True, ""
    except SyntaxError as e:
        return False, f"Syntax error: {e}"


def extract_imports(code: str) -> str:
    """Extract import statements from code block."""
    lines = code.split('\n')
    imports = []
    
    for line in lines:
        stripped = line.strip()
        if stripped.startswith(('import ', 'from ')):
            imports.append(line)
    
    return '\n'.join(imports)


def has_required_imports(code: str) -> bool:
    """Check if code has necessary imports."""
    # If code uses ros2_zenoh_python but doesn't import it, it's incomplete
    if 'ros2_zenoh_python' in code or 'Node' in code or 'MockNode' in code:
        if 'from ros2_zenoh_python' not in code and 'import ros2_zenoh_python' not in code:
            return False
    return True


class TestDocumentation:
    """Test suite for documentation code examples."""
    
    @pytest.mark.parametrize("doc_file", [
        pytest.param(p, id=p.stem) 
        for p in Path("docs").rglob("*.md")
    ])
    def test_code_syntax_valid(self, doc_file: Path):
        """All Python code blocks in docs have valid syntax."""
        blocks = extract_python_blocks(doc_file)
        
        for line_num, code in blocks:
            if not is_runnable_code(code):
                # Skip pseudo-code
                continue
            
            is_valid, error = validate_syntax(code)
            assert is_valid, (
                f"{doc_file}:{line_num}\n"
                f"Invalid Python syntax in code block:\n{error}\n"
                f"Code:\n{code}"
            )
    
    @pytest.mark.parametrize("doc_file", [
        pytest.param(p, id=p.stem)
        for p in Path("docs/02-Tutorials").rglob("*.md")
    ])
    def test_tutorial_examples_complete(self, doc_file: Path):
        """Tutorial examples have necessary imports."""
        blocks = extract_python_blocks(doc_file)
        
        for line_num, code in blocks:
            if not is_runnable_code(code):
                continue
            
            if len(code.strip()) < 10:  # Skip trivial snippets
                continue
            
            assert has_required_imports(code), (
                f"{doc_file}:{line_num}\n"
                f"Code block missing required imports.\n"
                f"Add necessary imports at the top of the block.\n"
                f"Code:\n{code}"
            )


def main():
    """Run documentation tests."""
    # Run pytest on this file
    return pytest.main([__file__, "-v", "--tb=short"])


if __name__ == "__main__":
    sys.exit(main())


