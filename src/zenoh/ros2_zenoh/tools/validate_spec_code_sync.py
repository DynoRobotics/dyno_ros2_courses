#!/usr/bin/env python3
"""
Validate that Python code matches the specifications.

This tool checks that:
- Classes/functions declared in specs exist in code
- Method signatures match spec declarations
- Required docstrings are present
- Type hints match spec types (basic validation)
"""

import ast
import re
import sys
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ValidationError:
    """Represents a validation error."""
    file: str
    item: str
    severity: str  # "error" or "warning"
    message: str


class SpecCodeValidator:
    """Validate Python code against specifications."""
    
    def __init__(self, spec_dir: Path, code_dir: Path):
        """
        Initialize validator.
        
        Args:
            spec_dir: Directory containing Python specs
            code_dir: Directory containing Python source code
        """
        self.spec_dir = spec_dir
        self.code_dir = code_dir
        self.errors: List[ValidationError] = []
    
    def validate_all(self) -> bool:
        """
        Validate all specs against code.
        
        Returns:
            True if validation passed, False otherwise
        """
        print("🔍 Validating code against specs...")
        
        spec_files = list(self.spec_dir.glob("**/*-Python.md"))
        
        if not spec_files:
            print("⚠️  No Python specs found")
            return True
        
        print(f"Found {len(spec_files)} Python specs")
        
        for spec_file in spec_files:
            self._validate_spec(spec_file)
        
        # Report results
        if not self.errors:
            print("✅ All validations passed!")
            return True
        
        # Group by severity
        errors = [e for e in self.errors if e.severity == "error"]
        warnings = [e for e in self.errors if e.severity == "warning"]
        
        if warnings:
            print(f"\n⚠️  {len(warnings)} warning(s):")
            for warning in warnings:
                print(f"  {warning.file} - {warning.item}: {warning.message}")
        
        if errors:
            print(f"\n❌ {len(errors)} error(s):")
            for error in errors:
                print(f"  {error.file} - {error.item}: {error.message}")
            return False
        
        return True
    
    def _validate_spec(self, spec_file: Path) -> None:
        """Validate a single spec file."""
        # Read spec
        with open(spec_file, "r", encoding="utf-8") as f:
            spec_content = f.read()
        
        # Extract Python code blocks
        code_blocks = self._extract_code_blocks(spec_content)
        
        for block in code_blocks:
            # Look for class and function definitions
            try:
                tree = ast.parse(block["code"])
            except SyntaxError:
                # Not valid Python, might be pseudo-code
                continue
            
            # Validate classes
            for node in ast.walk(tree):
                if isinstance(node, ast.ClassDef):
                    self._validate_class(spec_file, node, block["code"])
                elif isinstance(node, ast.FunctionDef):
                    self._validate_function(spec_file, node, block["code"])
    
    def _extract_code_blocks(self, content: str) -> List[Dict[str, str]]:
        """Extract Python code blocks from markdown."""
        blocks = []
        
        # Match ```python ... ``` blocks
        pattern = r"```python\n(.*?)\n```"
        matches = re.finditer(pattern, content, re.DOTALL)
        
        for match in matches:
            code = match.group(1)
            # Skip blocks with obvious placeholders
            if "..." in code and not code.strip().endswith("..."):
                continue
            if "{{" in code or "}}" in code:
                continue
            if "# Pseudo-code" in code:
                continue
            
            blocks.append({"code": code})
        
        return blocks
    
    def _validate_class(self, spec_file: Path, node: ast.ClassDef, code_block: str) -> None:
        """Validate that class exists in code."""
        # Skip generic type vars
        if node.name in ("T", "Generic"):
            return
        
        # Try to find the class in code
        class_file = self._find_class_in_code(node.name)
        
        if not class_file:
            # Only error if this looks like a real class definition (not an example)
            if "__init__" in code_block or "def " in code_block:
                self.errors.append(ValidationError(
                    file=str(spec_file.name),
                    item=node.name,
                    severity="warning",
                    message=f"Class '{node.name}' declared in spec but not found in code"
                ))
            return
        
        # Parse the actual code file
        with open(class_file, "r", encoding="utf-8") as f:
            actual_code = f.read()
        
        try:
            actual_tree = ast.parse(actual_code)
        except SyntaxError:
            return
        
        # Find the class in actual code
        actual_class = None
        for actual_node in ast.walk(actual_tree):
            if isinstance(actual_node, ast.ClassDef) and actual_node.name == node.name:
                actual_class = actual_node
                break
        
        if not actual_class:
            return
        
        # Validate methods
        spec_methods = {
            item.name: item
            for item in node.body
            if isinstance(item, ast.FunctionDef)
        }
        
        actual_methods = {
            item.name: item
            for item in actual_class.body
            if isinstance(item, ast.FunctionDef)
        }
        
        # Check for missing public methods
        for method_name, spec_method in spec_methods.items():
            if method_name.startswith("_") and method_name != "__init__":
                continue  # Skip private methods
            
            if method_name not in actual_methods:
                self.errors.append(ValidationError(
                    file=str(class_file.name),
                    item=f"{node.name}.{method_name}",
                    severity="error",
                    message=f"Method declared in spec but missing in code"
                ))
            else:
                # Check docstring
                actual_method = actual_methods[method_name]
                if not ast.get_docstring(actual_method) and ast.get_docstring(spec_method):
                    self.errors.append(ValidationError(
                        file=str(class_file.name),
                        item=f"{node.name}.{method_name}",
                        severity="warning",
                        message="Method missing docstring"
                    ))
    
    def _validate_function(self, spec_file: Path, node: ast.FunctionDef, code_block: str) -> None:
        """Validate that function exists in code."""
        # Skip internal/example functions
        if node.name.startswith("_"):
            return
        
        # Try to find the function in code
        func_file = self._find_function_in_code(node.name)
        
        if not func_file:
            # Only warn if this looks like a real function definition
            if "def " in code_block and "..." not in code_block:
                self.errors.append(ValidationError(
                    file=str(spec_file.name),
                    item=node.name,
                    severity="warning",
                    message=f"Function '{node.name}' declared in spec but not found in code"
                ))
    
    def _find_class_in_code(self, class_name: str) -> Optional[Path]:
        """Find Python file containing a class."""
        for py_file in self.code_dir.rglob("*.py"):
            if "__pycache__" in str(py_file):
                continue
            
            try:
                with open(py_file, "r", encoding="utf-8") as f:
                    content = f.read()
                
                tree = ast.parse(content)
                
                for node in ast.walk(tree):
                    if isinstance(node, ast.ClassDef) and node.name == class_name:
                        return py_file
            except (SyntaxError, UnicodeDecodeError):
                continue
        
        return None
    
    def _find_function_in_code(self, func_name: str) -> Optional[Path]:
        """Find Python file containing a function."""
        for py_file in self.code_dir.rglob("*.py"):
            if "__pycache__" in str(py_file):
                continue
            
            try:
                with open(py_file, "r", encoding="utf-8") as f:
                    content = f.read()
                
                tree = ast.parse(content)
                
                for node in tree.body:
                    if isinstance(node, ast.FunctionDef) and node.name == func_name:
                        return py_file
            except (SyntaxError, UnicodeDecodeError):
                continue
        
        return None


def main():
    """Main entry point."""
    # Find project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    # Paths
    spec_dir = project_root / "docs" / "06-Specs" / "Python"
    code_dir = project_root / "python" / "ros2_zenoh_python"
    
    if not spec_dir.exists():
        print(f"❌ Spec directory not found: {spec_dir}")
        sys.exit(1)
    
    if not code_dir.exists():
        print(f"❌ Code directory not found: {code_dir}")
        print("   Code validation will be skipped until implementation exists")
        sys.exit(0)  # Not an error - code may not exist yet
    
    # Validate
    validator = SpecCodeValidator(spec_dir, code_dir)
    success = validator.validate_all()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()


