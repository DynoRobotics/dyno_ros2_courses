#!/usr/bin/env python3
"""
Generate API reference documentation from Python source code docstrings.

This tool extracts docstrings from Python modules, classes, and functions,
and generates formatted markdown files in docs/07-API-Reference/Python/.
"""

import ast
import inspect
import importlib.util
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any
from dataclasses import dataclass


@dataclass
class DocItem:
    """Represents a documented item (class, function, etc.)."""
    name: str
    kind: str  # "class", "function", "method", "property"
    docstring: Optional[str]
    signature: Optional[str]
    line_number: int


class APIDocGenerator:
    """Generate API documentation from Python source files."""
    
    def __init__(self, source_root: Path, output_root: Path):
        """
        Initialize generator.
        
        Args:
            source_root: Root directory of Python source code
            output_root: Root directory for generated docs
        """
        self.source_root = source_root
        self.output_root = output_root
        self.output_root.mkdir(parents=True, exist_ok=True)
    
    def generate_all(self) -> None:
        """Generate docs for all Python modules."""
        print("🔍 Scanning for Python modules...")
        
        python_files = list(self.source_root.rglob("*.py"))
        python_files = [f for f in python_files if not self._should_skip(f)]
        
        print(f"Found {len(python_files)} Python files")
        
        for py_file in python_files:
            try:
                self.generate_for_file(py_file)
            except Exception as e:
                print(f"⚠️  Failed to generate docs for {py_file}: {e}")
        
        print(f"✅ Generated API docs in {self.output_root}")
    
    def _should_skip(self, path: Path) -> bool:
        """Check if file should be skipped."""
        # Skip private modules, tests, __pycache__
        parts = path.parts
        return any(
            part.startswith("_") and part != "__init__.py"
            or part == "tests"
            or part == "__pycache__"
            for part in parts
        )
    
    def generate_for_file(self, source_file: Path) -> None:
        """Generate API doc for a single Python file."""
        relative_path = source_file.relative_to(self.source_root)
        
        # Parse AST
        with open(source_file, "r", encoding="utf-8") as f:
            source = f.read()
        
        try:
            tree = ast.parse(source, filename=str(source_file))
        except SyntaxError as e:
            print(f"⚠️  Syntax error in {source_file}: {e}")
            return
        
        # Extract documentation
        module_doc = ast.get_docstring(tree)
        classes = self._extract_classes(tree)
        functions = self._extract_functions(tree)
        
        if not module_doc and not classes and not functions:
            # Skip files with no public API
            return
        
        # Generate markdown
        output_file = self._get_output_path(relative_path)
        output_file.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_file, "w", encoding="utf-8") as f:
            self._write_markdown(
                f,
                source_file=relative_path,
                module_doc=module_doc,
                classes=classes,
                functions=functions,
            )
        
        print(f"📝 Generated {output_file}")
    
    def _get_output_path(self, relative_path: Path) -> Path:
        """Convert source path to output markdown path."""
        # python/ros2_zenoh_python/node.py -> Python/node.md
        # python/ros2_zenoh_python/testing/clock.py -> Python/testing/clock.md
        
        parts = list(relative_path.parts)
        
        # Remove package prefixes
        while parts and parts[0] in ("python", "ros2_zenoh_python", "ros2_zenoh"):
            parts.pop(0)
        
        # Change extension
        if parts:
            parts[-1] = parts[-1].replace(".py", ".md")
        
        # Special case: __init__.py becomes module name
        if parts and parts[-1] == "__init__.md":
            parts[-1] = "index.md"
        
        return self.output_root / "Python" / Path(*parts) if parts else self.output_root / "Python" / "index.md"
    
    def _extract_classes(self, tree: ast.Module) -> List[Dict[str, Any]]:
        """Extract class definitions with docstrings."""
        classes = []
        
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef):
                # Only top-level classes
                if any(isinstance(parent, ast.ClassDef) for parent in ast.walk(tree)):
                    continue
                
                class_info = {
                    "name": node.name,
                    "docstring": ast.get_docstring(node),
                    "methods": [],
                    "properties": [],
                    "line": node.lineno,
                }
                
                # Extract methods
                for item in node.body:
                    if isinstance(item, ast.FunctionDef):
                        if item.name.startswith("_") and item.name != "__init__":
                            continue  # Skip private methods
                        
                        method_info = {
                            "name": item.name,
                            "docstring": ast.get_docstring(item),
                            "signature": self._get_signature(item),
                            "line": item.lineno,
                        }
                        
                        # Check if it's a property
                        is_property = any(
                            isinstance(d, ast.Name) and d.id == "property"
                            for d in item.decorator_list
                        )
                        
                        if is_property:
                            class_info["properties"].append(method_info)
                        else:
                            class_info["methods"].append(method_info)
                
                classes.append(class_info)
        
        return classes
    
    def _extract_functions(self, tree: ast.Module) -> List[Dict[str, Any]]:
        """Extract top-level function definitions."""
        functions = []
        
        for node in tree.body:
            if isinstance(node, ast.FunctionDef):
                if node.name.startswith("_"):
                    continue  # Skip private functions
                
                functions.append({
                    "name": node.name,
                    "docstring": ast.get_docstring(node),
                    "signature": self._get_signature(node),
                    "line": node.lineno,
                })
        
        return functions
    
    def _get_signature(self, node: ast.FunctionDef) -> str:
        """Extract function signature as string."""
        args = []
        
        # Positional args
        for arg in node.args.args:
            arg_str = arg.arg
            if arg.annotation:
                arg_str += f": {ast.unparse(arg.annotation)}"
            args.append(arg_str)
        
        # *args
        if node.args.vararg:
            arg_str = f"*{node.args.vararg.arg}"
            if node.args.vararg.annotation:
                arg_str += f": {ast.unparse(node.args.vararg.annotation)}"
            args.append(arg_str)
        
        # **kwargs
        if node.args.kwarg:
            arg_str = f"**{node.args.kwarg.arg}"
            if node.args.kwarg.annotation:
                arg_str += f": {ast.unparse(node.args.kwarg.annotation)}"
            args.append(arg_str)
        
        # Return type
        return_type = ""
        if node.returns:
            return_type = f" -> {ast.unparse(node.returns)}"
        
        return f"{node.name}({', '.join(args)}){return_type}"
    
    def _write_markdown(
        self,
        f,
        source_file: Path,
        module_doc: Optional[str],
        classes: List[Dict],
        functions: List[Dict],
    ) -> None:
        """Write formatted markdown documentation."""
        # Frontmatter
        f.write("---\n")
        f.write("classification: public\n")
        f.write("llm_processing: allowed\n")
        f.write('schema_version: "1.0"\n')
        f.write("type: api-reference\n")
        f.write("auto_generated: true\n")
        f.write(f'source_file: "{source_file}"\n')
        f.write(f'last_generated: "{datetime.now().isoformat()}"\n')
        f.write("tags: [\"api\", \"python\", \"auto-generated\"]\n")
        f.write("---\n\n")
        
        # Module title
        module_name = source_file.stem if source_file.stem != "__init__" else source_file.parent.name
        f.write(f"# {module_name}\n\n")
        
        # Warning
        f.write("⚠️ **This file is auto-generated. Do not edit manually.**\n\n")
        f.write(f"**Source**: `{source_file}`\n\n")
        
        # Module docstring
        if module_doc:
            f.write(f"{module_doc}\n\n")
        
        # Classes
        for cls in classes:
            f.write(f"## {cls['name']}\n\n")
            
            if cls["docstring"]:
                f.write(f"{cls['docstring']}\n\n")
            
            # Constructor
            init_methods = [m for m in cls["methods"] if m["name"] == "__init__"]
            if init_methods:
                init = init_methods[0]
                f.write("### Constructor\n\n")
                f.write(f"```python\n{init['signature']}\n```\n\n")
                if init["docstring"]:
                    f.write(f"{init['docstring']}\n\n")
            
            # Methods
            regular_methods = [m for m in cls["methods"] if m["name"] != "__init__"]
            if regular_methods:
                f.write("### Methods\n\n")
                for method in regular_methods:
                    f.write(f"#### `{method['name']}`\n\n")
                    f.write(f"```python\n{method['signature']}\n```\n\n")
                    if method["docstring"]:
                        f.write(f"{method['docstring']}\n\n")
            
            # Properties
            if cls["properties"]:
                f.write("### Properties\n\n")
                for prop in cls["properties"]:
                    f.write(f"#### `{prop['name']}`\n\n")
                    if prop["docstring"]:
                        f.write(f"{prop['docstring']}\n\n")
        
        # Functions
        if functions:
            f.write("## Functions\n\n")
            for func in functions:
                f.write(f"### `{func['name']}`\n\n")
                f.write(f"```python\n{func['signature']}\n```\n\n")
                if func["docstring"]:
                    f.write(f"{func['docstring']}\n\n")


def main():
    """Main entry point."""
    # Find project root
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    
    # Paths
    source_root = project_root / "python" / "ros2_zenoh_python"
    output_root = project_root / "docs" / "07-API-Reference"
    
    if not source_root.exists():
        print(f"❌ Source directory not found: {source_root}")
        print("   This tool should be run from the ros2_zenoh/ directory")
        sys.exit(1)
    
    # Generate
    generator = APIDocGenerator(source_root, output_root)
    generator.generate_all()


if __name__ == "__main__":
    main()


