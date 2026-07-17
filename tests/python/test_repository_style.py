"""Regression checks for repository-wide Python documentation and call layout."""

from __future__ import annotations

import ast
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOTS = (REPOSITORY_ROOT / "src", REPOSITORY_ROOT / "run")


def python_files() -> list[Path]:
    """Return every source-controlled Python file covered by the style checks."""
    files: list[Path] = []
    for root in PYTHON_ROOTS:
        files.extend(path for path in root.rglob("*.py") if "__pycache__" not in path.parts)
    return sorted(files)


def test_every_module_class_and_function_has_documentation() -> None:
    """Require module, class, function, and method docstrings in all Python sources."""
    missing: list[str] = []
    for path in python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        if ast.get_docstring(tree) is None:
            missing.append(f"{path.relative_to(REPOSITORY_ROOT)}: module")
        for node in ast.walk(tree):
            if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)) and ast.get_docstring(node) is None:
                missing.append(f"{path.relative_to(REPOSITORY_ROOT)}:{node.lineno} {node.name}")
    assert not missing, "Missing Python documentation:\n" + "\n".join(missing)


def test_small_calls_remain_on_one_line() -> None:
    """Keep calls with fewer than five explicit arguments on one source line."""
    violations: list[str] = []
    for path in python_files():
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            argument_count = len(node.args) + len(node.keywords)
            if argument_count < 5 and node.end_lineno is not None and node.end_lineno > node.lineno:
                violations.append(f"{path.relative_to(REPOSITORY_ROOT)}:{node.lineno} has {argument_count} arguments")
    assert not violations, "Small calls split across lines:\n" + "\n".join(violations)
