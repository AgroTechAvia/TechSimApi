"""Static compatibility checks for the public high-level examples."""
from __future__ import annotations

import ast
import inspect
from pathlib import Path

from agrotechsimapi.high_level_client import HighLevelSimClient


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples_high_level"


def _example_trees():
    files = sorted(EXAMPLES.rglob("*.py"))
    assert files
    for path in files:
        yield path, ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _looks_like_high_level_client(node: ast.expr) -> bool:
    if isinstance(node, ast.Name):
        return node.id in {"client", "drone"}
    return isinstance(node, ast.Attribute) and node.attr in {"client", "drone"}


def test_high_level_examples_only_call_current_client_methods():
    methods = {
        name: member
        for name, member in inspect.getmembers(HighLevelSimClient, inspect.isfunction)
        if not name.startswith("_")
    }
    failures = []
    for path, tree in _example_trees():
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if not _looks_like_high_level_client(node.func.value):
                continue
            name = node.func.attr
            if name not in methods:
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: unknown method {name}")
                continue
            if any(isinstance(arg, ast.Starred) for arg in node.args):
                continue
            if any(keyword.arg is None for keyword in node.keywords):
                continue
            signature = inspect.signature(methods[name])
            try:
                signature.bind(
                    None,
                    *(None for _ in node.args),
                    **{keyword.arg: None for keyword in node.keywords},
                )
            except TypeError as exc:
                failures.append(
                    f"{path.relative_to(ROOT)}:{node.lineno}: {name}{signature}: {exc}"
                )
    assert not failures, "\n".join(failures)


def test_high_level_client_constructors_match_current_signature():
    signature = inspect.signature(HighLevelSimClient)
    failures = []
    for path, tree in _example_trees():
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
                continue
            if node.func.id != "HighLevelSimClient":
                continue
            try:
                signature.bind(
                    *(None for _ in node.args),
                    **{keyword.arg: None for keyword in node.keywords if keyword.arg is not None},
                )
            except TypeError as exc:
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: {exc}")
    assert not failures, "\n".join(failures)


def test_high_level_examples_do_not_start_flight_on_import():
    failures = []
    for path, tree in _example_trees():
        for node in tree.body:
            if isinstance(node, ast.Expr) and isinstance(node.value, ast.Call):
                call = node.value
                if isinstance(call.func, ast.Name) and call.func.id == "main":
                    failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: unguarded main()")
                elif isinstance(call.func, ast.Attribute) and _looks_like_high_level_client(call.func.value):
                    failures.append(
                        f"{path.relative_to(ROOT)}:{node.lineno}: unguarded {call.func.attr}()"
                    )
            if isinstance(node, (ast.While, ast.For)):
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: top-level loop")
    assert not failures, "\n".join(failures)


def test_high_level_examples_use_current_default_msp_port():
    failures = []
    for path, tree in _example_trees():
        for node in tree.body:
            if not isinstance(node, (ast.Assign, ast.AnnAssign)):
                continue
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if not any(isinstance(target, ast.Name) and target.id == "port" for target in targets):
                continue
            value = node.value
            if not isinstance(value, ast.Constant) or value.value != 5762:
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: port must be 5762")
    assert not failures, "\n".join(failures)
