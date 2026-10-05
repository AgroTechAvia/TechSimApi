"""Static compatibility checks for the public low-level examples."""
from __future__ import annotations

import ast
import inspect
from pathlib import Path

from agrotechsimapi.client import SimClient


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples_low_level"
API_PREFIXES = ("get_", "set_", "call_", "start_", "stop_", "close_")


def _looks_like_sim_client(node: ast.expr) -> bool:
    if isinstance(node, ast.Name):
        return node.id in {"client", "sim_client", "client_1", "client_2"}
    return isinstance(node, ast.Attribute) and node.attr == "client"


def _example_trees():
    files = sorted(EXAMPLES.rglob("*.py"))
    assert files
    for path in files:
        yield path, ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_low_level_examples_only_call_current_sim_client_methods():
    methods = {
        name: member for name, member in inspect.getmembers(SimClient, inspect.isfunction)
        if not name.startswith("_")
    }
    failures = []
    for path, tree in _example_trees():
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if not _looks_like_sim_client(node.func.value):
                continue
            name = node.func.attr
            if not name.startswith(API_PREFIXES):
                continue
            if name not in methods:
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}: {name}")
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


def test_low_level_examples_do_not_run_on_import():
    failures = []
    for path, tree in _example_trees():
        for node in tree.body:
            if (isinstance(node, ast.Expr) and isinstance(node.value, ast.Call)
                    and isinstance(node.value.func, ast.Name)
                    and node.value.func.id in {"main", "serve"}):
                failures.append(f"{path.relative_to(ROOT)}:{node.lineno}")
    assert not failures, "unguarded example entry points:\n" + "\n".join(failures)


def test_mjpeg_home_page_embeds_the_camera_stream():
    path = EXAMPLES / "video" / "mjpeg_stream_example.py"
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    page = next(
        node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "PAGE"
                for target in node.targets)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    )
    assert "<img" in page
    assert 'src="{{ feed_url }}"' in page
    assert "/video_feed" in path.read_text(encoding="utf-8")
