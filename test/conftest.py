# tests/integration/conftest.py
import pytest

def pytest_addoption(parser):
    """Add command line option for simulator tests."""
    parser.addoption(
        "--with-simulator",
        action="store_true",
        default=False,
        help="Run tests that require a running simulator"
    )

def pytest_configure(config):
    """Register simulator marker."""
    config.addinivalue_line(
        "markers", "simulator: mark test as requiring a running simulator"
    )

def pytest_collection_modifyitems(config, items):
    """Skip simulator tests unless --with-simulator is specified."""
    if config.getoption("--with-simulator"):
        # Если указан флаг --with-simulator, НЕ пропускаем тесты
        return
    
    skip_simulator = pytest.mark.skip(
        reason="Need --with-simulator option to run (requires running simulator)"
    )
    
    for item in items:
        if "simulator" in item.keywords:
            item.add_marker(skip_simulator)