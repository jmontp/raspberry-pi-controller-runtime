"""Unit tests for vertical GRF staleness handling."""

from rpc_runtime.sensors.grf.base import BaseVerticalGRFConfig, GRFStaleDataError
from rpc_runtime.sensors.grf.mock import MockVerticalGRF


def test_mock_grf_fallback_sample() -> None:
    """Fallback strategy should return zeroed forces when data is stale."""
    config = BaseVerticalGRFConfig(max_stale_samples=1, fault_strategy="fallback")
    grf = MockVerticalGRF(generator=None, config_override=config, loop=True)
    grf.read()
    grf.read()
    diag = grf.diagnostics
    assert diag.hz_estimate is not None
    fallback = grf._handle_sample(None, fresh=False)
    assert all(force == 0.0 for force in fallback.forces_newton)


def test_mock_grf_stale_raise() -> None:
    """Raise strategy should surface `GRFStaleDataError` when stale."""
    config = BaseVerticalGRFConfig(max_stale_samples=1, fault_strategy="raise")
    grf = MockVerticalGRF(generator=None, config_override=config, loop=True)
    grf.read()
    grf.read()
    try:
        grf._handle_sample(None, fresh=False)
    except GRFStaleDataError:
        return
    raise AssertionError("Expected GRFStaleDataError when stale data exceeds threshold")
