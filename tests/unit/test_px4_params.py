import asyncio

import pytest

from config import settings
from flight.px4_params import PX4ParameterAudit


class FakeParamPlugin:
    """Fake MAVSDK param plugin. missing_params raise; others return their
    configured value. `hang_params` never resolve, to exercise the
    per-read timeout."""

    def __init__(self, values=None, missing_params=(), hang_params=()):
        self.values = values or {}
        self.missing_params = set(missing_params)
        self.hang_params = set(hang_params)

    async def get_param_float(self, name):
        return await self._get(name)

    async def get_param_int(self, name):
        return await self._get(name)

    async def _get(self, name):
        if name in self.hang_params:
            await asyncio.sleep(3600)  # never resolves in test time
        if name in self.missing_params or name not in self.values:
            raise RuntimeError(f"Param {name} not found")
        return self.values[name]


class FakeSystem:
    def __init__(self, param_plugin):
        self.param = param_plugin


class FakeDrone:
    def __init__(self, param_plugin):
        self.system = FakeSystem(param_plugin)


async def test_missing_parameter_warns_but_does_not_raise():
    drone = FakeDrone(FakeParamPlugin(missing_params=["BAT_LOW_THR"]))
    audit = PX4ParameterAudit(drone)

    values, warnings = await audit.run()  # must not raise

    assert values["BAT_LOW_THR"] is None


async def test_timeout_on_a_single_parameter_warns_but_does_not_raise():
    drone = FakeDrone(FakeParamPlugin(hang_params=["MPC_XY_VEL_MAX"]))
    audit = PX4ParameterAudit(drone, timeout=0.05)

    values, warnings = await audit.run()  # must not raise/hang

    assert values["MPC_XY_VEL_MAX"] is None


async def test_run_never_raises_even_on_unexpected_failure(monkeypatch):
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    async def broken_read_all():
        raise RuntimeError("something went badly wrong")

    monkeypatch.setattr(audit, "read_all", broken_read_all)

    values, warnings = await audit.run()  # must not raise
    assert values == {}
    assert warnings == []


def test_check_consistency_warns_when_app_limit_exceeds_px4_limit():
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    px4_limit_below_app_limit = settings.MAX_HORIZONTAL_SPEED_M_S - 1
    values = {"MPC_XY_VEL_MAX": px4_limit_below_app_limit}

    warnings = audit.check_consistency(values)

    assert any("MAX_HORIZONTAL_SPEED_M_S" in w for w in warnings)


def test_check_consistency_silent_when_app_limit_is_stricter():
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    px4_limit_above_app_limit = settings.MAX_HORIZONTAL_SPEED_M_S + 5
    values = {"MPC_XY_VEL_MAX": px4_limit_above_app_limit}

    warnings = audit.check_consistency(values)

    assert not any("MAX_HORIZONTAL_SPEED_M_S" in w for w in warnings)


def test_check_consistency_warns_when_ascent_limit_exceeds_px4():
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    values = {"MPC_Z_VEL_MAX_UP": settings.MAX_ASCENT_SPEED_M_S - 0.5}

    warnings = audit.check_consistency(values)

    assert any("MAX_ASCENT_SPEED_M_S" in w for w in warnings)


def test_check_consistency_warns_when_descent_limit_exceeds_px4():
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    values = {"MPC_Z_VEL_MAX_DN": settings.MAX_DESCENT_SPEED_M_S - 0.5}

    warnings = audit.check_consistency(values)

    assert any("MAX_DESCENT_SPEED_M_S" in w for w in warnings)


def test_check_consistency_skips_missing_values_silently():
    drone = FakeDrone(FakeParamPlugin())
    audit = PX4ParameterAudit(drone)

    warnings = audit.check_consistency({"MPC_XY_VEL_MAX": None})

    assert warnings == []
