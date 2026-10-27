import math

import numpy as np
import pytest

from adrc import ADRC


# ---------------------------------------------------------------------------
# Construction / validation
# ---------------------------------------------------------------------------

def test_constructor_valid_orders():
    for order in range(1, 5):
        controller = ADRC(order)
        assert controller.controller_order == order + 1


@pytest.mark.parametrize('order', [0, -1, 5, 100])
def test_constructor_invalid_orders(order):
    with pytest.raises(ValueError):
        ADRC(order)


def test_step_before_initialize_raises():
    controller = ADRC(2)
    with pytest.raises(RuntimeError):
        controller.step(1.0, 0.0)


@pytest.mark.parametrize('kwargs', [
    {'Tsettle': 0.0},
    {'Tsettle': -1.0},
    {'kob': 0.0},
    {'kob': -5.0},
    {'dt': 0.0},
    {'dt': -0.01},
    {'Ke': 0.0},
    {'Ke': -1.0},
    {'input_delay': -0.1},
])
def test_initialize_invalid_params(kwargs):
    controller = ADRC(2)
    with pytest.raises(ValueError):
        controller.initialize(**kwargs)


def test_initialize_invalid_state_vector_length():
    controller = ADRC(2)
    with pytest.raises(ValueError):
        controller.initialize(Xhat_init=[0.0, 0.0])  # expects length 3


# ---------------------------------------------------------------------------
# Gains and matrices
# ---------------------------------------------------------------------------

def test_controller_gains_first_order():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0)
    # n=1: K = 4 / Tsettle = 4
    assert np.allclose(controller.K, [4.0])


def test_controller_gains_second_order():
    controller = ADRC(2)
    controller.initialize(Tsettle=1.0)
    # n=2: K = [36, 12] for Tsettle=1
    assert np.allclose(controller.K, [36.0, 12.0])


def test_eso_gains_first_order():
    controller = ADRC(1)
    dt = 0.01
    controller.initialize(Tsettle=1.0, kob=10, b0=1.0, dt=dt)
    zes = math.exp(-40.0 * dt)  # SESO = kob * (-4 / Tsettle)
    expected = np.array([1.0 - zes**2, (1.0 - zes)**2 / dt])
    assert np.allclose(controller.Ld, expected)


def test_eso_gain_vectors_have_expected_lengths():
    for order in range(1, 5):
        controller = ADRC(order)
        controller.initialize()
        assert len(controller.Ld) == order + 1
        assert len(controller.K) == order
        assert controller.Ad.shape == (order + 1, order + 1)


# ---------------------------------------------------------------------------
# Closed-loop behavior
# ---------------------------------------------------------------------------

def test_first_order_integrator_tracks_step():
    dt = 0.01
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, kob=10, b0=1.0, dt=dt)

    r = 1.0
    y = 0.0
    for _ in range(2000):
        u = controller.step(r, y)
        y = y + u * dt

    assert abs(y - r) < 0.01


def test_first_order_controls_env():
    dt = 0.01
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, kob=10, b0=2.0, dt=dt)

    # Plant: G(s) = b0 / (s + a) with unknown dynamics and disturbance-free
    a = 1.0
    r = 1.0
    y = 0.0
    for _ in range(3000):
        u = controller.step(r, y)
        y = y + dt * (-a * y + 2.0 * u)

    assert abs(y - r) < 0.02


def test_second_order_double_integrator_rejects_constant_disturbance():
    dt = 0.01
    controller = ADRC(2)
    controller.initialize(Tsettle=1.0, kob=10, b0=1.0, dt=dt)

    r = 1.0
    d = 0.5  # constant disturbance acting on acceleration
    p, v = 0.0, 0.0
    for _ in range(4000):
        u = controller.step(r, p)
        v = v + (u + d) * dt
        p = p + v * dt

    assert abs(p - r) < 0.02
    assert np.isfinite(u)


def test_estimated_disturbance_converges_to_true_disturbance():
    dt = 0.01
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, kob=10, b0=1.0, dt=dt)

    d = 0.3
    y = 0.0
    for _ in range(2000):
        u = controller.step(1.0, y)
        y = y + (u + d) * dt

    assert abs(controller.get_estimated_disturbance() - d) < 0.02


# ---------------------------------------------------------------------------
# Saturation
# ---------------------------------------------------------------------------

def test_saturation_limits_control_input():
    dt = 0.01
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, kob=10, b0=1.0,
                          dt=dt, u_min=-0.5, u_max=0.5)

    y = 0.0
    for _ in range(200):
        u = controller.step(1.0, y)
        assert -0.5 - 1e-12 <= u <= 0.5 + 1e-12
        y = y + u * dt


def test_saturation_lower_bound_only():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, b0=1.0, dt=0.01,
                          u_min=0.0, u_max=np.inf)
    u = controller.step(1.0, 0.0)
    assert u >= -1e-12


# ---------------------------------------------------------------------------
# Input delay
# ---------------------------------------------------------------------------

def test_input_delay_configures_history_buffer():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, dt=0.01, input_delay=0.05)
    assert controller.input_delay_steps == 5
    assert len(controller.u_history) == 6


def test_input_delay_runs():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, dt=0.01, input_delay=0.1)
    y = 0.0
    for _ in range(500):
        u = controller.step(1.0, y)
        y = y + u * 0.01
    assert abs(y - 1.0) < 0.05


# ---------------------------------------------------------------------------
# Cascaded mode / misc API
# ---------------------------------------------------------------------------

def test_cascaded_mode_runs_and_reporting():
    controller = ADRC(2)
    controller.initialize(Tsettle=1.0, b0=1.0, dt=0.01, use_cascaded=True)
    for _ in range(100):
        controller.step(1.0, 0.0)
    assert controller.use_cascaded
    assert len(controller.get_estimated_states()) == 3
    assert controller.get_estimated_disturbance() == \
        controller.get_m_states()[-1] + controller.get_z_states()[-1]


def test_reset_restores_state():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, b0=1.0, dt=0.01)
    for _ in range(50):
        controller.step(1.0, 0.0)
    controller.reset()
    assert np.allclose(controller.get_estimated_states(), 0.0)


def test_set_td_dynamically():
    controller = ADRC(1)
    controller.initialize(Tsettle=1.0, b0=1.0, dt=0.01)
    assert not controller.use_td
    controller.set_td('euler', 0.7)
    assert controller.use_td
    u = controller.step(1.0, 0.0)
    assert np.isfinite(u)


def test_update_tuning_recomputes_gains():
    controller = ADRC(2)
    controller.initialize(Tsettle=1.0, kob=10)
    k_before = controller.K.copy()
    controller.update_tuning(Tsettle=2.0, kob=5)
    assert not np.allclose(controller.K, k_before)