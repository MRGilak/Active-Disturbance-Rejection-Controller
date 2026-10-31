import math

import numpy as np
import pytest

from adrc import TD


def test_invalid_method_raises():
    with pytest.raises(ValueError):
        TD('unknown_method', 0.01)


def test_nonpositive_sample_time_raises():
    with pytest.raises(ValueError):
        TD('euler', 0.0)
    with pytest.raises(ValueError):
        TD('euler', -0.01)


def test_all_methods_recognized():
    for method in ['euler', 'tod', 'ld', 'red', 'intd']:
        td = TD(method, 0.01)
        td.update(1.0)
        assert np.isfinite(td.y)
        assert np.isfinite(td.yd)
        assert np.isfinite(td.ydd)


def test_euler_tracks_constant_input():
    td = TD('euler', 0.01, 0.9)
    for _ in range(500):
        td.update(1.0)
    assert abs(td.y - 1.0) < 1e-6


def test_euler_derivative_of_constant_goes_to_zero():
    td = TD('euler', 0.01, 0.9)
    for _ in range(500):
        td.update(1.0)
    assert abs(td.yd) < 1e-3


def test_euler_derivative_of_ramp_approximates_slope():
    td = TD('euler', 0.01, 0.9)
    n = 500
    for k in range(n):
        ref = 2.0 * (k + 1) * 0.01
        td.update(ref)
    # Steady-state derivative of a ramp is the slope
    assert 1.5 < td.yd < 2.5


def test_reset():
    td = TD('euler', 0.01, 0.9)
    for _ in range(50):
        td.update(1.0)
    td.reset()
    assert td.y == 0.0
    assert td.yd == 0.0
    assert td.ydd == 0.0


def test_tod_tracks_constant_input():
    td = TD('tod', 0.01, 5.0)
    for _ in range(2000):
        td.update(1.0)
    assert abs(td.y - 1.0) < 0.02


def test_ld_tracks_constant_input():
    td = TD('ld', 0.001, 0.5)
    for _ in range(5000):
        td.update(1.0)
    assert abs(td.y - 1.0) < 0.02


def test_red_tracks_constant_input():
    td = TD('red', 0.01, 4.0, 2.0)
    for _ in range(2000):
        td.update(1.0)
    assert abs(td.y - 1.0) < 0.02


def test_intd_valid_params_run_finite():
    td = TD('intd', 0.01, 1.0, 1.0, 1.0, 1.0)
    for _ in range(300):
        td.update(1.0)
    assert np.isfinite(td.y)
    assert np.isfinite(td.yd)


def test_intd_invalid_params_raise():
    with pytest.raises(ValueError):
        TD('intd', 0.01, 1.0, 0.0, 1.0, 1.0).update(1.0)  # beta <= 0
    with pytest.raises(ValueError):
        TD('intd', 0.01, 1.0, 1.0, 0.0, 1.0).update(1.0)  # gamma <= 0
    with pytest.raises(ValueError):
        TD('intd', 0.01, 1.0, 1.0, 1.0, 0.0).update(1.0)  # r <= 0


def test_set_sample_time_and_parameters():
    td = TD('euler', 0.01, 0.9)
    td.set_sample_time(0.001)
    td.set_parameters(0.5)
    td.update(1.0)
    assert td.dt == 0.001
    assert td.params == [0.5]
    with pytest.raises(ValueError):
        td.set_sample_time(0.0)


def test_method_case_insensitive():
    td = TD('EULER', 0.01, 0.9)
    td.update(1.0)
    assert td.method == 'euler'