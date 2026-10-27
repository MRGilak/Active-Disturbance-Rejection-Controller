#include "adrc/ADRC.hpp"
#include "adrc/TD.hpp"

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

static int g_failures = 0;
static int g_checks = 0;

static void report_failure(const std::string& file, int line, const std::string& msg) {
    ++g_failures;
    std::cerr << "FAILED [" << file << ":" << line << "] " << msg << std::endl;
}

#define CHECK(cond)                             \
    do {                                        \
        ++g_checks;                             \
        if (!(cond)) {                          \
            report_failure(__FILE__, __LINE__, #cond); \
        }                                       \
    } while (0)

#define CHECK_CLOSE(actual, expected, tol)                          \
    do {                                                            \
        ++g_checks;                                                 \
        double a_ = (actual);                                       \
        double e_ = (expected);                                     \
        if (!(std::abs(a_ - e_) <= (tol))) {                        \
            report_failure(__FILE__, __LINE__,                       \
                           std::string(#actual " = ") +             \
                           std::to_string(a_) +                     \
                           " != " #expected " = " +                \
                           std::to_string(e_) +                     \
                           " (tol " + std::to_string((tol)) + ")"); \
        }                                                           \
    } while (0)

static bool throws_invalid_argument(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::invalid_argument&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

static bool throws_runtime_error(const std::function<void()>& fn) {
    try {
        fn();
    } catch (const std::runtime_error&) {
        return true;
    } catch (...) {
        return false;
    }
    return false;
}

// ---------------------------------------------------------------------------
// TD tests
// ---------------------------------------------------------------------------

static void test_td_invalid_inputs() {
    // Non-positive sample time and unknown methods must throw
    CHECK(throws_invalid_argument([] { TD("euler", 0.0); }));
    CHECK(throws_invalid_argument([] { TD("euler", -0.01); }));
    CHECK(throws_invalid_argument([] { TD("unknown_method", 0.01); }));
}

static void test_td_all_methods_construct_and_update() {
    const std::vector<std::string> methods = {"euler", "tod", "ld", "red", "intd"};
    for (const auto& m : methods) {
        TD td(m, 0.01);
        td.update(1.0);
        CHECK(std::isfinite(td.y));
        CHECK(std::isfinite(td.yd));
        CHECK(std::isfinite(td.ydd));
    }
}

static void test_td_euler_tracks_constant_input() {
    TD td("euler", 0.01);
    for (int i = 0; i < 500; ++i) {
        td.update(1.0);
    }
    CHECK_CLOSE(td.y, 1.0, 1e-6);
    CHECK_CLOSE(td.yd, 0.0, 1e-3);
}

static void test_td_euler_ramp_derivative() {
    TD td("euler", 0.01);
    const int n = 500;
    for (int k = 0; k < n; ++k) {
        td.update(2.0 * (k + 1) * 0.01);
    }
    CHECK_CLOSE(td.yd, 2.0, 0.5);
}

static void test_td_reset() {
    TD td("euler", 0.01);
    for (int i = 0; i < 50; ++i) {
        td.update(1.0);
    }
    td.reset();
    CHECK_CLOSE(td.y, 0.0, 1e-12);
    CHECK_CLOSE(td.yd, 0.0, 1e-12);
    CHECK_CLOSE(td.ydd, 0.0, 1e-12);
}

static void test_td_tod_tracks_constant_input() {
    TD td("tod", 0.01, {5.0});
    for (int i = 0; i < 2000; ++i) {
        td.update(1.0);
    }
    CHECK_CLOSE(td.y, 1.0, 0.02);
}

static void test_td_ld_tracks_constant_input() {
    TD td("ld", 0.001, {0.5});
    for (int i = 0; i < 5000; ++i) {
        td.update(1.0);
    }
    CHECK_CLOSE(td.y, 1.0, 0.02);
}

static void test_td_red_tracks_constant_input() {
    TD td("red", 0.01, {4.0, 2.0});
    for (int i = 0; i < 2000; ++i) {
        td.update(1.0);
    }
    CHECK_CLOSE(td.y, 1.0, 0.02);
}

static void test_td_intd_validation_and_run() {
    // Invalid parameters are rejected when the INTD step runs
    CHECK(throws_invalid_argument([] {
        TD td("intd", 0.01, {1.0, 0.0, 1.0, 1.0});  // beta <= 0
        td.update(1.0);
    }));
    CHECK(throws_invalid_argument([] {
        TD td("intd", 0.01, {1.0, 1.0, 0.0, 1.0});  // gamma <= 0
        td.update(1.0);
    }));
    CHECK(throws_invalid_argument([] {
        TD td("intd", 0.01, {1.0, 1.0, 1.0, 0.0});  // r <= 0
        td.update(1.0);
    }));

    // Valid parameters: finite outputs after updates
    TD td("intd", 0.01, {1.0, 1.0, 1.0, 1.0});
    for (int i = 0; i < 300; ++i) {
        td.update(1.0);
    }
    CHECK(std::isfinite(td.y));
    CHECK(std::isfinite(td.yd));
}

static void test_td_setters() {
    TD td("euler", 0.01);
    td.set_sample_time(0.001);
    td.set_parameters({0.5});
    td.update(1.0);
    CHECK(std::isfinite(td.y));
    CHECK(throws_invalid_argument([&] { td.set_sample_time(0.0); }));
}

// ---------------------------------------------------------------------------
// ADRC tests
// ---------------------------------------------------------------------------

static void test_adrc_constructor_validation() {
    for (int order = 1; order <= 4; ++order) {
        ADRC controller(order);
        controller.initialize();
        CHECK(controller.getEstimatedStates().size() == (size_t)(order + 1));
    }
    CHECK(throws_invalid_argument([] { ADRC(0); }));
    CHECK(throws_invalid_argument([] { ADRC(-1); }));
    CHECK(throws_invalid_argument([] { ADRC(5); }));
}

static void test_adrc_step_before_initialize() {
    ADRC controller(2);
    CHECK(throws_runtime_error([&] { controller.step(1.0, 0.0); }));
}

static void test_adrc_initialize_validation() {
    ADRC controller(2);
    CHECK(throws_invalid_argument([&] { controller.initialize(0.0); }));   // Tsettle
    CHECK(throws_invalid_argument([&] { controller.initialize(1.0, 0.0); }));  // kob
    CHECK(throws_invalid_argument([&] { controller.initialize(1.0, 10.0, 1.0, -1e30, 1e30, 0.0); }));  // dT
    CHECK(throws_invalid_argument([&] { controller.initialize(1.0, 10.0, 1.0, -1e30, 1e30, 0.01, {}, 0.0, 0.0); }));  // Ke
    CHECK(throws_invalid_argument([&] { controller.initialize(1.0, 10.0, 1.0, -1e30, 1e30, 0.01, {}, 0.0, 1.0, -0.1); }));  // inputDelay
    // Wrong state-vector length
    CHECK(throws_invalid_argument([&] { controller.initialize(1.0, 10.0, 1.0, -1e30, 1e30, 0.01, {0.0, 0.0}); }));
}

static void test_adrc_first_order_integrator_tracks_step() {
    const double dt = 0.01;
    ADRC controller(1);
    controller.initialize(1.0, 10.0, 1.0, -std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 0.01);

    const double r = 1.0;
    double y = 0.0;
    for (int i = 0; i < 2000; ++i) {
        double u = controller.step(r, y);
        y += u * dt;
    }
    CHECK_CLOSE(y, r, 0.01);
}

static void test_adrc_first_order_rejects_constant_disturbance() {
    const double dt = 0.01;
    ADRC controller(1);
    controller.initialize(1.0, 10.0, 1.0, -std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 0.01);

    const double d = 0.3;
    double y = 0.0;
    for (int i = 0; i < 2000; ++i) {
        double u = controller.step(1.0, y);
        y += (u + d) * dt;
    }
    CHECK_CLOSE(controller.getEstimatedDisturbance(), d, 0.02);
}

static void test_adrc_second_order_double_integrator() {
    const double dt = 0.01;
    ADRC controller(2);
    controller.initialize(1.0, 10.0, 1.0, -std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 0.01);

    const double r = 1.0;
    const double d = 0.5;
    double p = 0.0, v = 0.0, u = 0.0;
    for (int i = 0; i < 4000; ++i) {
        u = controller.step(r, p);
        v += (u + d) * dt;
        p += v * dt;
    }
    CHECK_CLOSE(p, r, 0.02);
    CHECK(std::isfinite(u));
}

static void test_adrc_saturation() {
    ADRC controller(1);
    controller.initialize(1.0, 10.0, 1.0, -0.5, 0.5, 0.01);

    double y = 0.0;
    for (int i = 0; i < 200; ++i) {
        double u = controller.step(1.0, y);
        CHECK(u >= -0.5 - 1e-12 && u <= 0.5 + 1e-12);
        y += u * 0.01;
    }
}

static void test_adrc_input_delay() {
    ADRC controller(1);
    controller.initialize(1.0, 10.0, 1.0, -std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 0.01, {}, 0.0, 1.0, 0.1);
    double y = 0.0;
    for (int i = 0; i < 500; ++i) {
        double u = controller.step(1.0, y);
        y += u * 0.01;
    }
    CHECK_CLOSE(y, 1.0, 0.05);
}

static void test_adrc_cascaded_mode() {
    ADRC controller(2);
    controller.initialize(1.0, 10.0, 1.0, -std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::infinity(), 0.01, {}, 0.0, 1.0, 0.0,
                          std::string("none"), {}, true);

    for (int i = 0; i < 100; ++i) {
        controller.step(1.0, 0.0);
    }
    CHECK(controller.getEstimatedStates().size() == 3);
    CHECK(controller.getMStates().size() == 3);
    CHECK_CLOSE(controller.getEstimatedDisturbance(),
                controller.getMStates().back() + controller.getZStates().back(), 1e-12);
}

static void test_adrc_reset_and_setters() {
    ADRC controller(1);
    controller.initialize();
    for (int i = 0; i < 50; ++i) {
        controller.step(1.0, 0.0);
    }
    controller.reset();
    for (double x : controller.getEstimatedStates()) {
        CHECK_CLOSE(x, 0.0, 1e-12);
    }

    controller.setSaturation(-1.0, 1.0);
    double u = controller.step(1.0, 0.0);
    CHECK(u >= -1.0 - 1e-12 && u <= 1.0 + 1e-12);

    controller.setTD("euler", {0.7});
    u = controller.step(1.0, 0.0);
    CHECK(std::isfinite(u));

    controller.updateTuning(2.0, 5.0);
    u = controller.step(1.0, 0.0);
    CHECK(std::isfinite(u));
}

// ---------------------------------------------------------------------------

int main() {
    test_td_invalid_inputs();
    test_td_all_methods_construct_and_update();
    test_td_euler_tracks_constant_input();
    test_td_euler_ramp_derivative();
    test_td_reset();
    test_td_tod_tracks_constant_input();
    test_td_ld_tracks_constant_input();
    test_td_red_tracks_constant_input();
    test_td_intd_validation_and_run();
    test_td_setters();

    test_adrc_constructor_validation();
    test_adrc_step_before_initialize();
    test_adrc_initialize_validation();
    test_adrc_first_order_integrator_tracks_step();
    test_adrc_first_order_rejects_constant_disturbance();
    test_adrc_second_order_double_integrator();
    test_adrc_saturation();
    test_adrc_input_delay();
    test_adrc_cascaded_mode();
    test_adrc_reset_and_setters();

    std::cout << g_checks << " checks, " << g_failures << " failures" << std::endl;
    if (g_failures > 0) {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}