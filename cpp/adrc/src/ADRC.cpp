#include "adrc/ADRC.hpp"
#include "adrc/TD.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace
{
    std::vector<double> matmul(const std::vector<double> &a, const std::vector<double> &b, int n)
    {
        std::vector<double> c(n * n, 0.0);
        for (int i = 0; i < n; ++i)
            for (int k = 0; k < n; ++k)
                for (int j = 0; j < n; ++j)
                    c[i * n + j] += a[i * n + k] * b[k * n + j];
        return c;
    }
}

ADRC::ADRC(int systemOrder) : n(systemOrder), dT(0.01), NADRC(systemOrder + 1), b0(1.0),
                              omega_c(4.0), numObservers(1), uPrev(0.0), uMin(-std::numeric_limits<double>::infinity()),
                              uMax(std::numeric_limits<double>::infinity()), inputDelaySteps(0),
                              useTD(false), TD_obj(nullptr), isInitialized(false), explicitObserverGains(false)
{
    if (systemOrder < 1 || systemOrder > 10)
        throw std::invalid_argument("System order must be between 1 and 10");
    observerStates.assign(NADRC, 0.0);
    omega_o.assign(1, 40.0);
    estimateLayers.assign(n, 1);
    uHistory.assign(1, 0.0);
    TD_obj = new TD("euler", 0.01, {0.9});
}

ADRC::~ADRC() { delete TD_obj; }

void ADRC::initialize(double Tsettle_val, double kob_val, double b0_val,
                      double uMin_val, double uMax_val, double dT_val,
                      std::vector<double> XhatInit, double uInit, double Ke_val,
                      double inputDelay, std::string TD_method,
                      std::vector<double> TD_params, bool useCascaded_val)
{
    if (Tsettle_val <= 0 || kob_val <= 0 || Ke_val <= 0 || dT_val <= 0 || inputDelay < 0)
        throw std::invalid_argument("Invalid ADRC initialization parameter");
    initialize(2.0 * (n + 1) / Tsettle_val,
               {2.0 * (n + 1) * kob_val / Tsettle_val}, b0_val / Ke_val,
               uMin_val, uMax_val, dT_val, XhatInit, uInit, inputDelay,
               TD_method, TD_params, useCascaded_val, 1, {}, {}, {});
}

void ADRC::initialize(double omega_c_val, std::vector<double> omega_o_val,
                      double b0_val, double uMin_val, double uMax_val,
                      double dT_val, std::vector<double> XhatInit, double uInit,
                      double inputDelay, std::string TD_method,
                      std::vector<double> TD_params, bool useCascaded_val,
                      int numObservers_val, std::vector<int> estimateLayers_val,
                      std::vector<double> observerGains,
                      std::vector<double> controllerGains)
{
    if (omega_c_val <= 0 || dT_val <= 0 || inputDelay < 0 || numObservers_val < 1)
        throw std::invalid_argument("Invalid ADRC initialization parameter");
    if (omega_o_val.size() == 1)
        omega_o_val.assign(numObservers_val, omega_o_val[0]);
    if ((int)omega_o_val.size() != numObservers_val)
        throw std::invalid_argument("omega_o must be scalar or contain one value per observer");
    for (double value : omega_o_val)
        if (value <= 0)
            throw std::invalid_argument("omega_o must be positive");
    if (XhatInit.empty())
        XhatInit.assign(NADRC, 0.0);
    if ((int)XhatInit.size() != NADRC)
        throw std::invalid_argument("XhatInit has wrong length");
    if (estimateLayers_val.empty())
        estimateLayers_val.assign(n, numObservers_val);
    if ((int)estimateLayers_val.size() != n)
        throw std::invalid_argument("estimateLayers has wrong length");
    for (int layer : estimateLayers_val)
        if (layer < 1 || layer > numObservers_val)
            throw std::invalid_argument("estimateLayers is out of range");

    omega_c = omega_c_val;
    omega_o = omega_o_val;
    numObservers = numObservers_val;
    estimateLayers = estimateLayers_val;
    b0 = b0_val;
    dT = dT_val;
    uMin = uMin_val;
    uMax = uMax_val;
    useCascaded = useCascaded_val || numObservers > 1;

    observerStates.resize(NADRC * numObservers);
    for (int layer = 0; layer < numObservers; ++layer)
        std::copy(XhatInit.begin(), XhatInit.end(), observerStates.begin() + layer * NADRC);
    uPrev = uInit;
    inputDelaySteps = std::max(0, (int)std::round(inputDelay / dT));
    uHistory.assign(inputDelaySteps + 1, uInit);
    Ld.clear();
    explicitObserverGains = !observerGains.empty();

    computeESOGains();
    computeControllerGains();

    if (!observerGains.empty())
    {
        if ((int)observerGains.size() != NADRC * numObservers)
            throw std::invalid_argument("observerGains has wrong size");
        Ld = observerGains;
        computeESOGains();
    }
    if (!controllerGains.empty())
    {
        if ((int)controllerGains.size() != n)
            throw std::invalid_argument("controllerGains has wrong length");
        K = controllerGains;
    }
    delete TD_obj;
    std::transform(TD_method.begin(), TD_method.end(), TD_method.begin(), [](unsigned char c)
                   { return (char)std::tolower(c); });
    useTD = TD_method != "none";
    TD_obj = useTD ? new TD(TD_method, dT, TD_params) : new TD("euler", dT, {0.9});
    if (useTD)
        TD_obj->reset(0.0);
    isInitialized = true;
}

double ADRC::step(double reference, double output, std::vector<double> refDerivatives)
{
    if (!isInitialized)
        throw std::runtime_error("Controller must be initialized before use");

    std::vector<double> refs;
    double filtered = reference;

    if (useTD)
    {
        TD_obj->update(reference);
        filtered = TD_obj->y;
        refs.push_back(TD_obj->yd);
        if (n >= 2)
            refs.push_back(TD_obj->ydd);
    }

    if (!refDerivatives.empty())
        refs = refDerivatives;

    double delayed = uHistory[0];
    for (int layer = 0; layer < numObservers; ++layer)
    {
        std::vector<double> next(NADRC, 0.0);
        double input = layer == 0 ? output : observerStates[layer * NADRC - NADRC];
        for (int i = 0; i < NADRC; ++i)
        {
            double value = Ld[layer * NADRC + i] * input + Beso[layer * NADRC + i] * delayed;
            for (int j = 0; j < NADRC; ++j)
                value += Ad_data[layer * NADRC * NADRC + i * NADRC + j] * observerStates[layer * NADRC + j];
            if (layer > 0 && i == n - 1)
                for (int prior = 0; prior < layer; ++prior)
                    value += dT * observerStates[prior * NADRC + n];
            next[i] = value;
        }
        std::copy(next.begin(), next.end(), observerStates.begin() + layer * NADRC);
    }
    std::vector<double> r(n, 0.0);
    r[0] = filtered;
    for (int i = 0; i < std::min(n - 1, (int)refs.size()); ++i)
        r[i + 1] = refs[i];
    double nth = refs.size() >= (size_t)n ? refs[n - 1] : 0.0;
    double u = nth - getEstimatedDisturbance();

    for (int i = 0; i < n; ++i)
        u += K[i] * (r[i] - observerStates[(estimateLayers[i] - 1) * NADRC + i]);
    u = saturate(u / b0);
    std::rotate(uHistory.begin(), uHistory.begin() + 1, uHistory.end());

    uHistory.back() = u;
    uPrev = u;
    return u;
}

void ADRC::reset(std::vector<double> XhatInit, double uInit)
{
    if (XhatInit.empty())
        XhatInit.assign(NADRC, 0.0);
    if ((int)XhatInit.size() != NADRC)
        throw std::invalid_argument("XhatInit has wrong length");

    for (int layer = 0; layer < numObservers; ++layer)
        std::copy(XhatInit.begin(), XhatInit.end(), observerStates.begin() + layer * NADRC);

    uPrev = uInit;
    uHistory.assign(inputDelaySteps + 1, uInit);
    if (useTD)
        TD_obj->reset(0.0);
}

void ADRC::setTD(std::string method, std::vector<double> params)
{
    std::transform(method.begin(), method.end(), method.begin(), [](unsigned char c)
                   { return (char)std::tolower(c); });
    useTD = method != "none";
    delete TD_obj;
    TD_obj = useTD ? new TD(method, dT, params) : new TD("euler", dT, {0.9});
    if (useTD)
        TD_obj->reset(0.0);
}

void ADRC::setSaturation(double min, double max)
{
    uMin = min;
    uMax = max;
}

void ADRC::setInputDelay(double delay)
{
    inputDelaySteps = std::max(0, (int)std::round(delay / dT));
    uHistory.assign(inputDelaySteps + 1, uPrev);
}

void ADRC::updateTuning(double oc, double oo)
{
    omega_c = oc;
    omega_o.assign(numObservers, oo);

    computeESOGains();
    computeControllerGains();
}

std::vector<double> ADRC::getEstimatedStates()
{
    std::vector<double> result(n + 1);

    for (int i = 0; i < n; ++i)
        result[i] = observerStates[(estimateLayers[i] - 1) * NADRC + i];
    result[n] = getEstimatedDisturbance();
    return result;
}

double ADRC::getEstimatedDisturbance()
{
    double result = 0;
    for (int layer = 0; layer < numObservers; ++layer)
        result += observerStates[layer * NADRC + n];
    return result;
}

std::vector<double> ADRC::getObserverStates(int layer)
{
    if (layer < 1 || layer > numObservers)
    {
        if (layer == 0)
            return observerStates;

        throw std::invalid_argument("Observer layer is out of range");
    }
    return std::vector<double>(observerStates.begin() + (layer - 1) * NADRC, observerStates.begin() + layer * NADRC);
}

void ADRC::computeESOGains()
{
    if (Ld.empty())
        Ld.assign(NADRC * numObservers, 0.0);

    std::vector<double> A(NADRC * NADRC, 0), B(NADRC, 0), C(NADRC, 0);

    for (int i = 0; i < n; ++i)
        A[i * NADRC + i + 1] = 1;

    B[n - 1] = b0;
    C[0] = 1;

    std::vector<double> full;
    expm(A, full, dT);
    std::vector<double> aug((NADRC + 1) * (NADRC + 1), 0);

    for (int i = 0; i < NADRC; ++i)
        for (int j = 0; j < NADRC; ++j)
            aug[i * (NADRC + 1) + j] = A[i * NADRC + j];
    for (int i = 0; i < NADRC; ++i)
        aug[i * (NADRC + 1) + NADRC] = B[i];

    std::vector<double> discrete;
    expm(aug, discrete, dT);
    Bd.assign(NADRC, 0);
    for (int i = 0; i < NADRC; ++i)
        Bd[i] = discrete[i * (NADRC + 1) + NADRC];

    Ad_data.assign(numObservers * NADRC * NADRC, 0);
    Beso.assign(numObservers * NADRC, 0);

    for (int layer = 0; layer < numObservers; ++layer)
    {
        double z = std::exp(-omega_o[layer] * dT);
        std::vector<double> desired(NADRC + 1, 0);
        desired[0] = 1;

        for (int i = 0; i < NADRC; ++i)
        {
            std::vector<double> next = desired;
            for (int j = 0; j <= i; ++j)
                next[j + 1] -= z * desired[j];
            desired = next;
        }

        std::vector<double> dual(NADRC * NADRC, 0), cb(NADRC * NADRC, 0);
        for (int i = 0; i < NADRC; ++i)
        {
            cb[i * NADRC] = full[i];
            for (int j = 0; j < NADRC; ++j)
                dual[i * NADRC + j] = full[j * NADRC + i];
        }

        std::vector<double> ctrl(NADRC * NADRC, 0), power(NADRC * NADRC, 0);
        for (int i = 0; i < NADRC; ++i)
            power[i * NADRC + i] = 1;
        for (int col = 0; col < NADRC; ++col)
        {
            std::vector<double> v(NADRC);
            for (int i = 0; i < NADRC; ++i)
                for (int j = 0; j < NADRC; ++j)
                    v[i] += power[i * NADRC + j] * cb[j * NADRC + 0];
            for (int i = 0; i < NADRC; ++i)
                ctrl[i * NADRC + col] = v[i];
            power = matmul(power, dual, NADRC);
        }
        std::vector<double> poly = power;
        for (int term = 1; term <= NADRC; ++term)
        {
            std::vector<double> p(NADRC * NADRC, 0);

            for (int i = 0; i < NADRC; ++i)
                p[i * NADRC + i] = 1;
            for (int q = 0; q < NADRC - term; ++q)
                p = matmul(p, dual, NADRC);
            for (int i = 0; i < NADRC * NADRC; ++i)
                poly[i] += desired[term] * p[i];
        }
        std::vector<double> inv = matrixInverse(ctrl), gain(NADRC, 0);
        for (int j = 0; j < NADRC; ++j)
            for (int i = 0; i < NADRC; ++i)
                gain[j] += inv[(NADRC - 1) * NADRC + i] * poly[i * NADRC + j];
        if (explicitObserverGains)
            for (int i = 0; i < NADRC; ++i)
                gain[i] = Ld[layer * NADRC + i];
        for (int i = 0; i < NADRC; ++i)
        {
            Ld[layer * NADRC + i] = gain[i];
            for (int j = 0; j < NADRC; ++j)
            {
                Ad_data[layer * NADRC * NADRC + i * NADRC + j] = full[i * NADRC + j] - gain[i] * full[j];
            }
            Beso[layer * NADRC + i] = Bd[i] - gain[i] * Bd[0];
        }
    }
}

void ADRC::computeControllerGains()
{
    K.resize(n);
    for (int i = 1; i <= n; ++i)
    {
        int c = factorial(n) / (factorial(n - i) * factorial(i));
        K[n - i] = c * std::pow(omega_c, i);
    }
}

double ADRC::saturate(double u) { return !std::isfinite(uMin) ? std::min(u, uMax) : !std::isfinite(uMax) ? std::max(u, uMin)
                                                                                                         : std::max(uMin, std::min(u, uMax)); }
int ADRC::factorial(int x)
{
    int r = 1;
    for (int i = 2; i <= x; ++i)
        r *= i;
    return r;
}

std::vector<double> ADRC::matrixInverse(const std::vector<double> &input)
{
    int s = NADRC;
    std::vector<long double> a(input.size());
    for (size_t i = 0; i < input.size(); ++i)
        a[i] = static_cast<long double>(input[i]);
    std::vector<long double> inv(s * s, 0);
    for (int i = 0; i < s; ++i)
        inv[i * s + i] = 1;
    for (int i = 0; i < s; ++i)
    {
        int p = i;
        for (int j = i + 1; j < s; ++j)
            if (std::abs(a[j * s + i]) > std::abs(a[p * s + i]))
                p = j;
        if (std::abs(a[p * s + i]) < 1e-30L)
            throw std::runtime_error("Observer controllability matrix is singular");
        for (int j = 0; j < s; ++j)
        {
            std::swap(a[i * s + j], a[p * s + j]);
            std::swap(inv[i * s + j], inv[p * s + j]);
        }
        long double d = a[i * s + i];
        for (int j = 0; j < s; ++j)
        {
            a[i * s + j] /= d;
            inv[i * s + j] /= d;
        }
        for (int k = 0; k < s; ++k)
            if (k != i)
            {
                d = a[k * s + i];
                for (int j = 0; j < s; ++j)
                {
                    a[k * s + j] -= d * a[i * s + j];
                    inv[k * s + j] -= d * inv[i * s + j];
                }
            }
    }
    std::vector<double> result(inv.size());
    for (size_t i = 0; i < inv.size(); ++i)
        result[i] = static_cast<double>(inv[i]);
    return result;
}

void ADRC::expm(const std::vector<double> &A, std::vector<double> &result, double dt)
{
    int s = (int)std::sqrt((double)A.size());
    result.assign(s * s, 0);
    for (int i = 0; i < s; ++i)
        result[i * s + i] = 1;
    std::vector<double> term = result, scaled = A;
    for (double &v : scaled)
        v *= dt;
    double inverseFactorial = 1.0;
    for (int k = 1; k <= 20; ++k)
    {
        term = matmul(term, scaled, s);
        inverseFactorial /= k;
        for (int i = 0; i < s * s; ++i)
            result[i] += term[i] * inverseFactorial;
    }
}
