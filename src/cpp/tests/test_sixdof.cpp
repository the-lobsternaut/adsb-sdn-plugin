/**
 * ADS-B Plugin — 6DOF Integration Tests
 */

#include "adsb/sixdof_core.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace sixdof;

void testBankingTurn() {
    State s;
    s.pos = {0, 0, -10000};
    s.vel = {250, 0, 0};
    s.mass = 75000;

    double bank = 30.0 * M_PI / 180.0;
    s.quat = qfromAxisAngle({1,0,0}, bank);
    s.omega = {0, 0, 0};

    InertiaTensor I = inertiaDiag(2e6, 4e6, 5e6);
    auto coastFn = [](const State&, double) -> ForcesTorques { return {}; };
    double dt = 0.01, t = 0;

    for (int i = 0; i < 3000; i++) { s = rk4Step(s, I, dt, t, coastFn); t += dt; }

    Vec3 euler = qtoEulerZYX(s.quat);
    assert(std::abs(euler[0] - bank) < 0.001);

    std::cout << "  Banking turn ✓ (bank=" << euler[0]*180/M_PI << "°)\n";
}

void testTurbulencePerturbation() {
    State s;
    s.quat = qidentity();
    s.omega = {0, 0, 0};
    s.mass = 50000;

    InertiaTensor I = inertiaDiag(1e6, 2e6, 3e6);
    double dt = 0.01, t = 0;

    auto turbFn = [](const State&, double time) -> ForcesTorques {
        ForcesTorques ft;
        ft.torque_body = {
            5000 * std::sin(2.0 * time),
            3000 * std::cos(3.0 * time),
            1000 * std::sin(1.5 * time)
        };
        return ft;
    };

    for (int i = 0; i < 5000; i++) { s = rk4Step(s, I, dt, t, turbFn); t += dt; }

    assert(std::abs(qnorm(s.quat) - 1.0) < 1e-6);
    double omegaMag = v3norm(s.omega);
    assert(omegaMag < 1.0);

    std::cout << "  Turbulence ✓ (|omega|=" << omegaMag << ")\n";
}

int main() {
    std::cout << "=== adsb 6DOF tests ===\n";
    testBankingTurn();
    testTurbulencePerturbation();
    std::cout << "All ADS-B 6DOF tests passed.\n";
    return 0;
}
