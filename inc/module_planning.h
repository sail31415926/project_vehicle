// ==== module_planning.h ====
#pragma once
#include <vector>
#include <string>

class ReferencePoint {
public:
    double X_ref, Y_ref, phi_ref, kappa_ref, v_ref;
    ReferencePoint(double x, double y, double phi, double kappa, double v)
        : X_ref(x), Y_ref(y), phi_ref(phi), kappa_ref(kappa), v_ref(v) {}
};

std::vector<ReferencePoint> module_planning(const std::string& csv_file);