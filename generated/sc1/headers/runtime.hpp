#pragma once

#include <cassert>
#include <cmath>
#include <iostream>
#include <kdl/chain.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <hddc2b/functions/platform.h>
#include <hddc2b/functions/solver.h>
#include <hddc2b/functions/drive.h>
#include <hddc2b/functions/wheel.h>

namespace motion_spec::runtime {

inline constexpr double kConstraintTolerance = 1e-9;
inline constexpr int NUM_DRIVES = 4;
inline constexpr int NUM_SLAVES = 4;
inline constexpr int NUM_WHL_COORD = 2;
inline constexpr int NUM_GND_COORD = 2;
inline constexpr int NUM_DRV_COORD = 2;
inline constexpr int NUM_PLTF_COORD = 3;
inline constexpr int NUM_G_COORD = NUM_DRV_COORD * NUM_PLTF_COORD;
inline constexpr double EPS = 0.001;

class PIDControl {
  public:
    PIDControl() = default;

    PIDControl(double p_gain, double i_gain, double d_gain, double decay_rate = 0.0)
        : kp(p_gain), ki(i_gain), kd(d_gain), decay_rate(decay_rate) {}

    double control(double error) {
        double err_diff = error - err_last;
        err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
        err_last = error;
        return kp * error + ki * err_integ + kd * err_diff;
    }

  private:
    double err_integ = 0.0;
    double err_last = 0.0;
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double decay_rate = 0.0;
};

inline double evaluate_equality_constraint(double quantity, double reference) {
    return quantity - reference;
}

inline double evaluate_less_than_constraint(double quantity, double threshold) {
    return (quantity < threshold) ? 0.0 : threshold - quantity;
}

inline double evaluate_greater_than_constraint(double quantity, double threshold) {
    return (quantity > threshold) ? 0.0 : quantity - threshold;
}

inline double evaluate_bilateral_constraint(double quantity, double lower, double upper) {
    if (quantity < lower) return lower - quantity;
    if (quantity > upper) return quantity - upper;
    return 0.0;
}

inline bool constraint_satisfied(double error) {
    return std::fabs(error) <= kConstraintTolerance;
}

enum class Axis {
    X = 0,
    Y = 1,
    Z = 2,
};

enum class Subspace {
    Linear = 0,
    Angular = 1,
};

inline int constraint_row(Subspace subspace, Axis axis) {
    return static_cast<int>(subspace) * 3 + static_cast<int>(axis);
}

inline bool rising_edge(bool &previous, bool active) {
    const bool detected = active && !previous;
    previous = active;
    return detected;
}

inline void set_flag(bool &flag, bool active) {
    flag = active;
}

inline void warn_produce_event_not_implemented(std::string_view event_id) {
    std::cerr << "produce_event(" << event_id << ") not implemented yet\n";
}

inline unsigned int find_segment_index(const KDL::Chain &chain, std::string_view model_name) {
    std::string target(model_name);
    if (target.rfind("frame_", 0) == 0) {
        target.replace(0, 6, "link_");
    }
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        if (chain.getSegment(i).getName() == target) {
            return i + 1;
        }
    }
    throw std::runtime_error("KDL segment not found for model name: " + target);
}
inline void hddc2b_pltf_vel_drv_to_pltf(
        int num_drv,
        double eps,
        const double *g,
        const double *w_drv_sqrt,
        const double *xd_drv,
        const double *w_pltf_inv_sqrt,
        double *xd_pltf)
{
    assert(num_drv >= 0);

    double g2[num_drv * NUM_G_COORD];
    double xd_drv2[num_drv * NUM_DRV_COORD];
    hddc2b_pltf_vel_sing_wgh(num_drv, g, xd_drv, w_drv_sqrt, g2, xd_drv2);

    double g3[num_drv * NUM_G_COORD];
    hddc2b_pltf_vel_redu_wgh_init(num_drv, g2, w_pltf_inv_sqrt, g3);

    double u[NUM_PLTF_COORD * NUM_PLTF_COORD];
    double s[NUM_PLTF_COORD];
    double vt[num_drv * NUM_G_COORD];
    hddc2b_pltf_dcmp(num_drv, g3, u, s, vt);

    double s_inv[NUM_PLTF_COORD];
    hddc2b_pltf_pinv(num_drv, eps, s, s_inv);

    double xd_pltf2[NUM_PLTF_COORD];
    hddc2b_pltf_vel_slv(num_drv, u, s_inv, vt, xd_drv2, xd_pltf2);

    hddc2b_pltf_vel_redu_wgh_fini(num_drv, xd_pltf2, w_pltf_inv_sqrt, xd_pltf);
}

inline void hddc2b_rescale(double *f_scnd) {
    double f_scnd_max = 0.0;
    for (int i = 0; i < NUM_DRIVES; ++i) {
        double current = fabs(f_scnd[i * NUM_DRV_COORD + 1]);
        if (current > f_scnd_max) f_scnd_max = current;
    }

    double scale_factor = (f_scnd_max == 0.0) ? 1.0 : f_scnd_max;

    for (int i = 0; i < NUM_DRIVES; ++i) {
        f_scnd[i * NUM_DRV_COORD + 0] /= scale_factor;
        f_scnd[i * NUM_DRV_COORD + 1] /= scale_factor;
    }
}

}  // namespace motion_spec::runtime