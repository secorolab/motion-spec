#pragma once

#include <cassert>
#include <cmath>
#include <iostream>
#include <kdl/chain.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
namespace motion_spec::runtime {

inline constexpr double kConstraintTolerance = 1e-9;
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
}  // namespace motion_spec::runtime