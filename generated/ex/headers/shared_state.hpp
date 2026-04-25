#pragma once

#include "runtime.hpp"

#include <memory>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainhdsolver_vereshchagin.hpp>
#include <robif2b/functions/kinova_gen3.h>
#include "chainhdsolver_vereshchagin_fext.hpp"

inline constexpr int KINOVA_NUM_JOINTS = 7;

struct kinova_state {
    bool success = false;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    double pos_msr[KINOVA_NUM_JOINTS] = {0.0};
    double vel_msr[KINOVA_NUM_JOINTS] = {0.0};
    double eff_msr[KINOVA_NUM_JOINTS] = {0.0};
    double cur_msr[KINOVA_NUM_JOINTS] = {0.0};
    double pos_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double vel_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double eff_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double cur_cmd[KINOVA_NUM_JOINTS] = {0.0};
};

struct manipulator_robot {
    kinova_state *state = nullptr;
    robif2b_kinova_gen3_nbx *robot = nullptr;
    KDL::Chain *chain = nullptr;
};

struct robot_io {
    manipulator_robot slv_m_contact;
    manipulator_robot slv_m_approach;
    KDL::Wrench *wrench_ee = nullptr;
};

struct shared_data {
    KDL::Twist twist_ee;
    KDL::Wrench wrench_ee;
    double frc_start = 1.0;
    double vel_zero = 0.0;
    double twist_ee.angular.z_err_m_contact;
    double frc_z_ref = 10.0;
    double q_j2;
    double tau_ctrl_limit_j4;
    double vel_y_zero = 0.0;
    double q_j4_lower = -0.8;
    double twist_ee.linear.z_err_m_approach;
    double angvel_zero = 0.0;
    double q_j4_err;
    double gravity_vec;
    double eacc_twist_ee.linear.z_m_approach;
    double twist_ee.angular.y_err_m_contact;
    double eacc_twist_ee.angular.z_m_contact;
    double eacc_twist_ee.linear.y_m_approach;
    double twist_ee.linear.y_err_m_approach;
    double q_j4;
    double wrench_ee.force.z_err;
    double eacc_twist_ee.linear.x_m_approach;
    double frc_contact_overload = 25.0;
    double eacc_twist_ee.angular.y_m_contact;
    double frc_threshold = 5.0;
    double q_j2_ref = 2.4;
    double q_j4;
    double frc_overload = 20.0;
    double q_j4_upper = 0.8;
    double tau_ctrl_keep_j2;
    double wrench_ee.force.z_err_m_contact;
    double twist_ee.linear.x_err_m_approach;
    double twist_ee.angular.x_err_m_contact;
    double q_j2_err;
    double q_j2;
    double eacc_twist_ee.angular.x_m_contact;
    double vel_z_down = -0.05;
};