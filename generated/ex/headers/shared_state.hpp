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

struct arm_solver_solver_state {
    bool initialized = false;
    int num_constraints = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin> achd_acc;
};

struct robot_io {
    manipulator_robot arm_solver;
    KDL::Wrench *wrench_ee = nullptr;
};

struct shared_data {
    KDL::Vector direction_ctrl_frc_z = KDL::Vector(0.0, 0.0, 1.0);
    KDL::Vector position_force_ctrl_frc_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Twist twist_ee;
    KDL::Wrench wrench_ee;
    KDL::Wrench wrench_force_ctrl_frc_z;
    double frc_z_ref = 10.0;

    double q_j4;
    double q_j2_err;
    double tau_ctrl_limit_j4;
    double eacc_twist_ee_angular_z_m_contact;
    double frc_start = 1.0;
    double twist_ee_linear_y_err_m_approach;
    double frc_overload = 20.0;
    double eacc_twist_ee_linear_y_m_approach;
    double twist_ee_linear_z_err_m_approach;
    double eacc_twist_ee_angular_y_m_contact;
    double tau_ctrl_keep_j2;
    double vel_y_zero = 0.0;
    double gravity_vec;
    double vel_z_down = -0.05;
    double wrench_ee_force_z_err;
    double twist_ee_linear_x_err_m_approach;
    double twist_ee_angular_x_err_m_contact;
    double eacc_twist_ee_angular_x_m_contact;
    double wrench_ee_force_z_err_m_contact;
    double eacc_twist_ee_linear_z_m_approach;
    double eacc_twist_ee_linear_x_m_approach;
    double frc_contact_overload = 25.0;
    double q_j2_ref = 2.4;
    double frc_threshold = 5.0;
    double twist_ee_angular_y_err_m_contact;
    double angvel_zero = 0.0;
    double q_j4_lower = -0.8;
    double twist_ee_angular_z_err_m_contact;
    double q_j4_upper = 0.8;
    double force_ctrl_frc_z;
    double vel_zero = 0.0;
    double q_j4_err;
};
