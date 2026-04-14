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
    manipulator_robot slv_arm;
    KDL::Wrench *wrench_ee_ee = nullptr;
};

struct shared_data {
    KDL::Frame pose_ee_world;
    KDL::Twist twist_ee_ee;
    KDL::Wrench wrench_ee_ee;
    double angvel_zero_ref = 0.0;
    double angvel_ee_z_ref_find = 1.0;
    double torque_loosen_upper = 8.4;
    double linvel_ee_y_err;
    double rotation_loosen_upper = 1.6581;
    double angvel_ee_z_err_loosen;
    double cstr_torque_loosen_contact_err;
    double eacc_ee_lin_y;
    double torque_find_shutoff = 4.0;
    double torque_find_upper_limit = 12.0;
    double angvel_ee_z_ref_loosen = -1.0;
    double eacc_ee_ang_z_find;
    double cstr_torque_find_upper_limit_err;
    double linvel_zero_ref = 0.0;
    double angvel_ee_x_err;
    double eacc_ee_lin_x;
    double torque_loosen_lower = 0.05;
    double eacc_ee_ang_z_loosen;
    double cstr_rotation_find_limit_err;
    double rotation_loosen_lower = 1.4835;
    double eacc_ee_ang_y;
    double rotation_loosen;
    double rotation_find_limit = 2.0944;
    double cstr_torque_loosen_bilateral_err;
    double linvel_ee_x_err;
    double eacc_ee_ang_x;
    double linvel_ee_z_err;
    double angvel_ee_z_err_find;
    double cstr_torque_find_shutoff_err;
    double rotation_find;
    double cstr_rotation_loosen_bilateral_err;
    double angvel_ee_y_err;
    double eacc_ee_lin_z;
    double torque_loosen_min_contact = 0.5;
};