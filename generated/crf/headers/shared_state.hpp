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
    manipulator_robot slv_m_loosen;
    manipulator_robot slv_m_find;
    KDL::Wrench *wrench_ee_ee = nullptr;
};

struct shared_data {
    KDL::Frame pose_arm_ee_world;
    KDL::Twist twist_ee_ee;
    KDL::Wrench wrench_ee_ee;
    double eacc_twist_ee_ee.linear.y;
    double linvel_zero_inline_ref = 0.0;
    double torque_loosen_min_contact = 0.5;
    double twist_ee_ee.angular.y_err;
    double pose_arm_ee_world.rotation.z;
    double rotation_loosen_lower = 1.4835;
    double twist_ee_ee.angular.z_err_m_find;
    double twist_ee_ee.linear.y_err;
    double torque_find_shutoff = 4.0;
    double twist_ee_ee.linear.x_err;
    double twist_ee_ee.linear.z_err;
    double linvel_zero_ref = 0.0;
    double pose_arm_ee_world.rotation.z_err;
    double eacc_twist_ee_ee.angular.y;
    double gravity_vec;
    double eacc_twist_ee_ee.angular.z_m_find;
    double rotation_loosen_upper = 1.6581;
    double angvel_zero_ref = 0.0;
    double angvel_ee_z_ref_find = 1.0;
    double twist_ee_ee.angular.z_err_m_loosen;
    double torque_loosen_upper = 8.4;
    double angvel_ee_z_ref_loosen = -1.0;
    double rotation_find_limit = 2.0944;
    double eacc_twist_ee_ee.linear.z;
    double eacc_twist_ee_ee.angular.x;
    double twist_ee_ee.angular.x_err;
    double wrench_ee_ee.torque.z_err;
    double torque_find_upper_limit = 12.0;
    double eacc_twist_ee_ee.angular.z_m_loosen;
    double torque_loosen_lower = 0.05;
    double eacc_twist_ee_ee.linear.x;
};