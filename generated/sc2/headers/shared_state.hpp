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

struct right_arm_solver_solver_state {
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

struct left_arm_solver_solver_state {
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
    manipulator_robot right_arm_solver;
    manipulator_robot left_arm_solver;
    KDL::Wrench *wrench_leftarm_ee_anteroposterior_ee = nullptr;
    KDL::Wrench *wrench_rightarm_ee_anteroposterior_ee = nullptr;
};

struct shared_data {
    KDL::Vector direction_ctrl_dist_rightarm_shoulder_ee;
    KDL::Vector direction_ctrl_pos_leftarm_platform_elbow_height;
    KDL::Vector direction_ctrl_dist_leftarm_shoulder_ee;
    KDL::Vector direction_ctrl_pos_rightarm_platform_elbow_height;
    KDL::Vector position_force_ctrl_pos_rightarm_platform_elbow_height = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_dist_leftarm_shoulder_ee = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_pos_leftarm_platform_elbow_height = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_dist_rightarm_shoulder_ee = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Frame pose_leftarm_platform_ee;
    KDL::Frame pose_rightarm_shoulder_elbow;
    KDL::Frame pose_leftarm_platform_shoulder;
    KDL::Frame inverse_pose_rightarm_platform_shoulder;
    KDL::Frame inverse_pose_leftarm_platform_shoulder;
    KDL::Frame pose_leftarm_platform_elbow;
    KDL::Frame pose_rightarm_shoulder_ee;
    KDL::Frame pose_leftarm_shoulder_ee;
    KDL::Frame pose_rightarm_platform_ee;
    KDL::Frame pose_rightarm_platform_shoulder;
    KDL::Frame pose_leftarm_shoulder_elbow;
    KDL::Frame pose_rightarm_platform_elbow;
    KDL::Twist twist_rightarm_shoulder_ee_shoulder;
    KDL::Twist twist_rightarm_shoulder_ee_platform;
    KDL::Twist twist_leftarm_shoulder_ee_platform;
    KDL::Twist twist_world_platform_leftarm_ee;
    KDL::Twist twist_leftarm_shoulder_ee_shoulder;
    KDL::Twist twist_world_platform_rightarm_ee;
    KDL::Wrench wrench_force_ctrl_pos_rightarm_platform_elbow_height;
    KDL::Wrench wrench_leftarm_elbow;
    KDL::Wrench wrench_force_ctrl_pos_leftarm_platform_elbow_height;
    KDL::Wrench wrench_force_ctrl_dist_leftarm_shoulder_ee;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_ee;
    KDL::Wrench wrench_rightarm_ee_anteroposterior_platform;
    KDL::Wrench wrench_leftarm_dist_shoulder;
    KDL::Wrench wrench_rightarm_elbow;
    KDL::Wrench wrench_force_ctrl_dist_rightarm_shoulder_ee;
    KDL::Wrench wrench_rightarm_ee_anteroposterior_ee;
    KDL::Wrench wrench_rightarm_dist_shoulder;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_platform;
    double pos_leftarm_platform_elbow_height_ref = 0.75;
    double pose_leftarm_shoulder_ee_distance;
    double angvel_rightarm_shoulder_ee_anteroposterior_ref = 0.0;
    double eacc_twist_leftarm_shoulder_ee_shoulder_linear_z_motion_leftarm;
    double twist_leftarm_shoulder_ee_shoulder_angular_z_err_motion_leftarm;
    double twist_leftarm_shoulder_ee_shoulder_linear_z_err_motion_leftarm;
    double eacc_twist_leftarm_shoulder_ee_shoulder_angular_z_motion_leftarm;
    double linvel_rightarm_world_ee_lateral_ref = 0.0;
    double force_ctrl_dist_leftarm_shoulder_ee;
    double twist_rightarm_shoulder_ee_shoulder_angular_z_err_motion_rightarm;
    double twist_leftarm_shoulder_ee_shoulder_angular_x_err_motion_leftarm;
    double pose_rightarm_platform_elbow_distance_z_err_motion_rightarm;
    double twist_world_platform_rightarm_ee_linear_y_err_motion_rightarm;
    double angvel_leftarm_shoulder_ee_lateral_ref = 0.0;
    double angvel_leftarm_shoulder_ee_anteroposterior_ref = 0.0;
    double twist_world_platform_leftarm_ee_linear_y_err_motion_leftarm;
    double dist_leftarm_shoulder_ee_lower = 0.68;
    double gravity_vec;
    double eacc_twist_rightarm_shoulder_ee_shoulder_angular_y_motion_rightarm;
    double twist_rightarm_shoulder_ee_shoulder_angular_y_err_motion_rightarm;
    double frc_leftarm_ee_anteroposterior_ref = 20.0;
    double eacc_twist_rightarm_shoulder_ee_shoulder_linear_z_motion_rightarm;
    double pose_leftarm_platform_elbow_distance_z_err_motion_leftarm;
    double dist_leftarm_shoulder_ee_upper = 0.72;
    double eacc_twist_leftarm_shoulder_ee_shoulder_angular_x_motion_leftarm;
    double force_ctrl_pos_rightarm_platform_elbow_height;
    double eacc_twist_world_platform_rightarm_ee_linear_y_motion_rightarm;
    double linvel_leftarm_shoulder_ee_vertical_ref = 0.0;
    double pos_rightarm_platform_elbow_height_ref = 0.75;
    double pose_rightarm_shoulder_ee_distance_err_motion_rightarm;
    double eacc_twist_rightarm_shoulder_ee_shoulder_angular_x_motion_rightarm;
    double twist_rightarm_shoulder_ee_shoulder_angular_x_err_motion_rightarm;
    double linvel_leftarm_world_ee_lateral_ref = 0.0;
    double dist_rightarm_shoulder_ee_lower = 0.68;
    double force_ctrl_pos_leftarm_platform_elbow_height;
    double frc_rightarm_ee_anteroposterior_ref = 20.0;
    double twist_rightarm_shoulder_ee_shoulder_linear_z_err_motion_rightarm;
    double angvel_rightarm_shoulder_ee_lateral_ref = 0.0;
    double twist_leftarm_shoulder_ee_shoulder_angular_y_err_motion_leftarm;
    double angvel_leftarm_shoulder_ee_vertical_ref = 0.0;
    double pose_rightarm_shoulder_ee_distance;
    double eacc_twist_leftarm_shoulder_ee_shoulder_angular_y_motion_leftarm;
    double pose_leftarm_shoulder_ee_distance_err_motion_leftarm;
    double angvel_rightarm_shoulder_ee_vertical_ref = 0.0;
    double dist_rightarm_shoulder_ee_upper = 0.72;
    double linvel_rightarm_shoulder_ee_vertical_ref = 0.0;
    double force_ctrl_dist_rightarm_shoulder_ee;
    double eacc_twist_world_platform_leftarm_ee_linear_y_motion_leftarm;
    double eacc_twist_rightarm_shoulder_ee_shoulder_angular_z_motion_rightarm;
};
