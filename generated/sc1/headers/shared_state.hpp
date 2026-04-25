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
#include <robif2b/functions/ethercat.h>
#include <robif2b/functions/kelo_drive.h>

#include "chainhdsolver_vereshchagin_fext.hpp"

inline constexpr int KINOVA_NUM_JOINTS = 7;

struct ecat_state {
    const char *ethernet_if = nullptr;
    int error_code = 0;
    int num_exposed_slaves = 0;
    int num_found_slaves = 0;
    int num_active_slaves = 0;
    int slave_idx[NUM_SLAVES] = {0};
    const char *name[NUM_SLAVES] = {nullptr};
    unsigned int prod_code[NUM_SLAVES] = {0U};
    size_t input_size[NUM_SLAVES] = {0U};
    size_t output_size[NUM_SLAVES] = {0U};
    bool is_connected[NUM_SLAVES] = {false};
};

struct ecat_comm {
    robif2b_kelo_drive_api_msr_pdo drv_msr_pdo[NUM_DRIVES];
    robif2b_kelo_drive_api_cmd_pdo drv_cmd_pdo[NUM_DRIVES];
};

struct kelo_msr {
    double pvt_off[NUM_DRIVES] = {0.0};
    double pvt_pos[NUM_DRIVES] = {0.0};
    double pvt_vel[NUM_DRIVES] = {0.0};
    double whl_pos[NUM_DRIVES * 2] = {0.0};
    double whl_vel[NUM_DRIVES * 2] = {0.0};
    double imu_ang_vel[NUM_DRIVES * 3] = {0.0};
    double imu_lin_acc[NUM_DRIVES * 3] = {0.0};
    double bat_volt = 0.0;
    double bat_cur = 0.0;
    double bat_pwr = 0.0;
    int bat_lvl = 0;
};

struct kelo_cmd {
    enum robif2b_ctrl_mode ctrl_mode[NUM_DRIVES] = {ROBIF2B_CTRL_MODE_FORCE};
    double vel[NUM_DRIVES * 2] = {0.0};
    double trq[NUM_DRIVES * 2] = {0.0};
    double cur[NUM_DRIVES * 2] = {0.0};
    double max_current[NUM_DRIVES * 2] = {0.0};
    double trq_const[NUM_DRIVES * 2] = {0.0};
};

struct mobile_base_robot {
    int *num_drives = nullptr;
    ecat_state *ethercat_state = nullptr;
    ecat_comm *ethercat_comm = nullptr;
    robif2b_ethercat *ethercat = nullptr;
    kelo_msr *measurement = nullptr;
    kelo_cmd *command = nullptr;
    robif2b_kelo_drive_encoder *drive_encoder = nullptr;
    robif2b_kelo_drive_actuator *wheel_actuator = nullptr;
};

struct mobile_base_runtime_state {
    double drive_attachment[NUM_DRIVES * 2] = {
         0.195,  0.21,
        -0.195,  0.21,
        -0.195, -0.21,
         0.195, -0.21,
    };
    double wheel_diameter[NUM_DRIVES * 2] = {
        0.115, 0.115,
        0.115, 0.115,
        0.115, 0.115,
        0.115, 0.115,
    };
    double wheel_distance[NUM_DRIVES] = {0.0775, 0.0775, 0.0775, 0.0775};
    double castor_offset[NUM_DRIVES] = {0.01, 0.01, 0.01, 0.01};
    double g[NUM_DRIVES * NUM_G_COORD] = {0.0};
    double f_platform[NUM_PLTF_COORD] = {0.0, 0.0, 0.0};
    double w_platform[NUM_PLTF_COORD * NUM_PLTF_COORD] = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    };
    double w_drive[NUM_DRIVES * 4] = {
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
        1.0, 0.0, 0.0, 1.0,
    };
    double f_drive_ref[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double w_align[NUM_DRIVES * 2] = {
        1.0, 1.0,
        1.0, 1.0,
        1.0, 1.0,
        1.0, 1.0,
    };
    double f_drive[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double f_wheel[NUM_DRIVES * NUM_GND_COORD] = {0.0};
    double f_prim[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double f_scnd[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double xd_ground[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double xd_drive[NUM_DRIVES * NUM_DRV_COORD] = {0.0};
    double xd_platform[NUM_PLTF_COORD] = {0.0, 0.0, 0.0};
};

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
    mobile_base_robot mobile_base;
    manipulator_robot slv_sc1_rightarm;
    manipulator_robot slv_sc1_leftarm;
    KDL::Wrench *wrench_rightarm_ee_anteroposterior_ee = nullptr;
    KDL::Wrench *wrench_rightarm_ee_anteroposterior_platform = nullptr;
    KDL::Wrench *wrench_rightarm_elbow = nullptr;
    KDL::Wrench *wrench_leftarm_ee_anteroposterior_ee = nullptr;
    KDL::Wrench *wrench_leftarm_ee_anteroposterior_platform = nullptr;
    KDL::Wrench *wrench_leftarm_elbow = nullptr;
    KDL::Wrench *wrench_rightarm_dist_shoulder = nullptr;
    KDL::Wrench *wrench_leftarm_dist_shoulder = nullptr;
    KDL::Wrench *wrench_rightarm_dist_platform = nullptr;
    KDL::Wrench *wrench_leftarm_dist_platform = nullptr;
    KDL::Wrench *wrench_dist_platform = nullptr;
};

struct shared_data {
    mobile_base_runtime_state mobile_base;
    KDL::Vector dir_rightarm_to_leftarm_contact_ee;
    KDL::Vector dir_rightarm_to_leftarm_contact_shoulder;
    KDL::Vector dir_rightarm_to_leftarm_shoulder_shoulder;
    KDL::Vector dir_rightarm_shoulder_to_ee;
    KDL::Vector dir_leftarm_shoulder_to_ee;
    KDL::Vector pos_rightarm_shoulder_shoulder = KDL::Vector(0, 0, 0);
    KDL::Vector pos_leftarm_shoulder_shoulder = KDL::Vector(0, 0, 0);
    KDL::Frame pose_rightarm_platform_shoulder;
    KDL::Frame pose_rightarm_shoulder_elbow;
    KDL::Frame pose_rightarm_platform_elbow;
    KDL::Frame pose_rightarm_shoulder_ee;
    KDL::Frame pose_rightarm_platform_ee;
    KDL::Frame pose_leftarm_platform_shoulder;
    KDL::Frame pose_leftarm_shoulder_elbow;
    KDL::Frame pose_leftarm_platform_elbow;
    KDL::Frame pose_leftarm_shoulder_ee;
    KDL::Frame pose_leftarm_platform_ee;
    KDL::Frame pose_rightarm_to_leftarm_ee;
    KDL::Frame pose_rightarm_to_leftarm_shoulder;
    KDL::Twist twist_world_platform_platform;
    KDL::Twist twist_world_platform_rightarm_ee;
    KDL::Twist twist_rightarm_shoulder_ee_platform;
    KDL::Twist twist_rightarm_shoulder_ee_shoulder;
    KDL::Twist twist_world_platform_leftarm_ee;
    KDL::Twist twist_leftarm_shoulder_ee_platform;
    KDL::Twist twist_leftarm_shoulder_ee_shoulder;
    KDL::Wrench wrench_rightarm_ee_anteroposterior_ee;
    KDL::Wrench wrench_rightarm_ee_anteroposterior_platform;
    KDL::Wrench wrench_rightarm_elbow;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_ee;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_platform;
    KDL::Wrench wrench_leftarm_elbow;
    KDL::Wrench wrench_rightarm_dist_shoulder;
    KDL::Wrench wrench_leftarm_dist_shoulder;
    KDL::Wrench wrench_rightarm_dist_platform;
    KDL::Wrench wrench_leftarm_dist_platform;
    KDL::Wrench wrench_dist_platform;
    double frc_rightarm_ee_anteroposterior_ref = 20.0;
    double linvel_rightarm_shoulder_ee_vertical_ref = 0.0;
    double linvel_rightarm_shoulder_ee_vertical_err;
    double eacc_rightarm_shoulder_ee_lin_z;
    double pos_rightarm_platform_elbow_height_ref = 0.75;
    double pos_rightarm_platform_elbow_height_err;
    double angvel_rightarm_shoulder_ee_anteroposterior_ref = 0.0;
    double angvel_rightarm_shoulder_ee_anteroposterior_err;
    double eacc_rightarm_shoulder_ee_ang_x;
    double angvel_rightarm_shoulder_ee_lateral_ref = 0.0;
    double angvel_rightarm_shoulder_ee_lateral_err;
    double eacc_rightarm_shoulder_ee_ang_y;
    double linvel_rightarm_world_ee_lateral_ref = 0.0;
    double linvel_rightarm_world_ee_lateral_err;
    double eacc_rightarm_world_ee_lin_y;
    double frc_leftarm_ee_anteroposterior_ref = 20.0;
    double linvel_leftarm_shoulder_ee_vertical_ref = 0.0;
    double linvel_leftarm_shoulder_ee_vertical_err;
    double eacc_leftarm_shoulder_ee_lin_z;
    double pos_leftarm_platform_elbow_height_ref = 0.75;
    double pos_leftarm_platform_elbow_height_err;
    double angvel_leftarm_shoulder_ee_anteroposterior_ref = 0.0;
    double angvel_leftarm_shoulder_ee_anteroposterior_err;
    double eacc_leftarm_shoulder_ee_ang_x;
    double angvel_leftarm_shoulder_ee_lateral_ref = 0.0;
    double angvel_leftarm_shoulder_ee_lateral_err;
    double eacc_leftarm_shoulder_ee_ang_y;
    double linvel_leftarm_world_ee_lateral_ref = 0.0;
    double linvel_leftarm_world_ee_lateral_err;
    double eacc_leftarm_world_ee_lin_y;
    double ang_table_shoulders;
    double dist_rightarm_shoulder_ee;
    double dist_rightarm_shoulder_ee_lower = 0.68;
    double dist_rightarm_shoulder_ee_upper = 0.72;
    double dist_rightarm_shoulder_ee_err;
    double frc_rightarm_dist;
    double dist_leftarm_shoulder_ee;
    double dist_leftarm_shoulder_ee_lower = 0.68;
    double dist_leftarm_shoulder_ee_upper = 0.72;
    double dist_leftarm_shoulder_ee_err;
    double frc_leftarm_dist;
    double ang_rightarm_shoulder_ee_vertical;
    double ang_rightarm_shoulder_ee_vertical_ref = 0.0;
    double ang_rightarm_shoulder_ee_vertical_err;
    double eacc_rightarm_shoulder_ee_ang_z;
    double ang_leftarm_shoulder_ee_vertical;
    double ang_leftarm_shoulder_ee_vertical_ref = 0.0;
    double ang_leftarm_shoulder_ee_vertical_err;
    double eacc_leftarm_shoulder_ee_ang_z;
};