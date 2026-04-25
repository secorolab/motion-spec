#include "runtime.hpp"
#include "shared_state.hpp"
#include "mobile_base_cycle.hpp"
#include "motion_rightarm.hpp"
#include "motion_leftarm.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <unistd.h>

int main() {
    robot_io robot{};
    shared_data shared{};

    int num_drives = NUM_DRIVES;
    ecat_comm ecat_comm{};
    ecat_state ethercat_state{};
    ethercat_state.ethernet_if = "net0";
    ethercat_state.num_exposed_slaves = NUM_SLAVES;
    ethercat_state.slave_idx[0] = 3;
    ethercat_state.slave_idx[1] = 4;
    ethercat_state.slave_idx[2] = 6;
    ethercat_state.slave_idx[3] = 7;

    for (int i = 0; i < NUM_DRIVES; ++i) {
        ethercat_state.name[i] = "KELOD105";
        ethercat_state.prod_code[i] = 0x02001001;
        ethercat_state.input_size[i] = sizeof(ecat_comm.drv_msr_pdo[i]);
        ethercat_state.output_size[i] = sizeof(ecat_comm.drv_cmd_pdo[i]);
    }

    void *input_array[NUM_SLAVES] = {
        &ecat_comm.drv_msr_pdo[0],
        &ecat_comm.drv_msr_pdo[1],
        &ecat_comm.drv_msr_pdo[2],
        &ecat_comm.drv_msr_pdo[3],
    };
    const void *output_array[NUM_SLAVES] = {
        &ecat_comm.drv_cmd_pdo[0],
        &ecat_comm.drv_cmd_pdo[1],
        &ecat_comm.drv_cmd_pdo[2],
        &ecat_comm.drv_cmd_pdo[3],
    };

    robif2b_ethercat ecat{};
    ecat.ethernet_if = &ethercat_state.ethernet_if[0];
    ecat.num_exposed_slaves = &ethercat_state.num_exposed_slaves;
    ecat.slave_idx = &ethercat_state.slave_idx[0];
    ecat.name = &ethercat_state.name[0];
    ecat.product_code = &ethercat_state.prod_code[0];
    ecat.input_size = &ethercat_state.input_size[0];
    ecat.output_size = &ethercat_state.output_size[0];
    ecat.error_code = &ethercat_state.error_code;
    ecat.num_initial_slaves = &ethercat_state.num_found_slaves;
    ecat.num_current_slaves = &ethercat_state.num_active_slaves;
    ecat.is_connected = &ethercat_state.is_connected[0];
    ecat.input = input_array;
    ecat.output = output_array;

    kelo_msr kelo_msr{};
    robif2b_kelo_drive_encoder drive_enc{};
    drive_enc.num_drives = &num_drives;
    drive_enc.msr_pdo = &ecat_comm.drv_msr_pdo[0];
    drive_enc.wheel_pos_msr = &kelo_msr.whl_pos[0];
    drive_enc.wheel_vel_msr = &kelo_msr.whl_vel[0];
    drive_enc.pivot_pos_msr = &kelo_msr.pvt_pos[0];
    drive_enc.pivot_vel_msr = &kelo_msr.pvt_vel[0];
    drive_enc.pivot_pos_off = &kelo_msr.pvt_off[0];

    kelo_cmd kelo_cmd{};
    for (int i = 0; i < NUM_DRIVES; ++i) {
        kelo_cmd.ctrl_mode[i] = ROBIF2B_CTRL_MODE_FORCE;
        kelo_cmd.max_current[i * 2 + 0] = 10.0;
        kelo_cmd.max_current[i * 2 + 1] = 10.0;
        kelo_cmd.trq_const[i * 2 + 0] = 0.29;
        kelo_cmd.trq_const[i * 2 + 1] = 0.29;
    }

    robif2b_kelo_drive_actuator wheel_act{
        .num_drives = &num_drives,
        .cmd_pdo = &ecat_comm.drv_cmd_pdo[0],
        .ctrl_mode = &kelo_cmd.ctrl_mode[0],
        .act_vel_cmd = &kelo_cmd.vel[0],
        .act_trq_cmd = &kelo_cmd.trq[0],
        .act_cur_cmd = &kelo_cmd.cur[0],
        .max_current = &kelo_cmd.max_current[0],
        .trq_const = &kelo_cmd.trq_const[0],
    };

    robot.mobile_base = mobile_base_robot{
        .num_drives = &num_drives,
        .ethercat_state = &ethercat_state,
        .ethercat_comm = &ecat_comm,
        .ethercat = &ecat,
        .measurement = &kelo_msr,
        .command = &kelo_cmd,
        .drive_encoder = &drive_enc,
        .wheel_actuator = &wheel_act,
    };

    urdf::ModelInterfaceSharedPtr kinova_model = urdf::parseURDFFile("../GEN3_URDF_V12.urdf");
    KDL::Tree kinova_tree;
    kdl_parser::treeFromUrdfModel(*kinova_model, kinova_tree);
    kinova_state kinova_slv_sc2_rightarm_state{};
    kinova_slv_sc2_rightarm_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_sc2_rightarm{};
    kinova_slv_sc2_rightarm.conf.ip_address = "192.168.1.11";
    kinova_slv_sc2_rightarm.conf.port = 10000;
    kinova_slv_sc2_rightarm.conf.port_real_time = 10001;
    kinova_slv_sc2_rightarm.conf.user = "admin";
    kinova_slv_sc2_rightarm.conf.password = "admin";
    kinova_slv_sc2_rightarm.conf.session_timeout = 60000;
    kinova_slv_sc2_rightarm.conf.connection_timeout = 2000;
    kinova_slv_sc2_rightarm.ctrl_mode = &kinova_slv_sc2_rightarm_state.ctrl_mode;
    kinova_slv_sc2_rightarm.jnt_pos_msr = &kinova_slv_sc2_rightarm_state.pos_msr[0];
    kinova_slv_sc2_rightarm.jnt_vel_msr = &kinova_slv_sc2_rightarm_state.vel_msr[0];
    kinova_slv_sc2_rightarm.jnt_trq_msr = &kinova_slv_sc2_rightarm_state.eff_msr[0];
    kinova_slv_sc2_rightarm.act_cur_msr = &kinova_slv_sc2_rightarm_state.cur_msr[0];
    kinova_slv_sc2_rightarm.jnt_pos_cmd = &kinova_slv_sc2_rightarm_state.pos_cmd[0];
    kinova_slv_sc2_rightarm.jnt_vel_cmd = &kinova_slv_sc2_rightarm_state.vel_cmd[0];
    kinova_slv_sc2_rightarm.jnt_trq_cmd = &kinova_slv_sc2_rightarm_state.eff_cmd[0];
    kinova_slv_sc2_rightarm.act_cur_cmd = &kinova_slv_sc2_rightarm_state.cur_cmd[0];
    kinova_slv_sc2_rightarm.success = &kinova_slv_sc2_rightarm_state.success;

    KDL::Chain chain_slv_sc2_rightarm;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_sc2_rightarm);

    robot.slv_sc2_rightarm = manipulator_robot{
        .state = &kinova_slv_sc2_rightarm_state,
        .robot = &kinova_slv_sc2_rightarm,
        .chain = &chain_slv_sc2_rightarm,
    };

    kinova_state kinova_slv_sc2_leftarm_state{};
    kinova_slv_sc2_leftarm_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_sc2_leftarm{};
    kinova_slv_sc2_leftarm.conf.ip_address = "192.168.1.10";
    kinova_slv_sc2_leftarm.conf.port = 10000;
    kinova_slv_sc2_leftarm.conf.port_real_time = 10001;
    kinova_slv_sc2_leftarm.conf.user = "admin";
    kinova_slv_sc2_leftarm.conf.password = "admin";
    kinova_slv_sc2_leftarm.conf.session_timeout = 60000;
    kinova_slv_sc2_leftarm.conf.connection_timeout = 2000;
    kinova_slv_sc2_leftarm.ctrl_mode = &kinova_slv_sc2_leftarm_state.ctrl_mode;
    kinova_slv_sc2_leftarm.jnt_pos_msr = &kinova_slv_sc2_leftarm_state.pos_msr[0];
    kinova_slv_sc2_leftarm.jnt_vel_msr = &kinova_slv_sc2_leftarm_state.vel_msr[0];
    kinova_slv_sc2_leftarm.jnt_trq_msr = &kinova_slv_sc2_leftarm_state.eff_msr[0];
    kinova_slv_sc2_leftarm.act_cur_msr = &kinova_slv_sc2_leftarm_state.cur_msr[0];
    kinova_slv_sc2_leftarm.jnt_pos_cmd = &kinova_slv_sc2_leftarm_state.pos_cmd[0];
    kinova_slv_sc2_leftarm.jnt_vel_cmd = &kinova_slv_sc2_leftarm_state.vel_cmd[0];
    kinova_slv_sc2_leftarm.jnt_trq_cmd = &kinova_slv_sc2_leftarm_state.eff_cmd[0];
    kinova_slv_sc2_leftarm.act_cur_cmd = &kinova_slv_sc2_leftarm_state.cur_cmd[0];
    kinova_slv_sc2_leftarm.success = &kinova_slv_sc2_leftarm_state.success;

    KDL::Chain chain_slv_sc2_leftarm;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_sc2_leftarm);

    robot.slv_sc2_leftarm = manipulator_robot{
        .state = &kinova_slv_sc2_leftarm_state,
        .robot = &kinova_slv_sc2_leftarm,
        .chain = &chain_slv_sc2_leftarm,
    };

    KDL::Wrench wrench_rightarm_ee_anteroposterior_ee_measurement;
    KDL::Wrench wrench_rightarm_ee_anteroposterior_platform_measurement;
    KDL::Wrench wrench_rightarm_elbow_measurement;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_ee_measurement;
    KDL::Wrench wrench_leftarm_ee_anteroposterior_platform_measurement;
    KDL::Wrench wrench_leftarm_elbow_measurement;
    KDL::Wrench wrench_rightarm_dist_shoulder_measurement;
    KDL::Wrench wrench_leftarm_dist_shoulder_measurement;
    KDL::Wrench wrench_rightarm_dist_platform_measurement;
    KDL::Wrench wrench_leftarm_dist_platform_measurement;
    KDL::Wrench wrench_dist_platform_measurement;
    robot.wrench_rightarm_ee_anteroposterior_ee = &wrench_rightarm_ee_anteroposterior_ee_measurement;
    robot.wrench_rightarm_ee_anteroposterior_platform = &wrench_rightarm_ee_anteroposterior_platform_measurement;
    robot.wrench_rightarm_elbow = &wrench_rightarm_elbow_measurement;
    robot.wrench_leftarm_ee_anteroposterior_ee = &wrench_leftarm_ee_anteroposterior_ee_measurement;
    robot.wrench_leftarm_ee_anteroposterior_platform = &wrench_leftarm_ee_anteroposterior_platform_measurement;
    robot.wrench_leftarm_elbow = &wrench_leftarm_elbow_measurement;
    robot.wrench_rightarm_dist_shoulder = &wrench_rightarm_dist_shoulder_measurement;
    robot.wrench_leftarm_dist_shoulder = &wrench_leftarm_dist_shoulder_measurement;
    robot.wrench_rightarm_dist_platform = &wrench_rightarm_dist_platform_measurement;
    robot.wrench_leftarm_dist_platform = &wrench_leftarm_dist_platform_measurement;
    robot.wrench_dist_platform = &wrench_dist_platform_measurement;
    motion_rightarm_state motion_rightarm_state_instance;
    motion_leftarm_state motion_leftarm_state_instance;
    mobile_base_state mobile_base_state_instance;

    robif2b_ethercat_configure(robot.mobile_base.ethercat);
    robif2b_ethercat_start(robot.mobile_base.ethercat);
    robif2b_kinova_gen3_configure(&kinova_slv_sc2_rightarm);
    robif2b_kinova_gen3_recover(&kinova_slv_sc2_rightarm);
    robif2b_kinova_gen3_start(&kinova_slv_sc2_rightarm);

    robif2b_kinova_gen3_configure(&kinova_slv_sc2_leftarm);
    robif2b_kinova_gen3_recover(&kinova_slv_sc2_leftarm);
    robif2b_kinova_gen3_start(&kinova_slv_sc2_leftarm);


    // Each motion runs its own control cycle.
    // Coordinate them using a state machine, FSM, or threads as needed.
    // --- motion_rightarm ---
    while (true) {
        update_motion_rightarm(motion_rightarm_state_instance, shared, robot);
        monitor_motion_rightarm(motion_rightarm_state_instance, shared);
        if (can_start_motion_rightarm(motion_rightarm_state_instance, shared)) {
            control_motion_rightarm(motion_rightarm_state_instance, shared, robot);
        }
        apply_motion_rightarm(motion_rightarm_state_instance, shared, robot);
        usleep(1000);
    }

    // --- motion_leftarm ---
    while (true) {
        update_motion_leftarm(motion_leftarm_state_instance, shared, robot);
        monitor_motion_leftarm(motion_leftarm_state_instance, shared);
        if (can_start_motion_leftarm(motion_leftarm_state_instance, shared)) {
            control_motion_leftarm(motion_leftarm_state_instance, shared, robot);
        }
        apply_motion_leftarm(motion_leftarm_state_instance, shared, robot);
        usleep(1000);
    }


    robif2b_kinova_gen3_stop(&kinova_slv_sc2_rightarm);
    robif2b_kinova_gen3_shutdown(&kinova_slv_sc2_rightarm);

    robif2b_kinova_gen3_stop(&kinova_slv_sc2_leftarm);
    robif2b_kinova_gen3_shutdown(&kinova_slv_sc2_leftarm);

    robif2b_kelo_drive_actuator_stop(robot.mobile_base.wheel_actuator);
    robif2b_ethercat_stop(robot.mobile_base.ethercat);
    robif2b_ethercat_shutdown(robot.mobile_base.ethercat);

    return 0;
}