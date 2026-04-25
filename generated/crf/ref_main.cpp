#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_m_loosen.hpp"
#include "motion_m_find.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <unistd.h>

int main() {
    robot_io robot{};
    shared_data shared{};

    urdf::ModelInterfaceSharedPtr kinova_model = urdf::parseURDFFile("../GEN3_URDF_V12.urdf");
    KDL::Tree kinova_tree;
    kdl_parser::treeFromUrdfModel(*kinova_model, kinova_tree);
    kinova_state kinova_slv_m_loosen_state{};
    kinova_slv_m_loosen_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_m_loosen{};
    kinova_slv_m_loosen.conf.ip_address = "127.0.0.1";
    kinova_slv_m_loosen.conf.port = 10000;
    kinova_slv_m_loosen.conf.port_real_time = 10001;
    kinova_slv_m_loosen.conf.user = "admin";
    kinova_slv_m_loosen.conf.password = "admin";
    kinova_slv_m_loosen.conf.session_timeout = 60000;
    kinova_slv_m_loosen.conf.connection_timeout = 2000;
    kinova_slv_m_loosen.ctrl_mode = &kinova_slv_m_loosen_state.ctrl_mode;
    kinova_slv_m_loosen.jnt_pos_msr = &kinova_slv_m_loosen_state.pos_msr[0];
    kinova_slv_m_loosen.jnt_vel_msr = &kinova_slv_m_loosen_state.vel_msr[0];
    kinova_slv_m_loosen.jnt_trq_msr = &kinova_slv_m_loosen_state.eff_msr[0];
    kinova_slv_m_loosen.act_cur_msr = &kinova_slv_m_loosen_state.cur_msr[0];
    kinova_slv_m_loosen.jnt_pos_cmd = &kinova_slv_m_loosen_state.pos_cmd[0];
    kinova_slv_m_loosen.jnt_vel_cmd = &kinova_slv_m_loosen_state.vel_cmd[0];
    kinova_slv_m_loosen.jnt_trq_cmd = &kinova_slv_m_loosen_state.eff_cmd[0];
    kinova_slv_m_loosen.act_cur_cmd = &kinova_slv_m_loosen_state.cur_cmd[0];
    kinova_slv_m_loosen.success = &kinova_slv_m_loosen_state.success;

    KDL::Chain chain_slv_m_loosen;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_m_loosen);

    robot.slv_m_loosen = manipulator_robot{
        .state = &kinova_slv_m_loosen_state,
        .robot = &kinova_slv_m_loosen,
        .chain = &chain_slv_m_loosen,
    };

    kinova_state kinova_slv_m_find_state{};
    kinova_slv_m_find_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_m_find{};
    kinova_slv_m_find.conf.ip_address = "127.0.0.1";
    kinova_slv_m_find.conf.port = 10000;
    kinova_slv_m_find.conf.port_real_time = 10001;
    kinova_slv_m_find.conf.user = "admin";
    kinova_slv_m_find.conf.password = "admin";
    kinova_slv_m_find.conf.session_timeout = 60000;
    kinova_slv_m_find.conf.connection_timeout = 2000;
    kinova_slv_m_find.ctrl_mode = &kinova_slv_m_find_state.ctrl_mode;
    kinova_slv_m_find.jnt_pos_msr = &kinova_slv_m_find_state.pos_msr[0];
    kinova_slv_m_find.jnt_vel_msr = &kinova_slv_m_find_state.vel_msr[0];
    kinova_slv_m_find.jnt_trq_msr = &kinova_slv_m_find_state.eff_msr[0];
    kinova_slv_m_find.act_cur_msr = &kinova_slv_m_find_state.cur_msr[0];
    kinova_slv_m_find.jnt_pos_cmd = &kinova_slv_m_find_state.pos_cmd[0];
    kinova_slv_m_find.jnt_vel_cmd = &kinova_slv_m_find_state.vel_cmd[0];
    kinova_slv_m_find.jnt_trq_cmd = &kinova_slv_m_find_state.eff_cmd[0];
    kinova_slv_m_find.act_cur_cmd = &kinova_slv_m_find_state.cur_cmd[0];
    kinova_slv_m_find.success = &kinova_slv_m_find_state.success;

    KDL::Chain chain_slv_m_find;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_m_find);

    robot.slv_m_find = manipulator_robot{
        .state = &kinova_slv_m_find_state,
        .robot = &kinova_slv_m_find,
        .chain = &chain_slv_m_find,
    };

    KDL::Wrench wrench_ee_ee_measurement;
    robot.wrench_ee_ee = &wrench_ee_ee_measurement;
    motion_m_loosen_state motion_m_loosen_state_instance;
    motion_m_find_state motion_m_find_state_instance;
    robif2b_kinova_gen3_configure(&kinova_slv_m_loosen);
    robif2b_kinova_gen3_recover(&kinova_slv_m_loosen);
    robif2b_kinova_gen3_start(&kinova_slv_m_loosen);

    robif2b_kinova_gen3_configure(&kinova_slv_m_find);
    robif2b_kinova_gen3_recover(&kinova_slv_m_find);
    robif2b_kinova_gen3_start(&kinova_slv_m_find);


    // Each motion runs its own control cycle.
    // Coordinate them using a state machine, FSM, or threads as needed.
    // --- motion_m_loosen ---
    while (true) {
        update_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
        monitor_motion_m_loosen(motion_m_loosen_state_instance, shared);
        if (can_start_motion_m_loosen(motion_m_loosen_state_instance, shared)) {
            control_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
        }
        apply_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
        usleep(1000);
    }

    // --- motion_m_find ---
    while (true) {
        update_motion_m_find(motion_m_find_state_instance, shared, robot);
        monitor_motion_m_find(motion_m_find_state_instance, shared);
        if (can_start_motion_m_find(motion_m_find_state_instance, shared)) {
            control_motion_m_find(motion_m_find_state_instance, shared, robot);
        }
        apply_motion_m_find(motion_m_find_state_instance, shared, robot);
        usleep(1000);
    }


    robif2b_kinova_gen3_stop(&kinova_slv_m_loosen);
    robif2b_kinova_gen3_shutdown(&kinova_slv_m_loosen);

    robif2b_kinova_gen3_stop(&kinova_slv_m_find);
    robif2b_kinova_gen3_shutdown(&kinova_slv_m_find);

    return 0;
}