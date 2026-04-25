#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_m_contact.hpp"
#include "motion_m_approach.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <unistd.h>

int main() {
    robot_io robot{};
    shared_data shared{};

    urdf::ModelInterfaceSharedPtr kinova_model = urdf::parseURDFFile("../robots/kg3.urdf");
    KDL::Tree kinova_tree;
    kdl_parser::treeFromUrdfModel(*kinova_model, kinova_tree);
    kinova_state kinova_slv_m_contact_state{};
    kinova_slv_m_contact_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_m_contact{};
    kinova_slv_m_contact.conf.ip_address = "127.0.0.1";
    kinova_slv_m_contact.conf.port = 10000;
    kinova_slv_m_contact.conf.port_real_time = 10001;
    kinova_slv_m_contact.conf.user = "admin";
    kinova_slv_m_contact.conf.password = "admin";
    kinova_slv_m_contact.conf.session_timeout = 60000;
    kinova_slv_m_contact.conf.connection_timeout = 2000;
    kinova_slv_m_contact.ctrl_mode = &kinova_slv_m_contact_state.ctrl_mode;
    kinova_slv_m_contact.jnt_pos_msr = &kinova_slv_m_contact_state.pos_msr[0];
    kinova_slv_m_contact.jnt_vel_msr = &kinova_slv_m_contact_state.vel_msr[0];
    kinova_slv_m_contact.jnt_trq_msr = &kinova_slv_m_contact_state.eff_msr[0];
    kinova_slv_m_contact.act_cur_msr = &kinova_slv_m_contact_state.cur_msr[0];
    kinova_slv_m_contact.jnt_pos_cmd = &kinova_slv_m_contact_state.pos_cmd[0];
    kinova_slv_m_contact.jnt_vel_cmd = &kinova_slv_m_contact_state.vel_cmd[0];
    kinova_slv_m_contact.jnt_trq_cmd = &kinova_slv_m_contact_state.eff_cmd[0];
    kinova_slv_m_contact.act_cur_cmd = &kinova_slv_m_contact_state.cur_cmd[0];
    kinova_slv_m_contact.success = &kinova_slv_m_contact_state.success;

    KDL::Chain chain_slv_m_contact;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_m_contact);

    robot.slv_m_contact = manipulator_robot{
        .state = &kinova_slv_m_contact_state,
        .robot = &kinova_slv_m_contact,
        .chain = &chain_slv_m_contact,
    };

    kinova_state kinova_slv_m_approach_state{};
    kinova_slv_m_approach_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_slv_m_approach{};
    kinova_slv_m_approach.conf.ip_address = "127.0.0.1";
    kinova_slv_m_approach.conf.port = 10000;
    kinova_slv_m_approach.conf.port_real_time = 10001;
    kinova_slv_m_approach.conf.user = "admin";
    kinova_slv_m_approach.conf.password = "admin";
    kinova_slv_m_approach.conf.session_timeout = 60000;
    kinova_slv_m_approach.conf.connection_timeout = 2000;
    kinova_slv_m_approach.ctrl_mode = &kinova_slv_m_approach_state.ctrl_mode;
    kinova_slv_m_approach.jnt_pos_msr = &kinova_slv_m_approach_state.pos_msr[0];
    kinova_slv_m_approach.jnt_vel_msr = &kinova_slv_m_approach_state.vel_msr[0];
    kinova_slv_m_approach.jnt_trq_msr = &kinova_slv_m_approach_state.eff_msr[0];
    kinova_slv_m_approach.act_cur_msr = &kinova_slv_m_approach_state.cur_msr[0];
    kinova_slv_m_approach.jnt_pos_cmd = &kinova_slv_m_approach_state.pos_cmd[0];
    kinova_slv_m_approach.jnt_vel_cmd = &kinova_slv_m_approach_state.vel_cmd[0];
    kinova_slv_m_approach.jnt_trq_cmd = &kinova_slv_m_approach_state.eff_cmd[0];
    kinova_slv_m_approach.act_cur_cmd = &kinova_slv_m_approach_state.cur_cmd[0];
    kinova_slv_m_approach.success = &kinova_slv_m_approach_state.success;

    KDL::Chain chain_slv_m_approach;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_slv_m_approach);

    robot.slv_m_approach = manipulator_robot{
        .state = &kinova_slv_m_approach_state,
        .robot = &kinova_slv_m_approach,
        .chain = &chain_slv_m_approach,
    };

    KDL::Wrench wrench_ee_measurement;
    robot.wrench_ee = &wrench_ee_measurement;
    motion_m_contact_state motion_m_contact_state_instance;
    motion_m_approach_state motion_m_approach_state_instance;
    robif2b_kinova_gen3_configure(&kinova_slv_m_contact);
    robif2b_kinova_gen3_recover(&kinova_slv_m_contact);
    robif2b_kinova_gen3_start(&kinova_slv_m_contact);

    robif2b_kinova_gen3_configure(&kinova_slv_m_approach);
    robif2b_kinova_gen3_recover(&kinova_slv_m_approach);
    robif2b_kinova_gen3_start(&kinova_slv_m_approach);


    // Each motion runs its own control cycle.
    // Coordinate them using a state machine, FSM, or threads as needed.
    // --- motion_m_contact ---
    while (true) {
        update_motion_m_contact(motion_m_contact_state_instance, shared, robot);
        monitor_motion_m_contact(motion_m_contact_state_instance, shared);
        if (can_start_motion_m_contact(motion_m_contact_state_instance, shared)) {
            control_motion_m_contact(motion_m_contact_state_instance, shared, robot);
        }
        apply_motion_m_contact(motion_m_contact_state_instance, shared, robot);
        usleep(1000);
    }

    // --- motion_m_approach ---
    while (true) {
        update_motion_m_approach(motion_m_approach_state_instance, shared, robot);
        monitor_motion_m_approach(motion_m_approach_state_instance, shared);
        if (can_start_motion_m_approach(motion_m_approach_state_instance, shared)) {
            control_motion_m_approach(motion_m_approach_state_instance, shared, robot);
        }
        apply_motion_m_approach(motion_m_approach_state_instance, shared, robot);
        usleep(1000);
    }


    robif2b_kinova_gen3_stop(&kinova_slv_m_contact);
    robif2b_kinova_gen3_shutdown(&kinova_slv_m_contact);

    robif2b_kinova_gen3_stop(&kinova_slv_m_approach);
    robif2b_kinova_gen3_shutdown(&kinova_slv_m_approach);

    return 0;
}