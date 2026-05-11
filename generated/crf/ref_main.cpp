#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_m_find.hpp"
#include "motion_m_loosen.hpp"

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
    kinova_state kinova_right_arm_solver_state{};
    kinova_right_arm_solver_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_right_arm_solver{};
    kinova_right_arm_solver.conf.ip_address = "127.0.0.1";
    kinova_right_arm_solver.conf.port = 10000;
    kinova_right_arm_solver.conf.port_real_time = 10001;
    kinova_right_arm_solver.conf.user = "admin";
    kinova_right_arm_solver.conf.password = "admin";
    kinova_right_arm_solver.conf.session_timeout = 60000;
    kinova_right_arm_solver.conf.connection_timeout = 2000;
    kinova_right_arm_solver.ctrl_mode = &kinova_right_arm_solver_state.ctrl_mode;
    kinova_right_arm_solver.jnt_pos_msr = &kinova_right_arm_solver_state.pos_msr[0];
    kinova_right_arm_solver.jnt_vel_msr = &kinova_right_arm_solver_state.vel_msr[0];
    kinova_right_arm_solver.jnt_trq_msr = &kinova_right_arm_solver_state.eff_msr[0];
    kinova_right_arm_solver.act_cur_msr = &kinova_right_arm_solver_state.cur_msr[0];
    kinova_right_arm_solver.jnt_pos_cmd = &kinova_right_arm_solver_state.pos_cmd[0];
    kinova_right_arm_solver.jnt_vel_cmd = &kinova_right_arm_solver_state.vel_cmd[0];
    kinova_right_arm_solver.jnt_trq_cmd = &kinova_right_arm_solver_state.eff_cmd[0];
    kinova_right_arm_solver.act_cur_cmd = &kinova_right_arm_solver_state.cur_cmd[0];
    kinova_right_arm_solver.success = &kinova_right_arm_solver_state.success;

    KDL::Chain chain_right_arm_solver;
    kinova_tree.getChain("base_link", "Bracelet_Link", chain_right_arm_solver);

    robot.right_arm_solver = manipulator_robot{
        .state = &kinova_right_arm_solver_state,
        .robot = &kinova_right_arm_solver,
        .chain = &chain_right_arm_solver,
    };

    KDL::Wrench wrench_ee_ee_measurement;
    KDL::Wrench wrench_ee_ee_measurement;
    robot.wrench_ee_ee = &wrench_ee_ee_measurement;
    robot.wrench_ee_ee = &wrench_ee_ee_measurement;
    motion_m_find_state motion_m_find_state_instance;
    motion_m_loosen_state motion_m_loosen_state_instance;
    robif2b_kinova_gen3_configure(&kinova_right_arm_solver);
    robif2b_kinova_gen3_recover(&kinova_right_arm_solver);
    robif2b_kinova_gen3_start(&kinova_right_arm_solver);


    int current_motion = 0;
    while (true) {
        switch (current_motion) {
        case 0: {
            update_motion_m_find(motion_m_find_state_instance, shared, robot);
            monitor_motion_m_find(motion_m_find_state_instance, shared);
            if (can_start_motion_m_find(motion_m_find_state_instance, shared)) {
                control_motion_m_find(motion_m_find_state_instance, shared, robot);
            }
            apply_motion_m_find(motion_m_find_state_instance, shared, robot);
            if (motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err) && motion_spec::runtime::constraint_satisfied(shared.pose_arm_ee_world_rotation_z_err) && motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err)) {
                current_motion = 1;
            }
            break;
        }

        case 1: {
            update_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
            monitor_motion_m_loosen(motion_m_loosen_state_instance, shared);
            if (can_start_motion_m_loosen(motion_m_loosen_state_instance, shared)) {
                control_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
            }
            apply_motion_m_loosen(motion_m_loosen_state_instance, shared, robot);
            if (motion_spec::runtime::constraint_satisfied(shared.pose_arm_ee_world_rotation_z_err) && motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err)) {
                current_motion = 2;
            }
            break;
        }

        default:
            current_motion = 2 - 1;
            break;
        }
        usleep(1000);
    }

    robif2b_kinova_gen3_stop(&kinova_right_arm_solver);
    robif2b_kinova_gen3_shutdown(&kinova_right_arm_solver);

    return 0;
}
