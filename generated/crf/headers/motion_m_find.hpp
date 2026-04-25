#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct slv_m_loosen_solver_state {
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
    KDL::Wrenches f_ext;
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin> achd_acc;
};

struct slv_m_find_solver_state {
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
    KDL::Wrenches f_ext;
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin> achd_acc;
};

struct motion_m_find_state {
    slv_m_loosen_solver_state slv_m_loosen;
    slv_m_find_solver_state slv_m_find;
    motion_spec::runtime::PIDControl ctrl_angvel_find{8.0, 0.5, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_x_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_y_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_z_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_x_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_y_zero{5.0, 1.0, 3.0};
    bool mon_torque_find_shutoff_previous = false;
    bool mon_rotation_find_limit_previous = false;
    bool mon_torque_find_upper_limit_previous = false;
};

inline void reset_motion_m_find(motion_m_find_state &state) {
    state = motion_m_find_state{};
}

inline void init_motion_m_find(motion_m_find_state &state, const robot_io &robot) {
    if (!state.slv_m_loosen.initialized) {
        state.slv_m_loosen.num_constraints = 6;
        state.slv_m_loosen.num_joints = robot.slv_m_loosen.chain->getNrOfJoints();
        state.slv_m_loosen.num_segments = robot.slv_m_loosen.chain->getNrOfSegments();
        state.slv_m_loosen.q = KDL::JntArray(state.slv_m_loosen.num_joints);
        state.slv_m_loosen.qd = KDL::JntArray(state.slv_m_loosen.num_joints);
        state.slv_m_loosen.qdd = KDL::JntArray(state.slv_m_loosen.num_joints);
        state.slv_m_loosen.tau_ff = KDL::JntArray(state.slv_m_loosen.num_joints);
        state.slv_m_loosen.tau_ctrl = KDL::JntArray(state.slv_m_loosen.num_joints);
        state.slv_m_loosen.f_ext = KDL::Wrenches(state.slv_m_loosen.num_segments);
        state.slv_m_loosen.f_cstr = KDL::Jacobian(state.slv_m_loosen.num_constraints);
        state.slv_m_loosen.e_acc = KDL::JntArray(state.slv_m_loosen.num_constraints);
        state.slv_m_loosen.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_m_loosen.chain, state.slv_m_loosen.root_acc, state.slv_m_loosen.num_constraints);
        state.slv_m_loosen.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_m_loosen.chain, state.slv_m_loosen.root_acc, state.slv_m_loosen.num_constraints);
        state.slv_m_loosen.initialized = true;
}
    if (!state.slv_m_find.initialized) {
        state.slv_m_find.num_constraints = 6;
        state.slv_m_find.num_joints = robot.slv_m_find.chain->getNrOfJoints();
        state.slv_m_find.num_segments = robot.slv_m_find.chain->getNrOfSegments();
        state.slv_m_find.q = KDL::JntArray(state.slv_m_find.num_joints);
        state.slv_m_find.qd = KDL::JntArray(state.slv_m_find.num_joints);
        state.slv_m_find.qdd = KDL::JntArray(state.slv_m_find.num_joints);
        state.slv_m_find.tau_ff = KDL::JntArray(state.slv_m_find.num_joints);
        state.slv_m_find.tau_ctrl = KDL::JntArray(state.slv_m_find.num_joints);
        state.slv_m_find.f_ext = KDL::Wrenches(state.slv_m_find.num_segments);
        state.slv_m_find.f_cstr = KDL::Jacobian(state.slv_m_find.num_constraints);
        state.slv_m_find.e_acc = KDL::JntArray(state.slv_m_find.num_constraints);
        state.slv_m_find.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_m_find.chain, state.slv_m_find.root_acc, state.slv_m_find.num_constraints);
        state.slv_m_find.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_m_find.chain, state.slv_m_find.root_acc, state.slv_m_find.num_constraints);
        state.slv_m_find.initialized = true;
}
}

inline void update_motion_m_find(
    motion_m_find_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_m_find(state, robot);
    if (robot.wrench_ee_ee != nullptr) {
        shared.wrench_ee_ee = *robot.wrench_ee_ee;
    }

    for (int i = 0; i < state.slv_m_loosen.num_joints; ++i) {
        state.slv_m_loosen.q(i) = robot.slv_m_loosen.state->pos_msr[i];
        state.slv_m_loosen.qd(i) = robot.slv_m_loosen.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_m_loosen(state.slv_m_loosen.q, state.slv_m_loosen.qd);


    for (int i = 0; i < state.slv_m_find.num_joints; ++i) {
        state.slv_m_find.q(i) = robot.slv_m_find.state->pos_msr[i];
        state.slv_m_find.qd(i) = robot.slv_m_find.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_m_find(state.slv_m_find.q, state.slv_m_find.qd);


}

inline bool can_start_motion_m_find(
    motion_m_find_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_m_find(
    motion_m_find_state &state,
    shared_data &shared) {
    // compute_pose_arm_ee_world.rotation.z
    KDL::Vector _pose_arm_ee_world.rotation.z = KDL::Vector;
    shared.pose_arm_ee_world.rotation.z = shared.pose_arm_ee_world.M.GetRotAngle(_pose_arm_ee_world.rotation.z);
    // eval_m_find_until_cstr_torque_find_shutoff
    shared.wrench_ee_ee.torque.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee_ee.torque[2], shared.torque_find_shutoff);
    // eval_m_find_until_cstr_rotation_find_limit
    shared.pose_arm_ee_world.rotation.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.pose_arm_ee_world.rotation.z, shared.rotation_find_limit);
    // eval_m_find_until_cstr_torque_find_upper_limit
    shared.wrench_ee_ee.torque.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee_ee.torque[2], shared.torque_find_upper_limit);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee.torque.z_err);
        if (motion_spec::runtime::rising_edge(state.mon_torque_find_shutoff_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_torque_find_shutoff");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.pose_arm_ee_world.rotation.z_err);
        if (motion_spec::runtime::rising_edge(state.mon_rotation_find_limit_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_rotation_find_limit");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee.torque.z_err);
        if (motion_spec::runtime::rising_edge(state.mon_torque_find_upper_limit_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_torque_find_upper_limit");
        }
    }

}

inline void control_motion_m_find(
    motion_m_find_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_m_find_while_cstr_angvel_find
    shared.twist_ee_ee.angular.z_err_m_find = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[2], shared.angvel_ee_z_ref_find);
    // eval_m_find_while_cstr_linvel_ee_x_zero
    shared.twist_ee_ee.linear.x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[0], shared.linvel_zero_ref);
    // eval_m_find_while_cstr_linvel_ee_y_zero
    shared.twist_ee_ee.linear.y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[1], shared.linvel_zero_inline_ref);
    // eval_m_find_while_cstr_linvel_ee_z_zero
    shared.twist_ee_ee.linear.z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[2], shared.linvel_zero_ref);
    // eval_m_find_while_cstr_angvel_ee_x_zero
    shared.twist_ee_ee.angular.x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[0], shared.angvel_zero_ref);
    // eval_m_find_while_cstr_angvel_ee_y_zero
    shared.twist_ee_ee.angular.y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[1], shared.angvel_zero_ref);
    // ctrl_angvel_ee_y_zero
    shared.eacc_twist_ee_ee.angular.y = state.ctrl_angvel_ee_y_zero.control(shared.twist_ee_ee.angular.y_err);
    // ctrl_angvel_ee_x_zero
    shared.eacc_twist_ee_ee.angular.x = state.ctrl_angvel_ee_x_zero.control(shared.twist_ee_ee.angular.x_err);
    // ctrl_linvel_ee_z_zero
    shared.eacc_twist_ee_ee.linear.z = state.ctrl_linvel_ee_z_zero.control(shared.twist_ee_ee.linear.z_err);
    // ctrl_linvel_ee_y_zero
    shared.eacc_twist_ee_ee.linear.y = state.ctrl_linvel_ee_y_zero.control(shared.twist_ee_ee.linear.y_err);
    // ctrl_linvel_ee_x_zero
    shared.eacc_twist_ee_ee.linear.x = state.ctrl_linvel_ee_x_zero.control(shared.twist_ee_ee.linear.x_err);
    // ctrl_angvel_find
    shared.eacc_twist_ee_ee.angular.z_m_find = state.ctrl_angvel_find.control(shared.twist_ee_ee.angular.z_err_m_find);



    KDL::SetToZero(state.slv_m_loosen.f_cstr);
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 0) = 1.0;
state.slv_m_loosen.e_acc(0) = shared.eacc_twist_ee_ee.angular.x;
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 1) = 1.0;
state.slv_m_loosen.e_acc(1) = shared.eacc_twist_ee_ee.angular.y;
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 2) = 1.0;
state.slv_m_loosen.e_acc(2) = shared.eacc_twist_ee_ee.angular.z_m_loosen;
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 3) = 1.0;
state.slv_m_loosen.e_acc(3) = shared.eacc_twist_ee_ee.linear.x;
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 4) = 1.0;
state.slv_m_loosen.e_acc(4) = shared.eacc_twist_ee_ee.linear.y;
    state.slv_m_loosen.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 5) = 1.0;
state.slv_m_loosen.e_acc(5) = shared.eacc_twist_ee_ee.linear.z;
    for (int i = 0; i < state.slv_m_loosen.num_segments; ++i) {
        KDL::SetToZero(state.slv_m_loosen.f_ext[i]);
    }
    KDL::JntArray tau_ctrl_fext_slv_m_loosen(state.slv_m_loosen.num_joints);
    state.slv_m_loosen.achd_fext->CartToJnt(
        state.slv_m_loosen.q,
        state.slv_m_loosen.qd,
        state.slv_m_loosen.qdd,
        state.slv_m_loosen.f_cstr,
        state.slv_m_loosen.e_acc,
        state.slv_m_loosen.f_ext,
        state.slv_m_loosen.tau_ff,
        tau_ctrl_fext_slv_m_loosen);
    KDL::Wrenches f_ext_zero_slv_m_loosen(state.slv_m_loosen.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_m_loosen(state.slv_m_loosen.num_joints);
    state.slv_m_loosen.achd_acc->CartToJnt(
        state.slv_m_loosen.q,
        state.slv_m_loosen.qd,
        state.slv_m_loosen.qdd,
        state.slv_m_loosen.f_cstr,
        state.slv_m_loosen.e_acc,
        f_ext_zero_slv_m_loosen,
        state.slv_m_loosen.tau_ff,
        tau_ctrl_acc_slv_m_loosen);
    KDL::Add(tau_ctrl_fext_slv_m_loosen, tau_ctrl_acc_slv_m_loosen, state.slv_m_loosen.tau_ctrl);
    KDL::SetToZero(state.slv_m_find.f_cstr);
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 0) = 1.0;
state.slv_m_find.e_acc(0) = shared.eacc_twist_ee_ee.angular.x;
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 1) = 1.0;
state.slv_m_find.e_acc(1) = shared.eacc_twist_ee_ee.angular.y;
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 2) = 1.0;
state.slv_m_find.e_acc(2) = shared.eacc_twist_ee_ee.angular.z_m_find;
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 3) = 1.0;
state.slv_m_find.e_acc(3) = shared.eacc_twist_ee_ee.linear.x;
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 4) = 1.0;
state.slv_m_find.e_acc(4) = shared.eacc_twist_ee_ee.linear.y;
    state.slv_m_find.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 5) = 1.0;
state.slv_m_find.e_acc(5) = shared.eacc_twist_ee_ee.linear.z;
    for (int i = 0; i < state.slv_m_find.num_segments; ++i) {
        KDL::SetToZero(state.slv_m_find.f_ext[i]);
    }
    KDL::JntArray tau_ctrl_fext_slv_m_find(state.slv_m_find.num_joints);
    state.slv_m_find.achd_fext->CartToJnt(
        state.slv_m_find.q,
        state.slv_m_find.qd,
        state.slv_m_find.qdd,
        state.slv_m_find.f_cstr,
        state.slv_m_find.e_acc,
        state.slv_m_find.f_ext,
        state.slv_m_find.tau_ff,
        tau_ctrl_fext_slv_m_find);
    KDL::Wrenches f_ext_zero_slv_m_find(state.slv_m_find.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_m_find(state.slv_m_find.num_joints);
    state.slv_m_find.achd_acc->CartToJnt(
        state.slv_m_find.q,
        state.slv_m_find.qd,
        state.slv_m_find.qdd,
        state.slv_m_find.f_cstr,
        state.slv_m_find.e_acc,
        f_ext_zero_slv_m_find,
        state.slv_m_find.tau_ff,
        tau_ctrl_acc_slv_m_find);
    KDL::Add(tau_ctrl_fext_slv_m_find, tau_ctrl_acc_slv_m_find, state.slv_m_find.tau_ctrl);
}

inline void apply_motion_m_find(
    motion_m_find_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.slv_m_loosen.num_joints; ++i) {
        robot.slv_m_loosen.state->eff_cmd[i] = state.slv_m_loosen.tau_ctrl(i);
    }
    for (int i = 0; i < state.slv_m_find.num_joints; ++i) {
        robot.slv_m_find.state->eff_cmd[i] = state.slv_m_find.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.slv_m_loosen.robot);
    robif2b_kinova_gen3_update(robot.slv_m_find.robot);
}