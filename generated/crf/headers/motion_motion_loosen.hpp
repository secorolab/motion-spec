#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct slv_arm_solver_state {
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

struct motion_loosen_state {
    slv_arm_solver_state slv_arm;
    motion_spec::runtime::PIDControl ctrl_linvel_ee_x_zero_loosen{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_x_zero_loosen{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_z_zero_loosen{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_y_zero_loosen{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_loosen{8.0, 0.5, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_y_zero_loosen{5.0, 1.0, 3.0};
    bool flg_cstr_torque_loosen_contact = false;
    bool mon_torque_loosen_previous = false;
    bool mon_rotation_loosen_previous = false;
};

inline void reset_motion_loosen(motion_loosen_state &state) {
    state = motion_loosen_state{};
}

inline void init_motion_loosen(motion_loosen_state &state, const robot_io &robot) {
    if (!state.slv_arm.initialized) {
        state.slv_arm.num_constraints = 6;
        state.slv_arm.num_joints = robot.slv_arm.chain->getNrOfJoints();
        state.slv_arm.num_segments = robot.slv_arm.chain->getNrOfSegments();
        state.slv_arm.q = KDL::JntArray(state.slv_arm.num_joints);
        state.slv_arm.qd = KDL::JntArray(state.slv_arm.num_joints);
        state.slv_arm.qdd = KDL::JntArray(state.slv_arm.num_joints);
        state.slv_arm.tau_ff = KDL::JntArray(state.slv_arm.num_joints);
        state.slv_arm.tau_ctrl = KDL::JntArray(state.slv_arm.num_joints);
        state.slv_arm.f_ext = KDL::Wrenches(state.slv_arm.num_segments);
        state.slv_arm.f_cstr = KDL::Jacobian(state.slv_arm.num_constraints);
        state.slv_arm.e_acc = KDL::JntArray(state.slv_arm.num_constraints);
        state.slv_arm.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_arm.chain, state.slv_arm.root_acc, state.slv_arm.num_constraints);
        state.slv_arm.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_arm.chain, state.slv_arm.root_acc, state.slv_arm.num_constraints);
        state.slv_arm.initialized = true;
}
}

inline void update_motion_loosen(
    motion_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_loosen(state, robot);
    if (robot.wrench_ee_ee != nullptr) {
        shared.wrench_ee_ee = *robot.wrench_ee_ee;
    }

    for (int i = 0; i < state.slv_arm.num_joints; ++i) {
        state.slv_arm.q(i) = robot.slv_arm.state->pos_msr[i];
        state.slv_arm.qd(i) = robot.slv_arm.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_arm(state.slv_arm.q, state.slv_arm.qd);
    {
        KDL::ChainFkSolverVel_recursive fk(*robot.slv_arm.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_slv_arm,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.slv_arm.chain, "link_ee"));
        shared.twist_ee_ee = tmp.deriv();
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.slv_arm.chain);
        fk.JntToCart(
            state.slv_arm.q,
            shared.pose_ee_world,
            motion_spec::runtime::find_segment_index(*robot.slv_arm.chain, "frame_ee"));
    }


}

inline bool can_start_motion_loosen(
    motion_loosen_state &state,
    shared_data &shared) {
    return true&& motion_spec::runtime::constraint_satisfied(shared.cstr_torque_loosen_contact_err);
}

inline void control_motion_loosen(
    motion_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_cstr_linvel_ee_x_zero_loosen
    shared.linvel_ee_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[0], shared.linvel_zero_ref);
    // eval_cstr_linvel_ee_x_zero
    shared.linvel_ee_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[0], shared.linvel_zero_ref);
    // eval_cstr_angvel_ee_x_zero_loosen
    shared.angvel_ee_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[0], shared.angvel_zero_ref);
    // eval_cstr_angvel_ee_x_zero
    shared.angvel_ee_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[0], shared.angvel_zero_ref);
    // eval_cstr_linvel_ee_z_zero
    shared.linvel_ee_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[2], shared.linvel_zero_ref);
    // eval_cstr_linvel_ee_z_zero_loosen
    shared.linvel_ee_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[2], shared.linvel_zero_ref);
    // eval_cstr_linvel_ee_y_zero
    shared.linvel_ee_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[1], shared.linvel_zero_ref);
    // eval_cstr_linvel_ee_y_zero_loosen
    shared.linvel_ee_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[1], shared.linvel_zero_ref);
    // eval_cstr_angvel_loosen
    shared.angvel_ee_z_err_loosen = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[2], shared.angvel_ee_z_ref_loosen);
    // eval_cstr_angvel_ee_y_zero_loosen
    shared.angvel_ee_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[1], shared.angvel_zero_ref);
    // eval_cstr_angvel_ee_y_zero
    shared.angvel_ee_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[1], shared.angvel_zero_ref);
    // ctrl_angvel_ee_y_zero_loosen
    shared.eacc_ee_ang_y = state.ctrl_angvel_ee_y_zero_loosen.control(shared.angvel_ee_y_err);
    // ctrl_angvel_loosen
    shared.eacc_ee_ang_z_loosen = state.ctrl_angvel_loosen.control(shared.angvel_ee_z_err_loosen);
    // ctrl_linvel_ee_y_zero_loosen
    shared.eacc_ee_lin_y = state.ctrl_linvel_ee_y_zero_loosen.control(shared.linvel_ee_y_err);
    // ctrl_linvel_ee_z_zero_loosen
    shared.eacc_ee_lin_z = state.ctrl_linvel_ee_z_zero_loosen.control(shared.linvel_ee_z_err);
    // ctrl_angvel_ee_x_zero_loosen
    shared.eacc_ee_ang_x = state.ctrl_angvel_ee_x_zero_loosen.control(shared.angvel_ee_x_err);
    // ctrl_linvel_ee_x_zero_loosen
    shared.eacc_ee_lin_x = state.ctrl_linvel_ee_x_zero_loosen.control(shared.linvel_ee_x_err);


    motion_spec::runtime::set_flag(
        state.flg_cstr_torque_loosen_contact,
        motion_spec::runtime::constraint_satisfied(shared.cstr_torque_loosen_contact_err));



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.cstr_torque_loosen_bilateral_err);
        if (motion_spec::runtime::rising_edge(state.mon_torque_loosen_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_cstr_torque_loosen_bilateral");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.cstr_rotation_loosen_bilateral_err);
        if (motion_spec::runtime::rising_edge(state.mon_rotation_loosen_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_cstr_rotation_loosen_bilateral");
        }
    }


    KDL::SetToZero(state.slv_arm.f_cstr);
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 0) = 1.0;
state.slv_arm.e_acc(0) = shared.eacc_ee_ang_z_find;
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 1) = 1.0;
state.slv_arm.e_acc(1) = shared.eacc_ee_ang_x;
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = 1.0;
state.slv_arm.e_acc(2) = shared.eacc_ee_lin_y;
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 3) = 1.0;
state.slv_arm.e_acc(3) = shared.eacc_ee_lin_x;
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
state.slv_arm.e_acc(4) = shared.eacc_ee_ang_y;
    state.slv_arm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 5) = 1.0;
state.slv_arm.e_acc(5) = shared.eacc_ee_lin_z;
    for (int i = 0; i < state.slv_arm.num_segments; ++i) {
        KDL::SetToZero(state.slv_arm.f_ext[i]);
    }
    KDL::JntArray tau_ctrl_fext_slv_arm(state.slv_arm.num_joints);
    state.slv_arm.achd_fext->CartToJnt(
        state.slv_arm.q,
        state.slv_arm.qd,
        state.slv_arm.qdd,
        state.slv_arm.f_cstr,
        state.slv_arm.e_acc,
        state.slv_arm.f_ext,
        state.slv_arm.tau_ff,
        tau_ctrl_fext_slv_arm);
    KDL::Wrenches f_ext_zero_slv_arm(state.slv_arm.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_arm(state.slv_arm.num_joints);
    state.slv_arm.achd_acc->CartToJnt(
        state.slv_arm.q,
        state.slv_arm.qd,
        state.slv_arm.qdd,
        state.slv_arm.f_cstr,
        state.slv_arm.e_acc,
        f_ext_zero_slv_arm,
        state.slv_arm.tau_ff,
        tau_ctrl_acc_slv_arm);
    KDL::Add(tau_ctrl_fext_slv_arm, tau_ctrl_acc_slv_arm, state.slv_arm.tau_ctrl);
}

inline void apply_motion_loosen(
    motion_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.slv_arm.num_joints; ++i) {
        robot.slv_arm.state->eff_cmd[i] = state.slv_arm.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.slv_arm.robot);
}