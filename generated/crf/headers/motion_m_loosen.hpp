#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_m_loosen_state {
    right_arm_solver_solver_state right_arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_angvel_loosen{8.0, 0.5, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_x_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_y_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_ee_z_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_x_zero{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_ee_y_zero{5.0, 1.0, 3.0};
    bool flg_torque_loosen_contact = false;
    bool mon_rotation_loosen_bilateral_previous = false;
    bool mon_torque_loosen_bilateral_previous = false;
};

inline void reset_motion_m_loosen(motion_m_loosen_state &state) {
    state = motion_m_loosen_state{};
}

inline void init_motion_m_loosen(motion_m_loosen_state &state, const robot_io &robot) {
    if (!state.right_arm_solver.initialized) {
        state.right_arm_solver.num_constraints = 6;
        state.right_arm_solver.num_joints = robot.right_arm_solver.chain->getNrOfJoints();
        state.right_arm_solver.num_segments = robot.right_arm_solver.chain->getNrOfSegments();
        state.right_arm_solver.q = KDL::JntArray(state.right_arm_solver.num_joints);
        state.right_arm_solver.qd = KDL::JntArray(state.right_arm_solver.num_joints);
        state.right_arm_solver.qdd = KDL::JntArray(state.right_arm_solver.num_joints);
        state.right_arm_solver.tau_ff = KDL::JntArray(state.right_arm_solver.num_joints);
        state.right_arm_solver.tau_ctrl = KDL::JntArray(state.right_arm_solver.num_joints);
        state.right_arm_solver.f_cstr = KDL::Jacobian(state.right_arm_solver.num_constraints);
        state.right_arm_solver.e_acc = KDL::JntArray(state.right_arm_solver.num_constraints);
        state.right_arm_solver.root_acc.vel = KDL::Vector(-0.0, -0.0, 9.81);
        state.right_arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.right_arm_solver.chain, state.right_arm_solver.root_acc, state.right_arm_solver.num_constraints);
        state.right_arm_solver.initialized = true;
    }
}

inline void update_motion_m_loosen(
    motion_m_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_m_loosen(state, robot);
    if (robot.wrench_ee_ee != nullptr) {
        shared.wrench_ee_ee = *robot.wrench_ee_ee;
    }
    if (robot.wrench_ee_ee != nullptr) {
        shared.wrench_ee_ee = *robot.wrench_ee_ee;
    }

    for (int i = 0; i < state.right_arm_solver.num_joints; ++i) {
        state.right_arm_solver.q(i) = robot.right_arm_solver.state->pos_msr[i];
        state.right_arm_solver.qd(i) = robot.right_arm_solver.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_right_arm_solver(state.right_arm_solver.q, state.right_arm_solver.qd);


    if (!state.snapshot_taken) {
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_m_loosen(
    motion_m_loosen_state &state,
    shared_data &shared) {
    // eval_m_loosen_when_cstr_torque_loosen_contact
    shared.wrench_ee_ee_torque_z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee_ee.torque[2], shared.torque_loosen_min_contact);

    return true&& motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err);
}

inline void monitor_motion_m_loosen(
    motion_m_loosen_state &state,
    shared_data &shared) {
    // eval_m_loosen_when_cstr_torque_loosen_contact
    shared.wrench_ee_ee_torque_z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee_ee.torque[2], shared.torque_loosen_min_contact);

    motion_spec::runtime::set_flag(
        state.flg_torque_loosen_contact,
        motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err));

    // compute_pose_arm_ee_world_rotation_z
    KDL::Vector diff_pose_arm_ee_world_rotation_z = KDL::diff(KDL::Rotation::Identity(), shared.pose_arm_ee_world.M);
    shared.pose_arm_ee_world_rotation_z = diff_pose_arm_ee_world_rotation_z[2];
    // eval_m_loosen_until_cstr_rotation_loosen_bilateral
    shared.pose_arm_ee_world_rotation_z_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_arm_ee_world_rotation_z, shared.rotation_loosen_lower, shared.rotation_loosen_upper);
    // eval_m_loosen_until_cstr_torque_loosen_bilateral
    shared.wrench_ee_ee_torque_z_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.wrench_ee_ee.torque[2], shared.torque_loosen_lower, shared.torque_loosen_upper);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.pose_arm_ee_world_rotation_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_rotation_loosen_bilateral_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_rotation_loosen");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee_ee_torque_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_torque_loosen_bilateral_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_torque_loosen");
        }
    }

}

inline void control_motion_m_loosen(
    motion_m_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_m_loosen_while_cstr_angvel_loosen
    shared.twist_ee_ee_angular_z_err_m_loosen = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[2], shared.angvel_ee_z_ref_loosen);
    // eval_m_find_while_cstr_linvel_ee_x_zero
    shared.twist_ee_ee_linear_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[0], shared.linvel_zero_ref);
    // eval_m_find_while_cstr_linvel_ee_y_zero
    shared.twist_ee_ee_linear_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[1], shared.linvel_zero_inline_ref);
    // eval_m_find_while_cstr_linvel_ee_z_zero
    shared.twist_ee_ee_linear_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.vel[2], shared.linvel_zero_ref);
    // eval_m_find_while_cstr_angvel_ee_x_zero
    shared.twist_ee_ee_angular_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[0], shared.angvel_zero_ref);
    // eval_m_find_while_cstr_angvel_ee_y_zero
    shared.twist_ee_ee_angular_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_ee.rot[1], shared.angvel_zero_ref);
    // ctrl_angvel_ee_y_zero
    shared.eacc_twist_ee_ee_angular_y = state.ctrl_angvel_ee_y_zero.control(shared.twist_ee_ee_angular_y_err);
    // ctrl_angvel_ee_x_zero
    shared.eacc_twist_ee_ee_angular_x = state.ctrl_angvel_ee_x_zero.control(shared.twist_ee_ee_angular_x_err);
    // ctrl_linvel_ee_z_zero
    shared.eacc_twist_ee_ee_linear_z = state.ctrl_linvel_ee_z_zero.control(shared.twist_ee_ee_linear_z_err);
    // ctrl_linvel_ee_y_zero
    shared.eacc_twist_ee_ee_linear_y = state.ctrl_linvel_ee_y_zero.control(shared.twist_ee_ee_linear_y_err);
    // ctrl_linvel_ee_x_zero
    shared.eacc_twist_ee_ee_linear_x = state.ctrl_linvel_ee_x_zero.control(shared.twist_ee_ee_linear_x_err);
    // ctrl_angvel_loosen
    shared.eacc_twist_ee_ee_angular_z_m_loosen = state.ctrl_angvel_loosen.control(shared.twist_ee_ee_angular_z_err_m_loosen);



    KDL::SetToZero(state.right_arm_solver.f_cstr);
    {
        KDL::Frame alpha_frame_right_arm_solver_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_0(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_0.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_0,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_0 =
            alpha_frame_right_arm_solver_0.M * KDL::Vector(0.0, 0.0, 1.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 0) = alpha_axis_right_arm_solver_0[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 0) = alpha_axis_right_arm_solver_0[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 0) = alpha_axis_right_arm_solver_0[2];
    }
    state.right_arm_solver.e_acc(0) = shared.eacc_twist_ee_ee_angular_z_m_loosen;

    {
        KDL::Frame alpha_frame_right_arm_solver_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_1(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_1.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_1,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_1 =
            alpha_frame_right_arm_solver_1.M * KDL::Vector(1.0, 0.0, 0.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_right_arm_solver_1[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_right_arm_solver_1[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_right_arm_solver_1[2];
    }
    state.right_arm_solver.e_acc(1) = shared.eacc_twist_ee_ee_linear_x;

    {
        KDL::Frame alpha_frame_right_arm_solver_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_2(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_2.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_2,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_2 =
            alpha_frame_right_arm_solver_2.M * KDL::Vector(0.0, 1.0, 0.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_right_arm_solver_2[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_right_arm_solver_2[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_right_arm_solver_2[2];
    }
    state.right_arm_solver.e_acc(2) = shared.eacc_twist_ee_ee_linear_y;

    {
        KDL::Frame alpha_frame_right_arm_solver_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_3(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_3.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_3,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_3 =
            alpha_frame_right_arm_solver_3.M * KDL::Vector(0.0, 0.0, 1.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 3) = alpha_axis_right_arm_solver_3[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 3) = alpha_axis_right_arm_solver_3[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 3) = alpha_axis_right_arm_solver_3[2];
    }
    state.right_arm_solver.e_acc(3) = shared.eacc_twist_ee_ee_linear_z;

    {
        KDL::Frame alpha_frame_right_arm_solver_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_4(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_4.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_4,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_4 =
            alpha_frame_right_arm_solver_4.M * KDL::Vector(1.0, 0.0, 0.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_right_arm_solver_4[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_right_arm_solver_4[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_right_arm_solver_4[2];
    }
    state.right_arm_solver.e_acc(4) = shared.eacc_twist_ee_ee_angular_x;

    {
        KDL::Frame alpha_frame_right_arm_solver_5;
        KDL::ChainFkSolverPos_recursive alpha_fk_right_arm_solver_5(*robot.right_arm_solver.chain);
        alpha_fk_right_arm_solver_5.JntToCart(
            state.right_arm_solver.q,
            alpha_frame_right_arm_solver_5,
            motion_spec::runtime::find_segment_index(*robot.right_arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_right_arm_solver_5 =
            alpha_frame_right_arm_solver_5.M * KDL::Vector(0.0, 1.0, 0.0);
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 5) = alpha_axis_right_arm_solver_5[0];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 5) = alpha_axis_right_arm_solver_5[1];
        state.right_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = alpha_axis_right_arm_solver_5[2];
    }
    state.right_arm_solver.e_acc(5) = shared.eacc_twist_ee_ee_angular_y;
    KDL::SetToZero(state.right_arm_solver.tau_ff);
    KDL::Wrenches f_ext_zero_right_arm_solver(state.right_arm_solver.num_segments);
    KDL::JntArray tau_ctrl_acc_right_arm_solver(state.right_arm_solver.num_joints);
    state.right_arm_solver.achd_acc->CartToJnt(
        state.right_arm_solver.q,
        state.right_arm_solver.qd,
        state.right_arm_solver.qdd,
        state.right_arm_solver.f_cstr,
        state.right_arm_solver.e_acc,
        f_ext_zero_right_arm_solver,
        state.right_arm_solver.tau_ff,
        tau_ctrl_acc_right_arm_solver);
    state.right_arm_solver.tau_ctrl = tau_ctrl_acc_right_arm_solver;

}

inline void apply_motion_m_loosen(
    motion_m_loosen_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.right_arm_solver.num_joints; ++i) {
        robot.right_arm_solver.state->eff_cmd[i] = state.right_arm_solver.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.right_arm_solver.robot);
}
