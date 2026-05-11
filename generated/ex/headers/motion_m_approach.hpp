#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_m_approach_state {
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_linvel_z{5.0, 1.0, 1.0};
    motion_spec::runtime::PIDControl ctrl_linvel_x{5.0, 1.0, 1.0};
    motion_spec::runtime::PIDControl ctrl_linvel_y{5.0, 1.0, 1.0};
    motion_spec::runtime::PIDControl ctrl_keep_j2{10.0, 0.0, 0.5};
    motion_spec::runtime::PIDControl ctrl_limit_j4{10.0, 0.0, 0.5};
    bool mon_contact_previous = false;
    bool mon_overload_previous = false;
};

inline void reset_motion_m_approach(motion_m_approach_state &state) {
    state = motion_m_approach_state{};
}

inline void init_motion_m_approach(motion_m_approach_state &state, const robot_io &robot) {
    if (!state.arm_solver.initialized) {
        state.arm_solver.num_constraints = 3;
        state.arm_solver.num_joints = robot.arm_solver.chain->getNrOfJoints();
        state.arm_solver.num_segments = robot.arm_solver.chain->getNrOfSegments();
        state.arm_solver.q = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qdd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ff = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ctrl = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.f_cstr = KDL::Jacobian(state.arm_solver.num_constraints);
        state.arm_solver.e_acc = KDL::JntArray(state.arm_solver.num_constraints);
        state.arm_solver.root_acc.vel = KDL::Vector(-0.0, -0.0, 9.81);
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_m_approach(state, robot);
    if (robot.wrench_ee != nullptr) {
        shared.wrench_ee = *robot.wrench_ee;
    }
    if (robot.wrench_ee != nullptr) {
        shared.wrench_ee = *robot.wrench_ee;
    }

    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        state.arm_solver.q(i) = robot.arm_solver.state->pos_msr[i];
        state.arm_solver.qd(i) = robot.arm_solver.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver(state.arm_solver.q, state.arm_solver.qd);
    shared.q_j2 = state.arm_solver.q(motion_spec::runtime::find_joint_index(*robot.arm_solver.chain, "joint-2"));

    shared.q_j4 = state.arm_solver.q(motion_spec::runtime::find_joint_index(*robot.arm_solver.chain, "joint-4"));


    if (!state.snapshot_taken) {
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared) {
    // eval_m_approach_until_cstr_contact
    shared.wrench_ee_force_z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_threshold);
    // eval_m_approach_until_cstr_overload
    shared.wrench_ee_force_z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_overload);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee_force_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_contact_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_contact");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee_force_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_overload_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_overload");
        }
    }

}

inline void control_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_m_approach_while_cstr_linvel_z
    shared.twist_ee_linear_z_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[2], shared.vel_z_down);
    // eval_m_approach_while_cstr_linvel_x
    shared.twist_ee_linear_x_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[0], shared.vel_zero);
    // eval_m_approach_while_cstr_linvel_y
    shared.twist_ee_linear_y_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[1], shared.vel_y_zero);
    // eval_m_approach_while_cstr_keep_j2
    shared.q_j2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.q_j2, shared.q_j2_ref);
    // eval_m_approach_while_cstr_limit_j4
    shared.q_j4_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.q_j4, shared.q_j4_lower, shared.q_j4_upper);
    // ctrl_limit_j4
    shared.tau_ctrl_limit_j4 = state.ctrl_limit_j4.control(shared.q_j4_err);
    // ctrl_keep_j2
    shared.tau_ctrl_keep_j2 = state.ctrl_keep_j2.control(shared.q_j2_err);
    // ctrl_linvel_y
    shared.eacc_twist_ee_linear_y_m_approach = state.ctrl_linvel_y.control(shared.twist_ee_linear_y_err_m_approach);
    // ctrl_linvel_x
    shared.eacc_twist_ee_linear_x_m_approach = state.ctrl_linvel_x.control(shared.twist_ee_linear_x_err_m_approach);
    // ctrl_linvel_z
    shared.eacc_twist_ee_linear_z_m_approach = state.ctrl_linvel_z.control(shared.twist_ee_linear_z_err_m_approach);



    KDL::SetToZero(state.arm_solver.f_cstr);
    {
        KDL::Frame alpha_frame_arm_solver_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm_solver_0(*robot.arm_solver.chain);
        alpha_fk_arm_solver_0.JntToCart(
            state.arm_solver.q,
            alpha_frame_arm_solver_0,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_arm_solver_0 =
            alpha_frame_arm_solver_0.M * KDL::Vector(0.0, 0.0, 1.0);
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_arm_solver_0[0];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_arm_solver_0[1];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_arm_solver_0[2];
    }
    state.arm_solver.e_acc(0) = shared.eacc_twist_ee_linear_z_m_approach;

    {
        KDL::Frame alpha_frame_arm_solver_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm_solver_1(*robot.arm_solver.chain);
        alpha_fk_arm_solver_1.JntToCart(
            state.arm_solver.q,
            alpha_frame_arm_solver_1,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_arm_solver_1 =
            alpha_frame_arm_solver_1.M * KDL::Vector(1.0, 0.0, 0.0);
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = alpha_axis_arm_solver_1[0];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = alpha_axis_arm_solver_1[1];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 1) = alpha_axis_arm_solver_1[2];
    }
    state.arm_solver.e_acc(1) = shared.eacc_twist_ee_linear_x_m_approach;

    {
        KDL::Frame alpha_frame_arm_solver_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_arm_solver_2(*robot.arm_solver.chain);
        alpha_fk_arm_solver_2.JntToCart(
            state.arm_solver.q,
            alpha_frame_arm_solver_2,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "frame_ee"));
        const KDL::Vector alpha_axis_arm_solver_2 =
            alpha_frame_arm_solver_2.M * KDL::Vector(0.0, 1.0, 0.0);
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 2) = alpha_axis_arm_solver_2[0];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = alpha_axis_arm_solver_2[1];
        state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = alpha_axis_arm_solver_2[2];
    }
    state.arm_solver.e_acc(2) = shared.eacc_twist_ee_linear_y_m_approach;
    KDL::SetToZero(state.arm_solver.tau_ff);
    state.arm_solver.tau_ff(motion_spec::runtime::find_joint_index(*robot.arm_solver.chain, "joint-2")) += shared.tau_ctrl_keep_j2;
    state.arm_solver.tau_ff(motion_spec::runtime::find_joint_index(*robot.arm_solver.chain, "joint-4")) += shared.tau_ctrl_limit_j4;
    KDL::Wrenches f_ext_zero_arm_solver(state.arm_solver.num_segments);
    KDL::JntArray tau_ctrl_acc_arm_solver(state.arm_solver.num_joints);
    state.arm_solver.achd_acc->CartToJnt(
        state.arm_solver.q,
        state.arm_solver.qd,
        state.arm_solver.qdd,
        state.arm_solver.f_cstr,
        state.arm_solver.e_acc,
        f_ext_zero_arm_solver,
        state.arm_solver.tau_ff,
        tau_ctrl_acc_arm_solver);
    state.arm_solver.tau_ctrl = tau_ctrl_acc_arm_solver;

}

inline void apply_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.state->eff_cmd[i] = state.arm_solver.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.arm_solver.robot);
}
