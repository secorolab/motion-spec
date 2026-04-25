#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct slv_m_approach_solver_state {
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

struct motion_m_approach_state {
    slv_m_approach_solver_state slv_m_approach;
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
    if (!state.slv_m_approach.initialized) {
        state.slv_m_approach.num_constraints = 3;
        state.slv_m_approach.num_joints = robot.slv_m_approach.chain->getNrOfJoints();
        state.slv_m_approach.num_segments = robot.slv_m_approach.chain->getNrOfSegments();
        state.slv_m_approach.q = KDL::JntArray(state.slv_m_approach.num_joints);
        state.slv_m_approach.qd = KDL::JntArray(state.slv_m_approach.num_joints);
        state.slv_m_approach.qdd = KDL::JntArray(state.slv_m_approach.num_joints);
        state.slv_m_approach.tau_ff = KDL::JntArray(state.slv_m_approach.num_joints);
        state.slv_m_approach.tau_ctrl = KDL::JntArray(state.slv_m_approach.num_joints);
        state.slv_m_approach.f_ext = KDL::Wrenches(state.slv_m_approach.num_segments);
        state.slv_m_approach.f_cstr = KDL::Jacobian(state.slv_m_approach.num_constraints);
        state.slv_m_approach.e_acc = KDL::JntArray(state.slv_m_approach.num_constraints);
        state.slv_m_approach.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_m_approach.chain, state.slv_m_approach.root_acc, state.slv_m_approach.num_constraints);
        state.slv_m_approach.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_m_approach.chain, state.slv_m_approach.root_acc, state.slv_m_approach.num_constraints);
        state.slv_m_approach.initialized = true;
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

    for (int i = 0; i < state.slv_m_approach.num_joints; ++i) {
        state.slv_m_approach.q(i) = robot.slv_m_approach.state->pos_msr[i];
        state.slv_m_approach.qd(i) = robot.slv_m_approach.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_m_approach(state.slv_m_approach.q, state.slv_m_approach.qd);


}

inline bool can_start_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared) {
    // eval_m_contact_while_cstr_frc_z
    shared.wrench_ee.force.z_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.wrench_ee.force[2], shared.frc_z_ref);
    // ctrl_frc_z
    shared.wrench_ee.force.z = state.ctrl_frc_z.control(shared.wrench_ee.force.z_err_m_contact);
    // eval_m_approach_until_cstr_contact
    shared.wrench_ee.force.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_threshold);
    // eval_m_approach_until_cstr_overload
    shared.wrench_ee.force.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_overload);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee.force.z_err);
        if (motion_spec::runtime::rising_edge(state.mon_contact_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_contact");
        }
    }



    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee.force.z_err);
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
    shared.twist_ee.linear.z_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[2], shared.vel_z_down);
    // eval_m_approach_while_cstr_linvel_x
    shared.twist_ee.linear.x_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[0], shared.vel_zero);
    // eval_m_approach_while_cstr_linvel_y
    shared.twist_ee.linear.y_err_m_approach = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.vel[1], shared.vel_y_zero);
    // eval_m_approach_while_cstr_keep_j2
    shared.q_j2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.q_j2, shared.q_j2_ref);
    // eval_m_approach_while_cstr_limit_j4
    shared.q_j4_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.q_j4, shared.q_j4_lower, shared.q_j4_upper);
    // ctrl_limit_j4
    shared.tau_ctrl_limit_j4 = state.ctrl_limit_j4.control(shared.q_j4_err);
    // ctrl_keep_j2
    shared.tau_ctrl_keep_j2 = state.ctrl_keep_j2.control(shared.q_j2_err);
    // ctrl_linvel_y
    shared.eacc_twist_ee.linear.y_m_approach = state.ctrl_linvel_y.control(shared.twist_ee.linear.y_err_m_approach);
    // ctrl_linvel_x
    shared.eacc_twist_ee.linear.x_m_approach = state.ctrl_linvel_x.control(shared.twist_ee.linear.x_err_m_approach);
    // ctrl_linvel_z
    shared.eacc_twist_ee.linear.z_m_approach = state.ctrl_linvel_z.control(shared.twist_ee.linear.z_err_m_approach);



    KDL::SetToZero(state.slv_m_approach.f_cstr);
    state.slv_m_approach.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
state.slv_m_approach.e_acc(0) = shared.eacc_twist_ee.linear.x_m_approach;
    state.slv_m_approach.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
state.slv_m_approach.e_acc(1) = shared.eacc_twist_ee.linear.y_m_approach;
    state.slv_m_approach.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
state.slv_m_approach.e_acc(2) = shared.eacc_twist_ee.linear.z_m_approach;
    for (int i = 0; i < state.slv_m_approach.num_segments; ++i) {
        KDL::SetToZero(state.slv_m_approach.f_ext[i]);
    }
    KDL::JntArray tau_ctrl_fext_slv_m_approach(state.slv_m_approach.num_joints);
    state.slv_m_approach.achd_fext->CartToJnt(
        state.slv_m_approach.q,
        state.slv_m_approach.qd,
        state.slv_m_approach.qdd,
        state.slv_m_approach.f_cstr,
        state.slv_m_approach.e_acc,
        state.slv_m_approach.f_ext,
        state.slv_m_approach.tau_ff,
        tau_ctrl_fext_slv_m_approach);
    KDL::Wrenches f_ext_zero_slv_m_approach(state.slv_m_approach.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_m_approach(state.slv_m_approach.num_joints);
    state.slv_m_approach.achd_acc->CartToJnt(
        state.slv_m_approach.q,
        state.slv_m_approach.qd,
        state.slv_m_approach.qdd,
        state.slv_m_approach.f_cstr,
        state.slv_m_approach.e_acc,
        f_ext_zero_slv_m_approach,
        state.slv_m_approach.tau_ff,
        tau_ctrl_acc_slv_m_approach);
    KDL::Add(tau_ctrl_fext_slv_m_approach, tau_ctrl_acc_slv_m_approach, state.slv_m_approach.tau_ctrl);
}

inline void apply_motion_m_approach(
    motion_m_approach_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.slv_m_approach.num_joints; ++i) {
        robot.slv_m_approach.state->eff_cmd[i] = state.slv_m_approach.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.slv_m_approach.robot);
}