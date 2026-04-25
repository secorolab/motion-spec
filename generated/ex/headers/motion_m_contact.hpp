#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct slv_m_contact_solver_state {
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

struct motion_m_contact_state {
    slv_m_contact_solver_state slv_m_contact;
    motion_spec::runtime::PIDControl ctrl_frc_z{1.0, 0.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_angvel_x{5.0, 0.5, 1.0};
    motion_spec::runtime::PIDControl ctrl_angvel_y{5.0, 0.5, 1.0};
    motion_spec::runtime::PIDControl ctrl_angvel_z{5.0, 0.5, 1.0};
    motion_spec::runtime::PIDControl ctrl_keep_j2{10.0, 0.0, 0.5};
    motion_spec::runtime::PIDControl ctrl_limit_j4{10.0, 0.0, 0.5};
    bool flg_in_contact = false;
    bool mon_overload_previous = false;
};

inline void reset_motion_m_contact(motion_m_contact_state &state) {
    state = motion_m_contact_state{};
}

inline void init_motion_m_contact(motion_m_contact_state &state, const robot_io &robot) {
    if (!state.slv_m_contact.initialized) {
        state.slv_m_contact.num_constraints = 3;
        state.slv_m_contact.num_joints = robot.slv_m_contact.chain->getNrOfJoints();
        state.slv_m_contact.num_segments = robot.slv_m_contact.chain->getNrOfSegments();
        state.slv_m_contact.q = KDL::JntArray(state.slv_m_contact.num_joints);
        state.slv_m_contact.qd = KDL::JntArray(state.slv_m_contact.num_joints);
        state.slv_m_contact.qdd = KDL::JntArray(state.slv_m_contact.num_joints);
        state.slv_m_contact.tau_ff = KDL::JntArray(state.slv_m_contact.num_joints);
        state.slv_m_contact.tau_ctrl = KDL::JntArray(state.slv_m_contact.num_joints);
        state.slv_m_contact.f_ext = KDL::Wrenches(state.slv_m_contact.num_segments);
        state.slv_m_contact.f_cstr = KDL::Jacobian(state.slv_m_contact.num_constraints);
        state.slv_m_contact.e_acc = KDL::JntArray(state.slv_m_contact.num_constraints);
        state.slv_m_contact.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_m_contact.chain, state.slv_m_contact.root_acc, state.slv_m_contact.num_constraints);
        state.slv_m_contact.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_m_contact.chain, state.slv_m_contact.root_acc, state.slv_m_contact.num_constraints);
        state.slv_m_contact.initialized = true;
}
}

inline void update_motion_m_contact(
    motion_m_contact_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_m_contact(state, robot);
    if (robot.wrench_ee != nullptr) {
        shared.wrench_ee = *robot.wrench_ee;
    }

    for (int i = 0; i < state.slv_m_contact.num_joints; ++i) {
        state.slv_m_contact.q(i) = robot.slv_m_contact.state->pos_msr[i];
        state.slv_m_contact.qd(i) = robot.slv_m_contact.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_m_contact(state.slv_m_contact.q, state.slv_m_contact.qd);


}

inline bool can_start_motion_m_contact(
    motion_m_contact_state &state,
    shared_data &shared) {
    // eval_m_contact_while_cstr_frc_z
    shared.wrench_ee.force.z_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.wrench_ee.force[2], shared.frc_z_ref);
    // ctrl_frc_z
    shared.wrench_ee.force.z = state.ctrl_frc_z.control(shared.wrench_ee.force.z_err_m_contact);
    // eval_m_contact_when_cstr_in_contact
    shared.wrench_ee.force.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_start);

    return true&& motion_spec::runtime::constraint_satisfied(shared.wrench_ee.force.z_err);
}

inline void monitor_motion_m_contact(
    motion_m_contact_state &state,
    shared_data &shared) {
    // eval_m_contact_while_cstr_frc_z
    shared.wrench_ee.force.z_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.wrench_ee.force[2], shared.frc_z_ref);
    // ctrl_frc_z
    shared.wrench_ee.force.z = state.ctrl_frc_z.control(shared.wrench_ee.force.z_err_m_contact);
    // eval_m_contact_when_cstr_in_contact
    shared.wrench_ee.force.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_start);

    motion_spec::runtime::set_flag(
        state.flg_in_contact,
        motion_spec::runtime::constraint_satisfied(shared.wrench_ee.force.z_err));

    // eval_m_contact_until_cstr_overload
    shared.wrench_ee.force.z_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.wrench_ee.force[2], shared.frc_contact_overload);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.wrench_ee.force.z_err);
        if (motion_spec::runtime::rising_edge(state.mon_overload_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("evt_overload");
        }
    }

}

inline void control_motion_m_contact(
    motion_m_contact_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_m_contact_while_cstr_frc_z
    shared.wrench_ee.force.z_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.wrench_ee.force[2], shared.frc_z_ref);
    // eval_m_contact_while_cstr_angvel_x
    shared.twist_ee.angular.x_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.rot[0], shared.angvel_zero);
    // eval_m_contact_while_cstr_angvel_y
    shared.twist_ee.angular.y_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.rot[1], shared.angvel_zero);
    // eval_m_contact_while_cstr_angvel_z
    shared.twist_ee.angular.z_err_m_contact = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee.rot[2], shared.angvel_zero);
    // eval_m_approach_while_cstr_keep_j2
    shared.q_j2_err = motion_spec::runtime::evaluate_greater_than_constraint(shared.q_j2, shared.q_j2_ref);
    // eval_m_approach_while_cstr_limit_j4
    shared.q_j4_err = motion_spec::runtime::evaluate_bilateral_constraint(shared.q_j4, shared.q_j4_lower, shared.q_j4_upper);
    // ctrl_limit_j4
    shared.tau_ctrl_limit_j4 = state.ctrl_limit_j4.control(shared.q_j4_err);
    // ctrl_keep_j2
    shared.tau_ctrl_keep_j2 = state.ctrl_keep_j2.control(shared.q_j2_err);
    // ctrl_angvel_z
    shared.eacc_twist_ee.angular.z_m_contact = state.ctrl_angvel_z.control(shared.twist_ee.angular.z_err_m_contact);
    // ctrl_angvel_y
    shared.eacc_twist_ee.angular.y_m_contact = state.ctrl_angvel_y.control(shared.twist_ee.angular.y_err_m_contact);
    // ctrl_angvel_x
    shared.eacc_twist_ee.angular.x_m_contact = state.ctrl_angvel_x.control(shared.twist_ee.angular.x_err_m_contact);
    // ctrl_frc_z
    shared.wrench_ee.force.z = state.ctrl_frc_z.control(shared.wrench_ee.force.z_err_m_contact);



    KDL::SetToZero(state.slv_m_contact.f_cstr);
    state.slv_m_contact.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 0) = 1.0;
state.slv_m_contact.e_acc(0) = shared.eacc_twist_ee.angular.x_m_contact;
    state.slv_m_contact.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 1) = 1.0;
state.slv_m_contact.e_acc(1) = shared.eacc_twist_ee.angular.y_m_contact;
    state.slv_m_contact.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 2) = 1.0;
state.slv_m_contact.e_acc(2) = shared.eacc_twist_ee.angular.z_m_contact;
    for (int i = 0; i < state.slv_m_contact.num_segments; ++i) {
        KDL::SetToZero(state.slv_m_contact.f_ext[i]);
    }
    state.slv_m_contact.f_ext[motion_spec::runtime::find_segment_index(*robot.slv_m_contact.chain, "link_ee") - 1] += shared.wrench_ee;
    KDL::JntArray tau_ctrl_fext_slv_m_contact(state.slv_m_contact.num_joints);
    state.slv_m_contact.achd_fext->CartToJnt(
        state.slv_m_contact.q,
        state.slv_m_contact.qd,
        state.slv_m_contact.qdd,
        state.slv_m_contact.f_cstr,
        state.slv_m_contact.e_acc,
        state.slv_m_contact.f_ext,
        state.slv_m_contact.tau_ff,
        tau_ctrl_fext_slv_m_contact);
    KDL::Wrenches f_ext_zero_slv_m_contact(state.slv_m_contact.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_m_contact(state.slv_m_contact.num_joints);
    state.slv_m_contact.achd_acc->CartToJnt(
        state.slv_m_contact.q,
        state.slv_m_contact.qd,
        state.slv_m_contact.qdd,
        state.slv_m_contact.f_cstr,
        state.slv_m_contact.e_acc,
        f_ext_zero_slv_m_contact,
        state.slv_m_contact.tau_ff,
        tau_ctrl_acc_slv_m_contact);
    KDL::Add(tau_ctrl_fext_slv_m_contact, tau_ctrl_acc_slv_m_contact, state.slv_m_contact.tau_ctrl);
}

inline void apply_motion_m_contact(
    motion_m_contact_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.slv_m_contact.num_joints; ++i) {
        robot.slv_m_contact.state->eff_cmd[i] = state.slv_m_contact.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.slv_m_contact.robot);
}