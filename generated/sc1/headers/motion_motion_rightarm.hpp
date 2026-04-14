#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct slv_sc1_rightarm_solver_state {
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

struct motion_rightarm_state {
    slv_sc1_rightarm_solver_state slv_sc1_rightarm;
    motion_spec::runtime::PIDControl ctrl_linvel_rightarm_shoulder_ee_vertical{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_pos_rightarm_platform_elbow_height{100.0, 0.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_angvel_rightarm_shoulder_ee_anteroposterior{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_rightarm_shoulder_ee_lateral{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_rightarm_world_ee_lateral{5.0, 1.0, 3.0};
};

inline void reset_motion_rightarm(motion_rightarm_state &state) {
    state = motion_rightarm_state{};
}

inline void init_motion_rightarm(motion_rightarm_state &state, const robot_io &robot) {
    if (!state.slv_sc1_rightarm.initialized) {
        state.slv_sc1_rightarm.num_constraints = 5;
        state.slv_sc1_rightarm.num_joints = robot.slv_sc1_rightarm.chain->getNrOfJoints();
        state.slv_sc1_rightarm.num_segments = robot.slv_sc1_rightarm.chain->getNrOfSegments();
        state.slv_sc1_rightarm.q = KDL::JntArray(state.slv_sc1_rightarm.num_joints);
        state.slv_sc1_rightarm.qd = KDL::JntArray(state.slv_sc1_rightarm.num_joints);
        state.slv_sc1_rightarm.qdd = KDL::JntArray(state.slv_sc1_rightarm.num_joints);
        state.slv_sc1_rightarm.tau_ff = KDL::JntArray(state.slv_sc1_rightarm.num_joints);
        state.slv_sc1_rightarm.tau_ctrl = KDL::JntArray(state.slv_sc1_rightarm.num_joints);
        state.slv_sc1_rightarm.f_ext = KDL::Wrenches(state.slv_sc1_rightarm.num_segments);
        state.slv_sc1_rightarm.f_cstr = KDL::Jacobian(state.slv_sc1_rightarm.num_constraints);
        state.slv_sc1_rightarm.e_acc = KDL::JntArray(state.slv_sc1_rightarm.num_constraints);
        state.slv_sc1_rightarm.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.slv_sc1_rightarm.chain, state.slv_sc1_rightarm.root_acc, state.slv_sc1_rightarm.num_constraints);
        state.slv_sc1_rightarm.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.slv_sc1_rightarm.chain, state.slv_sc1_rightarm.root_acc, state.slv_sc1_rightarm.num_constraints);
        state.slv_sc1_rightarm.initialized = true;
}
}

inline void update_motion_rightarm(
    motion_rightarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_rightarm(state, robot);
    if (robot.wrench_rightarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_rightarm_ee_anteroposterior_ee = *robot.wrench_rightarm_ee_anteroposterior_ee;
    }
    if (robot.wrench_rightarm_ee_anteroposterior_platform != nullptr) {
        shared.wrench_rightarm_ee_anteroposterior_platform = *robot.wrench_rightarm_ee_anteroposterior_platform;
    }
    if (robot.wrench_rightarm_elbow != nullptr) {
        shared.wrench_rightarm_elbow = *robot.wrench_rightarm_elbow;
    }
    if (robot.wrench_leftarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_leftarm_ee_anteroposterior_ee = *robot.wrench_leftarm_ee_anteroposterior_ee;
    }
    if (robot.wrench_leftarm_ee_anteroposterior_platform != nullptr) {
        shared.wrench_leftarm_ee_anteroposterior_platform = *robot.wrench_leftarm_ee_anteroposterior_platform;
    }
    if (robot.wrench_leftarm_elbow != nullptr) {
        shared.wrench_leftarm_elbow = *robot.wrench_leftarm_elbow;
    }
    if (robot.wrench_rightarm_dist_shoulder != nullptr) {
        shared.wrench_rightarm_dist_shoulder = *robot.wrench_rightarm_dist_shoulder;
    }
    if (robot.wrench_leftarm_dist_shoulder != nullptr) {
        shared.wrench_leftarm_dist_shoulder = *robot.wrench_leftarm_dist_shoulder;
    }
    if (robot.wrench_rightarm_dist_platform != nullptr) {
        shared.wrench_rightarm_dist_platform = *robot.wrench_rightarm_dist_platform;
    }
    if (robot.wrench_leftarm_dist_platform != nullptr) {
        shared.wrench_leftarm_dist_platform = *robot.wrench_leftarm_dist_platform;
    }
    if (robot.wrench_dist_platform != nullptr) {
        shared.wrench_dist_platform = *robot.wrench_dist_platform;
    }

    for (int i = 0; i < state.slv_sc1_rightarm.num_joints; ++i) {
        state.slv_sc1_rightarm.q(i) = robot.slv_sc1_rightarm.state->pos_msr[i];
        state.slv_sc1_rightarm.qd(i) = robot.slv_sc1_rightarm.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_slv_sc1_rightarm(state.slv_sc1_rightarm.q, state.slv_sc1_rightarm.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.slv_sc1_rightarm.chain);
        fk.JntToCart(
            state.slv_sc1_rightarm.q,
            shared.pose_rightarm_shoulder_elbow,
            motion_spec::runtime::find_segment_index(*robot.slv_sc1_rightarm.chain, "frame_rightarm_elbow"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.slv_sc1_rightarm.chain);
        fk.JntToCart(
            state.slv_sc1_rightarm.q,
            shared.pose_rightarm_shoulder_ee,
            motion_spec::runtime::find_segment_index(*robot.slv_sc1_rightarm.chain, "frame_rightarm_ee"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.slv_sc1_rightarm.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_slv_sc1_rightarm,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.slv_sc1_rightarm.chain, "link_rightarm_ee"));
        shared.twist_rightarm_shoulder_ee_shoulder = tmp.deriv();
    }


}

inline bool can_start_motion_rightarm(
    motion_rightarm_state &state,
    shared_data &shared) {
    return true;
}

inline void control_motion_rightarm(
    motion_rightarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // comp_pose_rightarm_shoulder_elbow
    shared.pose_rightarm_platform_elbow = shared.pose_rightarm_platform_shoulder * shared.pose_rightarm_shoulder_elbow;
    // rot_twist_rightarm_shoulder_ee
    shared.twist_rightarm_shoulder_ee_platform = shared.pose_rightarm_platform_shoulder.M * shared.twist_rightarm_shoulder_ee_shoulder;
    // comp_pose_rightarm_shoulder_ee
    shared.pose_rightarm_platform_ee = shared.pose_rightarm_platform_shoulder * shared.pose_rightarm_shoulder_ee;
    // rot_platform_to_rightarm_ee
    shared.twist_world_platform_rightarm_ee = shared.pose_rightarm_platform_ee.M * shared.twist_world_platform_platform;
    // eval_linvel_rightarm_shoulder_ee_vertical
    shared.linvel_rightarm_shoulder_ee_vertical_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_rightarm_shoulder_ee_platform.vel[2], shared.linvel_rightarm_shoulder_ee_vertical_ref);
    // eval_pos_rightarm_platform_elbow_height
    shared.pos_rightarm_platform_elbow_height_err = motion_spec::runtime::evaluate_equality_constraint(shared.pose_rightarm_platform_elbow.p[2], shared.pos_rightarm_platform_elbow_height_ref);
    // eval_angvel_rightarm_shoulder_ee_anteroposterior
    shared.angvel_rightarm_shoulder_ee_anteroposterior_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_rightarm_shoulder_ee_platform.rot[0], shared.angvel_rightarm_shoulder_ee_anteroposterior_ref);
    // eval_angvel_rightarm_shoulder_ee_lateral
    shared.angvel_rightarm_shoulder_ee_lateral_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_rightarm_shoulder_ee_platform.rot[1], shared.angvel_rightarm_shoulder_ee_lateral_ref);
    // eval_linvel_rightarm_world_ee_lateral
    shared.linvel_rightarm_world_ee_lateral_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_world_platform_rightarm_ee.vel[1], shared.linvel_rightarm_world_ee_lateral_ref);
    // ctrl_linvel_rightarm_world_ee_lateral
    shared.eacc_rightarm_world_ee_lin_y = state.ctrl_linvel_rightarm_world_ee_lateral.control(shared.linvel_rightarm_world_ee_lateral_err);
    // ctrl_angvel_rightarm_shoulder_ee_lateral
    shared.eacc_rightarm_shoulder_ee_ang_y = state.ctrl_angvel_rightarm_shoulder_ee_lateral.control(shared.angvel_rightarm_shoulder_ee_lateral_err);
    // ctrl_angvel_rightarm_shoulder_ee_anteroposterior
    shared.eacc_rightarm_shoulder_ee_ang_x = state.ctrl_angvel_rightarm_shoulder_ee_anteroposterior.control(shared.angvel_rightarm_shoulder_ee_anteroposterior_err);
    // ctrl_pos_rightarm_platform_elbow_height
    shared.frc_rightarm_elbow = state.ctrl_pos_rightarm_platform_elbow_height.control(shared.pos_rightarm_platform_elbow_height_err);
    // ctrl_linvel_rightarm_shoulder_ee_vertical
    shared.eacc_rightarm_shoulder_ee_lin_z = state.ctrl_linvel_rightarm_shoulder_ee_vertical.control(shared.linvel_rightarm_shoulder_ee_vertical_err);
    // eval_frc_rightarm_ee_anteroposterior
    shared.frc_rightarm_ee_anteroposterior = shared.frc_rightarm_ee_anteroposterior_ref;





    KDL::SetToZero(state.slv_sc1_rightarm.f_cstr);
    state.slv_sc1_rightarm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = 1.0;
state.slv_sc1_rightarm.e_acc(0) = shared.eacc_rightarm_world_ee_lin_y;
    state.slv_sc1_rightarm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 1) = 1.0;
state.slv_sc1_rightarm.e_acc(1) = shared.eacc_rightarm_shoulder_ee_ang_x;
    state.slv_sc1_rightarm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 2) = 1.0;
state.slv_sc1_rightarm.e_acc(2) = shared.eacc_rightarm_shoulder_ee_ang_y;
    state.slv_sc1_rightarm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 3) = 1.0;
state.slv_sc1_rightarm.e_acc(3) = shared.eacc_rightarm_shoulder_ee_ang_z;
    state.slv_sc1_rightarm.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 4) = 1.0;
state.slv_sc1_rightarm.e_acc(4) = shared.eacc_rightarm_shoulder_ee_lin_z;
    for (int i = 0; i < state.slv_sc1_rightarm.num_segments; ++i) {
        KDL::SetToZero(state.slv_sc1_rightarm.f_ext[i]);
    }
    state.slv_sc1_rightarm.f_ext[motion_spec::runtime::find_segment_index(*robot.slv_sc1_rightarm.chain, "link_rightarm_ee") - 1] += shared.wrench_rightarm_ee_anteroposterior_platform;
    state.slv_sc1_rightarm.f_ext[motion_spec::runtime::find_segment_index(*robot.slv_sc1_rightarm.chain, "link_rightarm_elbow") - 1] += shared.wrench_rightarm_elbow;
    KDL::JntArray tau_ctrl_fext_slv_sc1_rightarm(state.slv_sc1_rightarm.num_joints);
    state.slv_sc1_rightarm.achd_fext->CartToJnt(
        state.slv_sc1_rightarm.q,
        state.slv_sc1_rightarm.qd,
        state.slv_sc1_rightarm.qdd,
        state.slv_sc1_rightarm.f_cstr,
        state.slv_sc1_rightarm.e_acc,
        state.slv_sc1_rightarm.f_ext,
        state.slv_sc1_rightarm.tau_ff,
        tau_ctrl_fext_slv_sc1_rightarm);
    KDL::Wrenches f_ext_zero_slv_sc1_rightarm(state.slv_sc1_rightarm.num_segments);
    KDL::JntArray tau_ctrl_acc_slv_sc1_rightarm(state.slv_sc1_rightarm.num_joints);
    state.slv_sc1_rightarm.achd_acc->CartToJnt(
        state.slv_sc1_rightarm.q,
        state.slv_sc1_rightarm.qd,
        state.slv_sc1_rightarm.qdd,
        state.slv_sc1_rightarm.f_cstr,
        state.slv_sc1_rightarm.e_acc,
        f_ext_zero_slv_sc1_rightarm,
        state.slv_sc1_rightarm.tau_ff,
        tau_ctrl_acc_slv_sc1_rightarm);
    KDL::Add(tau_ctrl_fext_slv_sc1_rightarm, tau_ctrl_acc_slv_sc1_rightarm, state.slv_sc1_rightarm.tau_ctrl);
}

inline void apply_motion_rightarm(
    motion_rightarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.slv_sc1_rightarm.num_joints; ++i) {
        robot.slv_sc1_rightarm.state->eff_cmd[i] = state.slv_sc1_rightarm.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.slv_sc1_rightarm.robot);
}