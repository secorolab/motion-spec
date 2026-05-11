#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_motion_leftarm_state {
    left_arm_solver_solver_state left_arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_linvel_leftarm_shoulder_ee_vertical{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_pos_leftarm_platform_elbow_height{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_leftarm_shoulder_ee_anteroposterior{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_leftarm_shoulder_ee_lateral{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_linvel_leftarm_world_ee_lateral{5.0, 1.0, 3.0};
    motion_spec::runtime::PIDControl ctrl_angvel_leftarm_shoulder_ee_vertical{200.0, 2.9, 80.5};
    motion_spec::runtime::PIDControl ctrl_dist_leftarm_shoulder_ee{450.0, 65.5, 80.0, 0.99};
};

inline void reset_motion_motion_leftarm(motion_motion_leftarm_state &state) {
    state = motion_motion_leftarm_state{};
}

inline void init_motion_motion_leftarm(motion_motion_leftarm_state &state, const robot_io &robot) {
    if (!state.left_arm_solver.initialized) {
        state.left_arm_solver.num_constraints = 5;
        state.left_arm_solver.num_joints = robot.left_arm_solver.chain->getNrOfJoints();
        state.left_arm_solver.num_segments = robot.left_arm_solver.chain->getNrOfSegments();
        state.left_arm_solver.q = KDL::JntArray(state.left_arm_solver.num_joints);
        state.left_arm_solver.qd = KDL::JntArray(state.left_arm_solver.num_joints);
        state.left_arm_solver.qdd = KDL::JntArray(state.left_arm_solver.num_joints);
        state.left_arm_solver.tau_ff = KDL::JntArray(state.left_arm_solver.num_joints);
        state.left_arm_solver.tau_ctrl = KDL::JntArray(state.left_arm_solver.num_joints);
        state.left_arm_solver.f_ext = KDL::Wrenches(state.left_arm_solver.num_segments);
        state.left_arm_solver.f_cstr = KDL::Jacobian(state.left_arm_solver.num_constraints);
        state.left_arm_solver.e_acc = KDL::JntArray(state.left_arm_solver.num_constraints);
        state.left_arm_solver.root_acc.vel = KDL::Vector(-9.72607409, 0.90542332, 0.90542332);
        state.left_arm_solver.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext>(*robot.left_arm_solver.chain, state.left_arm_solver.root_acc, state.left_arm_solver.num_constraints);
        state.left_arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.left_arm_solver.chain, state.left_arm_solver.root_acc, state.left_arm_solver.num_constraints);
        state.left_arm_solver.initialized = true;
    }
}

inline void update_motion_motion_leftarm(
    motion_motion_leftarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_motion_leftarm(state, robot);
    if (robot.wrench_leftarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_leftarm_ee_anteroposterior_ee = *robot.wrench_leftarm_ee_anteroposterior_ee;
    }
    if (robot.wrench_rightarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_rightarm_ee_anteroposterior_ee = *robot.wrench_rightarm_ee_anteroposterior_ee;
    }
    if (robot.wrench_leftarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_leftarm_ee_anteroposterior_ee = *robot.wrench_leftarm_ee_anteroposterior_ee;
    }
    if (robot.wrench_rightarm_ee_anteroposterior_ee != nullptr) {
        shared.wrench_rightarm_ee_anteroposterior_ee = *robot.wrench_rightarm_ee_anteroposterior_ee;
    }

    for (int i = 0; i < state.left_arm_solver.num_joints; ++i) {
        state.left_arm_solver.q(i) = robot.left_arm_solver.state->pos_msr[i];
        state.left_arm_solver.qd(i) = robot.left_arm_solver.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_left_arm_solver(state.left_arm_solver.q, state.left_arm_solver.qd);
    {
        KDL::ChainFkSolverVel_recursive fk(*robot.left_arm_solver.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_left_arm_solver,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "link_leftarm_ee"));
        shared.twist_leftarm_shoulder_ee_platform = tmp.deriv();
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.left_arm_solver.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_left_arm_solver,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "link_leftarm_ee"));
        shared.twist_leftarm_shoulder_ee_shoulder = tmp.deriv();
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.left_arm_solver.chain);
        fk.JntToCart(
            state.left_arm_solver.q,
            shared.pose_leftarm_shoulder_elbow,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_elbow"));
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.left_arm_solver.chain);
        fk.JntToCart(
            state.left_arm_solver.q,
            shared.pose_leftarm_shoulder_ee,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_ee"));
    }


    if (!state.snapshot_taken) {
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_motion_leftarm(
    motion_motion_leftarm_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_motion_leftarm(
    motion_motion_leftarm_state &state,
    shared_data &shared) {
}

inline void control_motion_motion_leftarm(
    motion_motion_leftarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // transform_twist_leftarm_shoulder_ee_platform_to_twist_leftarm_shoulder_ee_shoulder
    shared.twist_leftarm_shoulder_ee_shoulder = shared.inverse_pose_leftarm_platform_shoulder.M * shared.twist_leftarm_shoulder_ee_platform;
    // compute_inverse_pose_leftarm_platform_shoulder
    shared.inverse_pose_leftarm_platform_shoulder = shared.pose_leftarm_platform_shoulder.Inverse();
    // compute_pose_leftarm_shoulder_ee
    shared.pose_leftarm_shoulder_ee = shared.inverse_pose_leftarm_platform_shoulder * shared.pose_leftarm_platform_ee;
    // compute_pose_leftarm_shoulder_ee_distance
    shared.pose_leftarm_shoulder_ee_distance = shared.pose_leftarm_shoulder_ee.p.Norm();
    // eval_motion_leftarm_while_cstr_linvel_leftarm_shoulder_ee_vertical
    shared.twist_leftarm_shoulder_ee_shoulder_linear_z_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.twist_leftarm_shoulder_ee_shoulder.vel[2], shared.linvel_leftarm_shoulder_ee_vertical_ref);
    // eval_motion_leftarm_while_cstr_pos_leftarm_platform_elbow_height
    shared.pose_leftarm_platform_elbow_distance_z_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.pose_leftarm_platform_elbow.p[2], shared.pos_leftarm_platform_elbow_height_ref);
    // eval_motion_leftarm_while_cstr_angvel_leftarm_shoulder_ee_anteroposterior
    shared.twist_leftarm_shoulder_ee_shoulder_angular_x_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.twist_leftarm_shoulder_ee_shoulder.rot[0], shared.angvel_leftarm_shoulder_ee_anteroposterior_ref);
    // eval_motion_leftarm_while_cstr_angvel_leftarm_shoulder_ee_lateral
    shared.twist_leftarm_shoulder_ee_shoulder_angular_y_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.twist_leftarm_shoulder_ee_shoulder.rot[1], shared.angvel_leftarm_shoulder_ee_lateral_ref);
    // eval_motion_leftarm_while_cstr_linvel_leftarm_world_ee_lateral
    shared.twist_world_platform_leftarm_ee_linear_y_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.twist_world_platform_leftarm_ee.vel[1], shared.linvel_leftarm_world_ee_lateral_ref);
    // eval_motion_leftarm_while_cstr_angvel_leftarm_shoulder_ee_vertical
    shared.twist_leftarm_shoulder_ee_shoulder_angular_z_err_motion_leftarm = motion_spec::runtime::evaluate_equality_constraint(shared.twist_leftarm_shoulder_ee_shoulder.rot[2], shared.angvel_leftarm_shoulder_ee_vertical_ref);
    // eval_motion_leftarm_while_cstr_dist_leftarm_shoulder_ee
    shared.pose_leftarm_shoulder_ee_distance_err_motion_leftarm = motion_spec::runtime::evaluate_bilateral_constraint(shared.pose_leftarm_shoulder_ee_distance, shared.dist_leftarm_shoulder_ee_lower, shared.dist_leftarm_shoulder_ee_upper);
    // ctrl_dist_leftarm_shoulder_ee
    shared.force_ctrl_dist_leftarm_shoulder_ee = state.ctrl_dist_leftarm_shoulder_ee.control(shared.pose_leftarm_shoulder_ee_distance_err_motion_leftarm);
    // ctrl_angvel_leftarm_shoulder_ee_vertical
    shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_z_motion_leftarm = state.ctrl_angvel_leftarm_shoulder_ee_vertical.control(shared.twist_leftarm_shoulder_ee_shoulder_angular_z_err_motion_leftarm);
    // ctrl_linvel_leftarm_world_ee_lateral
    shared.eacc_twist_world_platform_leftarm_ee_linear_y_motion_leftarm = state.ctrl_linvel_leftarm_world_ee_lateral.control(shared.twist_world_platform_leftarm_ee_linear_y_err_motion_leftarm);
    // ctrl_angvel_leftarm_shoulder_ee_lateral
    shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_y_motion_leftarm = state.ctrl_angvel_leftarm_shoulder_ee_lateral.control(shared.twist_leftarm_shoulder_ee_shoulder_angular_y_err_motion_leftarm);
    // ctrl_angvel_leftarm_shoulder_ee_anteroposterior
    shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_x_motion_leftarm = state.ctrl_angvel_leftarm_shoulder_ee_anteroposterior.control(shared.twist_leftarm_shoulder_ee_shoulder_angular_x_err_motion_leftarm);
    // ctrl_pos_leftarm_platform_elbow_height
    shared.force_ctrl_pos_leftarm_platform_elbow_height = state.ctrl_pos_leftarm_platform_elbow_height.control(shared.pose_leftarm_platform_elbow_distance_z_err_motion_leftarm);
    // ctrl_linvel_leftarm_shoulder_ee_vertical
    shared.eacc_twist_leftarm_shoulder_ee_shoulder_linear_z_motion_leftarm = state.ctrl_linvel_leftarm_shoulder_ee_vertical.control(shared.twist_leftarm_shoulder_ee_shoulder_linear_z_err_motion_leftarm);
    // compute_direction_ctrl_pos_leftarm_platform_elbow_height
    shared.direction_ctrl_pos_leftarm_platform_elbow_height = shared.pose_leftarm_platform_elbow.p;
    shared.direction_ctrl_pos_leftarm_platform_elbow_height.Normalize();
    // compute_wrench_force_ctrl_pos_leftarm_platform_elbow_height
    shared.wrench_force_ctrl_pos_leftarm_platform_elbow_height = KDL::Wrench(shared.direction_ctrl_pos_leftarm_platform_elbow_height * shared.force_ctrl_pos_leftarm_platform_elbow_height, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pos_leftarm_platform_elbow_height);
    // compute_direction_ctrl_dist_leftarm_shoulder_ee
    shared.direction_ctrl_dist_leftarm_shoulder_ee = shared.pose_leftarm_shoulder_ee.p;
    shared.direction_ctrl_dist_leftarm_shoulder_ee.Normalize();
    // compute_wrench_force_ctrl_dist_leftarm_shoulder_ee
    shared.wrench_force_ctrl_dist_leftarm_shoulder_ee = KDL::Wrench(shared.direction_ctrl_dist_leftarm_shoulder_ee * shared.force_ctrl_dist_leftarm_shoulder_ee, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_dist_leftarm_shoulder_ee);



    KDL::SetToZero(state.left_arm_solver.f_cstr);
    {
        KDL::Frame alpha_frame_left_arm_solver_0;
        KDL::ChainFkSolverPos_recursive alpha_fk_left_arm_solver_0(*robot.left_arm_solver.chain);
        alpha_fk_left_arm_solver_0.JntToCart(
            state.left_arm_solver.q,
            alpha_frame_left_arm_solver_0,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_shoulder"));
        const KDL::Vector alpha_axis_left_arm_solver_0 =
            alpha_frame_left_arm_solver_0.M * KDL::Vector(0.0, 0.0, 1.0);
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = alpha_axis_left_arm_solver_0[0];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 0) = alpha_axis_left_arm_solver_0[1];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = alpha_axis_left_arm_solver_0[2];
    }
    state.left_arm_solver.e_acc(0) = shared.eacc_twist_leftarm_shoulder_ee_shoulder_linear_z_motion_leftarm;

    {
        KDL::Frame alpha_frame_left_arm_solver_1;
        KDL::ChainFkSolverPos_recursive alpha_fk_left_arm_solver_1(*robot.left_arm_solver.chain);
        alpha_fk_left_arm_solver_1.JntToCart(
            state.left_arm_solver.q,
            alpha_frame_left_arm_solver_1,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_shoulder"));
        const KDL::Vector alpha_axis_left_arm_solver_1 =
            alpha_frame_left_arm_solver_1.M * KDL::Vector(1.0, 0.0, 0.0);
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 1) = alpha_axis_left_arm_solver_1[0];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 1) = alpha_axis_left_arm_solver_1[1];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 1) = alpha_axis_left_arm_solver_1[2];
    }
    state.left_arm_solver.e_acc(1) = shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_x_motion_leftarm;

    {
        KDL::Frame alpha_frame_left_arm_solver_2;
        KDL::ChainFkSolverPos_recursive alpha_fk_left_arm_solver_2(*robot.left_arm_solver.chain);
        alpha_fk_left_arm_solver_2.JntToCart(
            state.left_arm_solver.q,
            alpha_frame_left_arm_solver_2,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_shoulder"));
        const KDL::Vector alpha_axis_left_arm_solver_2 =
            alpha_frame_left_arm_solver_2.M * KDL::Vector(0.0, 1.0, 0.0);
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 2) = alpha_axis_left_arm_solver_2[0];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 2) = alpha_axis_left_arm_solver_2[1];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 2) = alpha_axis_left_arm_solver_2[2];
    }
    state.left_arm_solver.e_acc(2) = shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_y_motion_leftarm;

    {
        KDL::Frame alpha_frame_left_arm_solver_3;
        KDL::ChainFkSolverPos_recursive alpha_fk_left_arm_solver_3(*robot.left_arm_solver.chain);
        alpha_fk_left_arm_solver_3.JntToCart(
            state.left_arm_solver.q,
            alpha_frame_left_arm_solver_3,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_ee"));
        const KDL::Vector alpha_axis_left_arm_solver_3 =
            alpha_frame_left_arm_solver_3.M * KDL::Vector(0.0, 1.0, 0.0);
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 3) = alpha_axis_left_arm_solver_3[0];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 3) = alpha_axis_left_arm_solver_3[1];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 3) = alpha_axis_left_arm_solver_3[2];
    }
    state.left_arm_solver.e_acc(3) = shared.eacc_twist_world_platform_leftarm_ee_linear_y_motion_leftarm;

    {
        KDL::Frame alpha_frame_left_arm_solver_4;
        KDL::ChainFkSolverPos_recursive alpha_fk_left_arm_solver_4(*robot.left_arm_solver.chain);
        alpha_fk_left_arm_solver_4.JntToCart(
            state.left_arm_solver.q,
            alpha_frame_left_arm_solver_4,
            motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "frame_leftarm_shoulder"));
        const KDL::Vector alpha_axis_left_arm_solver_4 =
            alpha_frame_left_arm_solver_4.M * KDL::Vector(0.0, 0.0, 1.0);
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 4) = alpha_axis_left_arm_solver_4[0];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = alpha_axis_left_arm_solver_4[1];
        state.left_arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 4) = alpha_axis_left_arm_solver_4[2];
    }
    state.left_arm_solver.e_acc(4) = shared.eacc_twist_leftarm_shoulder_ee_shoulder_angular_z_motion_leftarm;
    KDL::SetToZero(state.left_arm_solver.tau_ff);
    for (int i = 0; i < state.left_arm_solver.num_segments; ++i) {
        KDL::SetToZero(state.left_arm_solver.f_ext[i]);
    }
    state.left_arm_solver.f_ext[motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "link_leftarm_elbow") - 1] += shared.wrench_force_ctrl_pos_leftarm_platform_elbow_height;
    state.left_arm_solver.f_ext[motion_spec::runtime::find_segment_index(*robot.left_arm_solver.chain, "link_leftarm_ee") - 1] += shared.wrench_force_ctrl_dist_leftarm_shoulder_ee;
    KDL::JntArray tau_ctrl_fext_left_arm_solver(state.left_arm_solver.num_joints);
    state.left_arm_solver.achd_fext->CartToJnt(
        state.left_arm_solver.q,
        state.left_arm_solver.qd,
        state.left_arm_solver.qdd,
        state.left_arm_solver.f_cstr,
        state.left_arm_solver.e_acc,
        state.left_arm_solver.f_ext,
        state.left_arm_solver.tau_ff,
        tau_ctrl_fext_left_arm_solver);
    KDL::Wrenches f_ext_zero_left_arm_solver(state.left_arm_solver.num_segments);
    KDL::JntArray tau_ctrl_acc_left_arm_solver(state.left_arm_solver.num_joints);
    state.left_arm_solver.achd_acc->CartToJnt(
        state.left_arm_solver.q,
        state.left_arm_solver.qd,
        state.left_arm_solver.qdd,
        state.left_arm_solver.f_cstr,
        state.left_arm_solver.e_acc,
        f_ext_zero_left_arm_solver,
        state.left_arm_solver.tau_ff,
        tau_ctrl_acc_left_arm_solver);
    KDL::Add(tau_ctrl_fext_left_arm_solver, tau_ctrl_acc_left_arm_solver, state.left_arm_solver.tau_ctrl);

}

inline void apply_motion_motion_leftarm(
    motion_motion_leftarm_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.left_arm_solver.num_joints; ++i) {
        robot.left_arm_solver.state->eff_cmd[i] = state.left_arm_solver.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.left_arm_solver.robot);
}
