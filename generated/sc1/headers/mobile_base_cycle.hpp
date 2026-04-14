#pragma once

#include "shared_state.hpp"

struct mobile_base_state {
    bool initialized = false;
};

inline void reset_mobile_base(mobile_base_state &state) {
    state = mobile_base_state{};
}

inline void init_mobile_base(mobile_base_state &state, shared_data &shared, const robot_io &robot) {
    state.initialized = true;
}

inline void update_mobile_base(
    mobile_base_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_mobile_base(state, shared, robot);
    {
        auto &mobile_base = shared.mobile_base;
        auto &ecat = *robot.mobile_base.ethercat;
        auto &drive_enc = *robot.mobile_base.drive_encoder;
        auto &kelo_msr = *robot.mobile_base.measurement;

        robif2b_ethercat_update(&ecat);
        robif2b_kelo_drive_encoder_update(&drive_enc);
        hddc2b_pltf_frc_comp_mat(NUM_DRIVES, mobile_base.drive_attachment, kelo_msr.pvt_pos, mobile_base.g);
        hddc2b_whl_vel_hub_to_gnd(NUM_DRIVES, mobile_base.wheel_diameter, kelo_msr.whl_vel, mobile_base.xd_ground);
        hddc2b_drv_vel_gnd_to_pvt(NUM_DRIVES, mobile_base.wheel_distance, mobile_base.castor_offset, mobile_base.xd_ground, mobile_base.xd_drive);
        motion_spec::runtime::hddc2b_pltf_vel_drv_to_pltf(NUM_DRIVES, EPS, mobile_base.g, mobile_base.w_drive, mobile_base.xd_drive, mobile_base.w_platform, mobile_base.xd_platform);
        shared.twist_world_platform_platform.vel.x(mobile_base.xd_platform[0]);
        shared.twist_world_platform_platform.vel.y(mobile_base.xd_platform[1]);
        shared.twist_world_platform_platform.rot.z(mobile_base.xd_platform[2]);

    }
}

inline void control_mobile_base(
    mobile_base_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        auto &mobile_base = shared.mobile_base;
        auto &kelo_msr = *robot.mobile_base.measurement;
        auto &kelo_cmd = *robot.mobile_base.command;
        mobile_base.f_platform[0] = shared.wrench_dist_platform.force.x();
        mobile_base.f_platform[1] = shared.wrench_dist_platform.force.y();
        mobile_base.f_platform[2] = shared.wrench_dist_platform.torque.z();
        hddc2b_pltf_drv_algn_dst(NUM_DRIVES, mobile_base.drive_attachment, mobile_base.w_align, kelo_msr.pvt_pos, mobile_base.f_platform, &mobile_base.f_drive_ref[1], 2);
        hddc2b_pltf_frc_pltf_to_drv(NUM_DRIVES, EPS, mobile_base.g, mobile_base.w_platform, mobile_base.f_platform, mobile_base.w_drive, mobile_base.f_drive_ref, mobile_base.f_prim, mobile_base.f_scnd);
        motion_spec::runtime::hddc2b_rescale(mobile_base.f_scnd);
        hddc2b_pltf_frc_redu_ref_fini(NUM_DRIVES, mobile_base.f_prim, mobile_base.f_scnd, mobile_base.f_drive);
        hddc2b_drv_frc_pvt_to_gnd(NUM_DRIVES, mobile_base.wheel_distance, mobile_base.castor_offset, mobile_base.f_drive, mobile_base.f_wheel);
        hddc2b_whl_frc_gnd_to_hub(NUM_DRIVES, mobile_base.wheel_diameter, mobile_base.f_wheel, kelo_cmd.trq);

    }
}

inline void apply_mobile_base(
    mobile_base_state &state,
    shared_data &shared,
    const robot_io &robot) {
    auto &wheel_act = *robot.mobile_base.wheel_actuator;
    robif2b_kelo_drive_actuator_update(&wheel_act);
}
