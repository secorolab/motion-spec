# SPDX-License-Identifier: MPL-2.0
"""Load a .scenex in mj_kdl_wrapper and view it, without generating or building anything.

The scene comes from `motion_spec.rdf_parser.scene.read_scene` -- the same reader the
generated C++ uses -- so what this shows is what a run of the model would compose: robots,
attachments, objects and placement. Given a robot config, every driven chain is put at its
start pose, which is what makes it useful for checking a mount.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

import mj_kdl_wrapper as mjk
import rdflib
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.rdf_parser import agents
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.scene import name_object_attachments, read_scene


def find_asset(path: str) -> str:
    """An asset file the scene states, checked to be on this machine.

    Raises:
        FileNotFoundError: it is not there, so the scene cannot be composed.
    """
    if not Path(path).is_file():
        raise FileNotFoundError(f"asset '{path}' the scene names is not there")
    return path


def view(scenex_path: Path, config: Path | None, headless: bool) -> None:
    """Compose SCENEX_PATH, describe it, and open the viewer unless HEADLESS."""
    graph = rdflib.Dataset(default_union=True)
    graph.add_graph(create_scenex_model_graph(scenex_metamodel().model_from_file(str(scenex_path))))
    model = Model(graph=graph)
    poses = home_poses(model, config) if config else []
    _setups, trees = agents.robot_setups(model)
    spec = scene_spec(read_scene(model, trees))
    env = mjk.Env.build(spec)
    try:
        # kept alive: the env drives a robot only while it lives
        robots = []
        for root, tip, prefix, home in poses:
            robot = chain_robot(env, root, tip, prefix)
            if robot is None:
                print(f"  (no chain {root} -> {tip}; home pose not applied)")
                continue
            q = home[: robot.n_joints]
            robot.set_joint_pos(q)
            robot.set_control_mode(mjk.CtrlMode.POSITION)
            robot.jnt_pos_cmd = q
            robots.append(robot)
        describe(spec, env)
        if headless:
            return
        env.open_viewer(scenex_path.name)
        while True:
            env.update()
            if not env.step():
                break
            env.pace()  # step() never sleeps; unpaced, the render thread starves
    finally:
        env.close()


def describe(spec: mjk.SceneSpec, env: mjk.Env) -> None:
    """Print what SPEC composed in ENV: timestep, robots and attachments, objects and cameras."""
    print(f"timestep: {spec.timestep} s, floor at the anchor (z = {spec.floor_z} m)")
    for robot_spec in spec.robots:
        print(
            f"robot: {robot_spec.path} @ "
            f"{robot_spec.attach_to.kind} '{robot_spec.attach_to.name}'"
        )
        print(f"       pos {list(robot_spec.pos)} quat {list(robot_spec.quat)}")
        for attachment_spec in robot_spec.attachments:
            print(
                f"  attachment: {attachment_spec.mjcf_path} @ "
                f"{attachment_spec.attach_to.kind} '{attachment_spec.attach_to.name}'"
            )
            print(f"       pos {list(attachment_spec.pos)} quat {list(attachment_spec.quat)}")
    for object_spec in spec.objects:
        print(
            f"object: {object_spec.name} {object_spec.mjcf_path or object_spec.shape} "
            f"at {list(object_spec.pos)}"
        )
    print(f"cameras: {[env.model.camera(i).name for i in range(env.model.ncam)]}")


def home_poses(model: Model, config: Path) -> list[tuple[str, str, str, list[float]]]:
    """Each assembly's chain root, tip, prefix and the `home` its section of the robot CONFIG
    states; an assembly whose section states none is left out."""
    robot_config = tomllib.loads(config.read_text())
    attach_by_body, _root = agents.fixed_attachments(model)
    name_object_attachments(model, attach_by_body)
    poses = []
    for assembly in agents.agent_assemblies(model, attach_by_body):
        section = robot_config
        for key in assembly.config_key.split("."):
            section = section.get(key, {}) if isinstance(section, dict) else {}
        home = section.get("home") if isinstance(section, dict) else None
        if home:
            poses.append(
                (assembly.chain_root, assembly.tip, assembly.prefix, [float(v) for v in home])
            )
    return poses


def chain_robot(env: mjk.Env, root: str, tip: str, prefix: str) -> mjk.Robot | None:
    """The robot driving ROOT to TIP in ENV, None when no chain joins them.

    Joint names may already carry PREFIX, so the chain is tried without it too.
    """
    for attempt in (prefix, ""):
        try:
            return env.create_robot(root, tip, attempt)
        except RuntimeError:
            pass
    return None


def scene_spec(scene) -> mjk.SceneSpec:
    """The mj_kdl_wrapper scene SCENE composes: robots and their attachments, objects, frame
    sites and static cameras, every asset checked to be on this machine."""
    spec = mjk.SceneSpec()
    spec.timestep = scene.timestep_s
    spec.add_skybox = True
    spec.add_floor = True

    robot_specs = []
    for robot in scene.robots:
        robot_spec = mjk.RobotSpec()
        robot_spec.path = find_asset(robot.path)
        robot_spec.prefix = robot.prefix
        robot_spec.attach_to = mjk.AttachTarget(
            getattr(mjk.AttachKind, robot.attach_kind), robot.attach_name
        )
        robot_spec.pos = [robot.pos_x, robot.pos_y, robot.pos_z]
        robot_spec.quat = [robot.quat_x, robot.quat_y, robot.quat_z, robot.quat_w]
        attachments = []
        for attachment in robot.attachments:
            attachment_spec = mjk.AttachmentSpec()
            attachment_spec.mjcf_path = find_asset(attachment.path)
            attachment_spec.attach_to = mjk.AttachTarget(
                getattr(mjk.AttachKind, attachment.attach_kind), attachment.attach_to
            )
            attachment_spec.prefix = attachment.prefix
            attachment_spec.pos = [attachment.pos_x, attachment.pos_y, attachment.pos_z]
            attachment_spec.quat = [
                attachment.quat_x,
                attachment.quat_y,
                attachment.quat_z,
                attachment.quat_w,
            ]
            attachments.append(attachment_spec)
        robot_spec.attachments = attachments
        robot_specs.append(robot_spec)
    spec.robots = robot_specs

    objects = []
    for obj in scene.objects:
        object_spec = mjk.SceneObject()
        object_spec.name = obj.body
        # prefixed as backend/mj_kdl/robot.stg names it
        object_spec.prefix = f"{obj.body}_"
        object_spec.attach_to = mjk.AttachTarget(
            getattr(mjk.AttachKind, obj.attach_kind), obj.attach_name
        )
        object_spec.pos = [obj.pos_x, obj.pos_y, obj.pos_z]
        object_spec.quat = [obj.quat_x, obj.quat_y, obj.quat_z, obj.quat_w]
        object_spec.fixed = obj.fixed
        if obj.has_path:
            object_spec.mjcf_path = find_asset(obj.path)
            if obj.color is not None:
                object_spec.rgba = obj.color
        else:
            object_spec.shape = getattr(mjk.Shape, obj.shape)
            object_spec.size = obj.size
            object_spec.rgba = obj.color
            object_spec.mass = obj.mass
            object_spec.condim = mjk.Condim.Rolling
            object_spec.friction = obj.friction
        objects.append(object_spec)
    spec.objects = objects

    sites = []
    for frame in scene.frames:
        site = mjk.SiteSpec()
        site.body = frame.body
        site.name = frame.name
        site.pos = [frame.pos_x, frame.pos_y, frame.pos_z]
        site.quat = [frame.quat_x, frame.quat_y, frame.quat_z, frame.quat_w]
        sites.append(site)
    spec.sites = sites

    cameras = []
    for camera in scene.static_cameras:
        camera_spec = mjk.CameraSpec()
        camera_spec.name = camera.name
        camera_spec.body = camera.body
        camera_spec.pos = [camera.pos_x, camera.pos_y, camera.pos_z]
        camera_spec.quat = [camera.quat_x, camera.quat_y, camera.quat_z, camera.quat_w]
        camera_spec.fovy = camera.fovy_deg
        cameras.append(camera_spec)
    spec.cameras = cameras
    return spec
