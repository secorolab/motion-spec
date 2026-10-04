# SPDX-License-Identifier: MPL-2.0
"""Load a .scenex in mjkdl and view it, without generating or building anything.

The scene comes from `motion_spec.rdf_parser.scene.read_scene` -- the same reader the
generated C++ uses -- so what this shows is what a run of the model would compose: robots,
attachments, objects and placement. Given a robot config, every driven chain is put at its
start pose, which is what makes it useful for checking a mount.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

import mjkdl
import rdflib
from motion_spec_dsl.rdf_parser.vocab import AGN, GEOM_ENT
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph

from motion_spec.classes.scene import MjcfSceneSpec
from motion_spec.generation.codegen import asset_candidates
from motion_spec.rdf_parser import agents
from motion_spec.rdf_parser.model import Model
from motion_spec.rdf_parser.scene import name_object_attachments, read_scene


def scenex_model(scenex_path: Path) -> Model:
    """The .scenex as the model the scene readers take, named by the file it was read from."""
    graph = rdflib.Dataset(default_union=True)
    named = graph.graph(rdflib.URIRef(scenex_path.resolve().as_uri()))
    named += create_scenex_model_graph(scenex_metamodel().model_from_file(str(scenex_path)))
    return Model(graph=graph, app_path=scenex_path.resolve())


def home_poses(model: Model, config_path: Path):
    """Each driven chain's start pose from a robot config, as (root, tip, prefix, q).

    The config's `[<agent>] home` is what a run's reset writes into the chain that agent
    drives, so the viewer resolves the chains the same way the scene reader does.
    """
    with config_path.open("rb") as config_file:
        config = tomllib.load(config_file)
    bound_trees = agents.mapped_targets(model, AGN["AgentModel"], GEOM_ENT.KinematicTree)
    attach_by_body, _root = agents.fixed_attachments(model, bound_trees)
    name_object_attachments(model, attach_by_body)
    poses = []
    for assembly in agents.agent_assemblies(model, attach_by_body):
        section = config
        for key in assembly.config_key.split("."):
            section = section.get(key, {}) if isinstance(section, dict) else {}
        home = section.get("home") if isinstance(section, dict) else None
        if home:
            poses.append(
                (assembly.chain_root, assembly.tip, assembly.prefix, [float(v) for v in home])
            )
    return poses


def apply_home_poses(env: mjkdl.Env, poses) -> list:
    """Put each chain at its home pose and hold it there in position mode; returns the robots,
    which the env drives only while they live.

    The prefix is tried and then dropped: where a scene names its bodies `left_arm/base_link`
    the joints are already `left_arm_joint_1`, and prefixing again looks for a joint called
    `left_arm_left_arm_joint_1` that no model has.
    """
    robots = []
    for root, tip, prefix, home in poses:
        for attempt in (prefix, ""):
            try:
                robot = env.create_robot(root, tip, attempt)
                break
            except RuntimeError:
                robot = None
        if robot is None:
            print(f"  (no chain {root} -> {tip}; home pose not applied)")
            continue
        q = home[: robot.n_joints]
        robot.set_joint_pos(q)
        robot.set_control_mode(mjkdl.CtrlMode.POSITION)
        robot.jnt_pos_cmd = q
        robots.append(robot)
    return robots


def find_asset(relative: str, start: Path) -> str:
    """An authored asset path as a file on this machine.

    Searched from the working directory and from the .scenex upwards -- the paths are
    workspace-relative -- then in the mjkdl caches the assets are fetched into.

    Raises:
        FileNotFoundError: no candidate exists, so the scene cannot be composed.
    """
    path = Path(relative)
    if path.is_absolute():
        if path.exists():
            return str(path)
        raise FileNotFoundError(relative)

    for root in (Path.cwd(), start.resolve().parent):
        for base in (root, *root.parents):
            if (base / path).exists():
                return str(base / path)

    cached = asset_candidates(relative)[1:]
    for candidate in cached:
        if candidate.exists():
            return str(candidate)

    raise FileNotFoundError(
        f"asset '{relative}' was not found from {Path.cwd()}, from {start.resolve().parent} "
        f"or in the mjkdl cache ({', '.join(map(str, cached)) or 'no HOME'}); install mjkdl"
    )


def _target(kind: str, name: str) -> mjkdl.AttachTarget:
    return mjkdl.AttachTarget(getattr(mjkdl.AttachKind, kind), name)


def build_spec(scene: MjcfSceneSpec, start: Path) -> mjkdl.SceneSpec:
    """The scene as a wrapper SceneSpec, with every authored asset path resolved."""
    spec = mjkdl.SceneSpec()
    spec.timestep = scene.timestep_s
    spec.add_skybox = True
    spec.add_floor = True

    robots = []
    for robot in scene.robots:
        robot_spec = mjkdl.RobotSpec()
        robot_spec.path = find_asset(robot.path, start)
        robot_spec.prefix = robot.prefix
        robot_spec.attach_to = _target(robot.attach_kind, robot.attach_name)
        robot_spec.pos = [robot.pos_x, robot.pos_y, robot.pos_z]
        robot_spec.quat = [robot.quat_x, robot.quat_y, robot.quat_z, robot.quat_w]
        attachments = []
        for attachment in robot.attachments:
            attachment_spec = mjkdl.AttachmentSpec()
            attachment_spec.mjcf_path = find_asset(attachment.path, start)
            attachment_spec.attach_to = _target(attachment.attach_kind, attachment.attach_to)
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
        robots.append(robot_spec)
    spec.robots = robots

    objects = []
    for obj in scene.objects:
        object_spec = mjkdl.SceneObject()
        object_spec.name = obj.body
        # As backend/mj_kdl/robot.stg names it: the scene's mappings use the prefixed element names.
        object_spec.prefix = f"{obj.body}_"
        object_spec.attach_to = _target(obj.attach_kind, obj.attach_name)
        object_spec.pos = [obj.pos_x, obj.pos_y, obj.pos_z]
        object_spec.quat = [obj.quat_x, obj.quat_y, obj.quat_z, obj.quat_w]
        object_spec.fixed = obj.fixed
        if obj.has_path:
            object_spec.mjcf_path = find_asset(obj.path, start)
            if obj.color is not None:
                object_spec.rgba = obj.color
        else:
            object_spec.shape = getattr(mjkdl.Shape, obj.shape)
            object_spec.size = obj.size
            object_spec.rgba = obj.color
            object_spec.mass = obj.mass
            object_spec.condim = mjkdl.Condim.Rolling
            object_spec.friction = obj.friction
        objects.append(object_spec)
    spec.objects = objects

    sites = []
    for frame in scene.frames:
        site = mjkdl.SiteSpec()
        site.body = frame.body
        site.name = frame.name
        site.pos = [frame.pos_x, frame.pos_y, frame.pos_z]
        site.quat = [frame.quat_x, frame.quat_y, frame.quat_z, frame.quat_w]
        sites.append(site)
    spec.sites = sites

    cameras = []
    for camera in scene.static_cameras:
        camera_spec = mjkdl.CameraSpec()
        camera_spec.name = camera.name
        camera_spec.body = camera.body
        camera_spec.pos = [camera.pos_x, camera.pos_y, camera.pos_z]
        camera_spec.quat = [camera.quat_x, camera.quat_y, camera.quat_z, camera.quat_w]
        camera_spec.fovy = camera.fovy_deg
        cameras.append(camera_spec)
    spec.cameras = cameras

    return spec


def describe(spec: mjkdl.SceneSpec, env: mjkdl.Env) -> None:
    print(f"timestep: {spec.timestep} s, floor at the anchor (z = {spec.floor_z} m)")
    for robot in spec.robots:
        print(f"robot: {robot.path} @ {robot.attach_to.kind} '{robot.attach_to.name}'")
        print(f"       pos {list(robot.pos)} quat {list(robot.quat)}")
        for attachment in robot.attachments:
            print(
                f"  attachment: {attachment.mjcf_path} @ "
                f"{attachment.attach_to.kind} '{attachment.attach_to.name}'"
            )
            print(f"       pos {list(attachment.pos)} quat {list(attachment.quat)}")
    for obj in spec.objects:
        print(f"object: {obj.name} {obj.mjcf_path or obj.shape} at {list(obj.pos)}")
    print(f"cameras: {[env.model.camera(i).name for i in range(env.model.ncam)]}")


def view(scenex_path: Path, config: Path | None, headless: bool) -> None:
    """Compose SCENEX_PATH, describe it, and open the viewer unless HEADLESS."""
    model = scenex_model(scenex_path)
    poses = home_poses(model, config) if config else []
    _setups, _ordered, trees = agents.robot_setups(model)
    spec = build_spec(read_scene(model, trees), scenex_path)
    env = mjkdl.Env.build(spec)
    try:
        _robots = apply_home_poses(env, poses)
        describe(spec, env)
        if headless:
            return
        env.open_viewer(scenex_path.name)
        while True:
            env.update()
            if not env.step():
                break
            env.pace()  # step() never sleeps; an unpaced loop starves the render thread
    finally:
        env.close()
