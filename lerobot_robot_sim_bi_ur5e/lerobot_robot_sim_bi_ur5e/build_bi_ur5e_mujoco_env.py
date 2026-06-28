"""Build the MuJoCo scene for the bimanual UR5e simulation."""

from pathlib import Path

import mujoco
from dm_control import mjcf


DEFAULT_OBJECT_FRICTION = "1.6 0.08 0.02"
BROOM_MESH_SIZE = (0.125, 0.1123, 0.02)
DUSTPAN_MESH_SIZE = (0.124, 0.137, 0.05)


def attach_gripper_to_arm(arm_mjcf: mjcf.RootElement, gripper_mjcf: mjcf.RootElement) -> None:
    attachment_site = arm_mjcf.find("site", "attachment_site")
    if attachment_site is None:
        raise ValueError("attachment_site not found in UR5e MJCF")
    attachment_site.attach(gripper_mjcf)


def add_wrist_camera(arm_mjcf: mjcf.RootElement, camera_name: str) -> None:
    wrist_3_link = arm_mjcf.find("body", "wrist_3_link")
    if wrist_3_link is None:
        raise ValueError("wrist_3_link not found in UR5e MJCF")

    wrist_3_link.add(
        "camera",
        name=camera_name,
        mode="fixed",
        pos="0 -0.08 0.02",
        euler="2.8 0 0",
        fovy="60",
    )


def add_lights_and_floor(arena: mjcf.RootElement) -> None:
    arena.asset.add("material", name="floor_material", rgba="0.22 0.23 0.24 1")
    arena.worldbody.add(
        "light",
        name="key_light",
        pos="1.6 -1.2 2.8",
        dir="-0.4 0.25 -1",
        directional="true",
        diffuse="0.9 0.9 0.86",
        specular="0.25 0.25 0.25",
        castshadow="true",
    )
    arena.worldbody.add(
        "light",
        name="fill_light",
        pos="-1.4 1.1 1.8",
        dir="0.35 -0.25 -1",
        directional="true",
        diffuse="0.35 0.38 0.42",
        specular="0.05 0.05 0.05",
        castshadow="false",
    )
    arena.worldbody.add(
        "geom",
        name="floor",
        type="plane",
        pos="0 0 0",
        size="3 3 0.05",
        material="floor_material",
    )


def add_table(
    arena: mjcf.RootElement,
    table_size: tuple[float, float, float],
    table_height: float,
) -> None:
    table_x, table_y, table_thickness = table_size
    half_x = table_x / 2.0
    half_y = table_y / 2.0
    half_z = table_thickness / 2.0
    leg_height = table_height - table_thickness
    leg_half_height = leg_height / 2.0

    arena.asset.add("material", name="table_top_material", rgba="0.52 0.42 0.32 1")
    arena.asset.add("material", name="table_leg_material", rgba="0.32 0.32 0.34 1")
    table_body = arena.worldbody.add("body", name="table_body", pos="0 0 0")
    table_body.add(
        "geom",
        name="table_top",
        type="box",
        pos=f"0 0 {table_height - half_z}",
        size=f"{half_x} {half_y} {half_z}",
        material="table_top_material",
        friction="0.001 0.001 0.0001",
    )

    leg_x = max(half_x - 0.07, 0.0)
    leg_y = max(half_y - 0.07, 0.0)
    for leg_name, x, y in (
        ("table_leg_front_left", leg_x, leg_y),
        ("table_leg_front_right", leg_x, -leg_y),
        ("table_leg_back_left", -leg_x, leg_y),
        ("table_leg_back_right", -leg_x, -leg_y),
    ):
        table_body.add(
            "geom",
            name=leg_name,
            type="box",
            pos=f"{x} {y} {leg_half_height}",
            size=f"0.035 0.035 {leg_half_height}",
            material="table_leg_material",
        )


def add_cubes(
    arena: mjcf.RootElement,
    cube_count: int,
    cube_size: float,
    table_height: float,
) -> None:
    colors = (
        "0.86 0.22 0.18 1",
        "0.12 0.45 0.90 1",
        "0.12 0.70 0.34 1",
        "0.94 0.68 0.16 1",
        "0.58 0.28 0.82 1",
        "0.05 0.62 0.68 1",
        "0.90 0.35 0.55 1",
        "0.45 0.50 0.18 1",
        "0.20 0.20 0.22 1",
        "0.86 0.86 0.78 1",
    )
    half = cube_size / 2.0
    xs = (-0.36, -0.18, 0.0, 0.18, 0.36)
    ys = (-0.13, 0.13)

    for i in range(cube_count):
        material_name = f"cube_{i}_material"
        arena.asset.add("material", name=material_name, rgba=colors[i % len(colors)])
        x = xs[i % len(xs)]
        y = ys[(i // len(xs)) % len(ys)]
        cube = arena.worldbody.add(
            "body",
            name=f"cube_{i}_body",
            pos=f"{x} {y} {table_height + half + 0.002}",
        )
        cube.add("freejoint", name=f"cube_{i}_freejoint")
        cube.add(
            "geom",
            name=f"cube_{i}_geom",
            type="box",
            size=f"{half} {half} {half}",
            mass="0.06",
            # friction=DEFAULT_OBJECT_FRICTION,
            material=material_name,
            solref="0.01 1",
            solimp="0.95 0.99 0.001",
            friction="0.1 0.1 0.0001",
        )


def add_cleaning_tools(
    arena: mjcf.RootElement,
    assets_dir: str | Path,
    table_height: float,
    show_collision: bool = False,
    attached_broom_side: str | None = None,
    attached_dustpan_side: str | None = None,
) -> None:
    assets_dir = Path(assets_dir)
    broom_path = assets_dir / "Office_Broom.stl"
    dustpan_path = assets_dir / "Dustpan.stl"
    if not broom_path.is_file() or not dustpan_path.is_file():
        return

    arena.asset.add("material", name="broom_material", rgba="0.72 0.50 0.28 1")
    arena.asset.add("material", name="dustpan_material", rgba="0.15 0.38 0.72 1")
    collision_rgba = "0.95 0.15 0.10 0.35" if show_collision else "0 0 0 0"
    arena.asset.add(
        "mesh",
        name="broom_visual_mesh",
        file=str(broom_path),
        scale="0.001 0.001 0.001",
    )
    arena.asset.add(
        "mesh",
        name="dustpan_visual_mesh",
        file=str(dustpan_path),
        scale="0.001 0.001 0.001",
    )

    if attached_broom_side is None:
        add_table_broom(arena, table_height=table_height, collision_rgba=collision_rgba)

    if attached_dustpan_side is None:
        add_table_dustpan(arena, table_height=table_height, collision_rgba=collision_rgba)


def add_table_dustpan(
    arena: mjcf.RootElement,
    table_height: float,
    collision_rgba: str,
) -> None:
    dustpan_half_x = DUSTPAN_MESH_SIZE[0] / 2.0
    dustpan_half_y = DUSTPAN_MESH_SIZE[1] / 2.0
    dustpan_half_z = DUSTPAN_MESH_SIZE[2] / 2.0
    dustpan = arena.worldbody.add(
        "body",
        name="dustpan_body",
        pos=f"0.18 -0.25 {table_height + dustpan_half_z + 0.004}",
        euler="0 0 -0.25",
    )
    dustpan.add("freejoint", name="dustpan_freejoint")
    add_dustpan_visual_geom(
        dustpan,
        mesh_name="dustpan_visual_mesh",
        material_name="dustpan_material",
    )
    add_dustpan_collision_geoms(dustpan, collision_rgba=collision_rgba)


def add_dustpan_visual_geom(parent, mesh_name: str, material_name: str) -> None:
    dustpan_half_x = DUSTPAN_MESH_SIZE[0] / 2.0
    dustpan_half_y = DUSTPAN_MESH_SIZE[1] / 2.0
    dustpan_half_z = DUSTPAN_MESH_SIZE[2] / 2.0
    parent.add(
        "geom",
        name="dustpan_visual",
        type="mesh",
        mesh=mesh_name,
        pos=f"-{dustpan_half_x} -{dustpan_half_y} -{dustpan_half_z}",
        material=material_name,
        contype="0",
        conaffinity="0",
        group="2",
    )


def add_dustpan_collision_geoms(parent, collision_rgba: str) -> None:
    dustpan_half_x = DUSTPAN_MESH_SIZE[0] / 2.0
    dustpan_half_y = DUSTPAN_MESH_SIZE[1] / 2.0
    dustpan_half_z = DUSTPAN_MESH_SIZE[2] / 2.0
    floor_half_z = 0.003
    wall_half_thickness = 0.004
    side_wall_half_z = 0.017
    back_wall_half_z = 0.024

    collision_geoms = (
        {
            "name": "dustpan_floor_collision",
            "pos": f"0 -0.006 {-dustpan_half_z + floor_half_z}",
            "size": f"{dustpan_half_x * 0.88} {dustpan_half_y * 0.78} {floor_half_z}",
            "mass": "0.07",
        },
        {
            "name": "dustpan_back_wall_collision",
            "pos": f"0 {dustpan_half_y - wall_half_thickness} {-dustpan_half_z + back_wall_half_z}",
            "size": f"{dustpan_half_x * 0.92} {wall_half_thickness} {back_wall_half_z}",
            "mass": "0.04",
        },
        {
            "name": "dustpan_left_wall_collision",
            "pos": f"{-dustpan_half_x + wall_half_thickness} 0 {-dustpan_half_z + side_wall_half_z}",
            "size": f"{wall_half_thickness} {dustpan_half_y * 0.82} {side_wall_half_z}",
            "mass": "0.025",
        },
        {
            "name": "dustpan_right_wall_collision",
            "pos": f"{dustpan_half_x - wall_half_thickness} 0 {-dustpan_half_z + side_wall_half_z}",
            "size": f"{wall_half_thickness} {dustpan_half_y * 0.82} {side_wall_half_z}",
            "mass": "0.025",
        },
    )

    for geom_kwargs in collision_geoms:
        parent.add(
            "geom",
            type="box",
            friction=DEFAULT_OBJECT_FRICTION,
            rgba=collision_rgba,
            group="3",
            **geom_kwargs,
        )


def add_table_broom(
    arena: mjcf.RootElement,
    table_height: float,
    collision_rgba: str,
) -> None:
    broom_half_x = BROOM_MESH_SIZE[0] / 2.0
    broom_half_y = BROOM_MESH_SIZE[1] / 2.0
    broom_half_z = BROOM_MESH_SIZE[2] / 2.0
    broom = arena.worldbody.add(
        "body",
        name="broom_body",
        pos=f"-0.22 -0.25 {table_height + broom_half_z + 0.004}",
        euler="0 0 0.35",
    )
    broom.add("freejoint", name="broom_freejoint")
    add_broom_geoms(
        broom,
        mesh_name="broom_visual_mesh",
        material_name="broom_material",
        collision_rgba=collision_rgba,
    )


def add_broom_geoms(
    parent,
    mesh_name: str,
    material_name: str,
    collision_rgba: str,
) -> None:
    broom_half_x = BROOM_MESH_SIZE[0] / 2.0
    broom_half_y = BROOM_MESH_SIZE[1] / 2.0
    broom_half_z = BROOM_MESH_SIZE[2] / 2.0
    parent.add(
        "geom",
        name="broom_visual",
        type="mesh",
        mesh=mesh_name,
        pos=f"-{broom_half_x} -{broom_half_y} -{broom_half_z}",
        material=material_name,
        contype="0",
        conaffinity="0",
        group="2",
    )
    parent.add(
        "geom",
        name="broom_collision",
        type="box",
        size=f"{broom_half_x} {broom_half_y} {broom_half_z}",
        mass="0.12",
        friction=DEFAULT_OBJECT_FRICTION,
        rgba=collision_rgba,
        group="3",
    )


def attach_broom_to_gripper(
    gripper_mjcf: mjcf.RootElement,
    assets_dir: str | Path,
    side: str,
    show_collision: bool = False,
) -> None:
    gripper_base = gripper_mjcf.find("body", "base")
    if gripper_base is None:
        raise ValueError("base body not found in Robotiq MJCF")

    broom_path = Path(assets_dir) / "Office_Broom.stl"
    if not broom_path.is_file():
        raise FileNotFoundError(f"Broom STL not found: {broom_path}")

    material_name = f"{side}_attached_broom_material"
    mesh_name = f"{side}_attached_broom_visual_mesh"
    gripper_mjcf.asset.add("material", name=material_name, rgba="0.72 0.50 0.28 1")
    gripper_mjcf.asset.add(
        "mesh",
        name=mesh_name,
        file=str(broom_path),
        scale="0.001 0.001 0.001",
    )
    collision_rgba = "0.95 0.15 0.10 0.35" if show_collision else "0 0 0 0"
    broom = gripper_base.add(
        "body",
        name="attached_broom_body",
        pos="0.0 0 0.2",
        euler="1.5708 1.5708 0",
    )
    add_broom_geoms(
        broom,
        mesh_name=mesh_name,
        material_name=material_name,
        collision_rgba=collision_rgba,
    )


def attach_dustpan_to_gripper(
    gripper_mjcf: mjcf.RootElement,
    assets_dir: str | Path,
    side: str,
    show_collision: bool = False,
) -> None:
    gripper_base = gripper_mjcf.find("body", "base")
    if gripper_base is None:
        raise ValueError("base body not found in Robotiq MJCF")

    dustpan_path = Path(assets_dir) / "Dustpan.stl"
    if not dustpan_path.is_file():
        raise FileNotFoundError(f"Dustpan STL not found: {dustpan_path}")

    material_name = f"{side}_attached_dustpan_material"
    mesh_name = f"{side}_attached_dustpan_visual_mesh"
    gripper_mjcf.asset.add("material", name=material_name, rgba="0.15 0.38 0.72 1")
    gripper_mjcf.asset.add(
        "mesh",
        name=mesh_name,
        file=str(dustpan_path),
        scale="0.001 0.001 0.001",
    )
    collision_rgba = "0.95 0.15 0.10 0.35" if show_collision else "0 0 0 0"
    dustpan = gripper_base.add(
        "body",
        name="attached_dustpan_body",
        pos="0.0 -0.085 0.165",
        euler="3.14159 0 3.14159",
    )
    add_dustpan_visual_geom(
        dustpan,
        mesh_name=mesh_name,
        material_name=material_name,
    )
    add_dustpan_collision_geoms(dustpan, collision_rgba=collision_rgba)


def add_global_camera(arena: mjcf.RootElement) -> None:
    arena.worldbody.add(
        "camera",
        name="global",
        mode="fixed",
        pos="1.35 -1.45 1.45",
        xyaxes="0.732 0.681 0 -0.429 0.461 0.777",
        fovy="45",
    )


def build_single_arm(
    arm_xml_path: str | Path,
    gripper_xml_path: str | Path,
    side: str,
    base_pos: tuple[float, float, float],
    base_quat: tuple[float, float, float, float],
    assets_dir: str | Path | None = None,
    attach_broom: bool = False,
    attach_dustpan: bool = False,
    show_tool_collision: bool = False,
) -> mjcf.RootElement:
    arm_mjcf = mjcf.from_path(str(arm_xml_path))
    arm_mjcf.model = f"{side}_ur5e"
    gripper_mjcf = mjcf.from_path(str(gripper_xml_path))
    gripper_mjcf.model = f"{side}_robotiq"

    base_body = arm_mjcf.find("body", "base")
    if base_body is None:
        raise ValueError("base body not found in UR5e MJCF")
    base_body.pos = base_pos
    base_body.quat = base_quat

    add_wrist_camera(arm_mjcf, f"{side}_wrist")
    if attach_broom:
        if assets_dir is None:
            raise ValueError("assets_dir is required when attaching broom to a gripper.")
        attach_broom_to_gripper(
            gripper_mjcf,
            assets_dir=assets_dir,
            side=side,
            show_collision=show_tool_collision,
        )
    if attach_dustpan:
        if assets_dir is None:
            raise ValueError("assets_dir is required when attaching dustpan to a gripper.")
        attach_dustpan_to_gripper(
            gripper_mjcf,
            assets_dir=assets_dir,
            side=side,
            show_collision=show_tool_collision,
        )
    attach_gripper_to_arm(arm_mjcf, gripper_mjcf)
    return arm_mjcf


def build_bi_ur5e_mujoco_env(
    ur5e_xml_path: str | Path,
    robotiq_xml_path: str | Path,
    assets_dir: str | Path | None = None,
    table_size: tuple[float, float, float] = (1.2, 0.75, 0.05),
    table_height: float = 0.75,
    cube_count: int = 10,
    cube_size: float = 0.035,
    show_tool_collision: bool = False,
    attached_broom_side: str | None = "left",
    attached_dustpan_side: str | None = "right",
) -> mujoco.MjModel:
    if attached_broom_side not in (None, "left", "right"):
        raise ValueError("attached_broom_side must be None, 'left', or 'right'.")
    if attached_dustpan_side not in (None, "left", "right"):
        raise ValueError("attached_dustpan_side must be None, 'left', or 'right'.")
    effective_attached_broom_side = attached_broom_side if assets_dir is not None else None
    effective_attached_dustpan_side = attached_dustpan_side if assets_dir is not None else None

    arena = mjcf.RootElement(model="bi_ur5e_tabletop")
    arena.option.timestep = 0.002
    arena.option.integrator = "implicitfast"

    add_lights_and_floor(arena)
    add_table(arena, table_size=table_size, table_height=table_height)
    add_cubes(arena, cube_count=cube_count, cube_size=cube_size, table_height=table_height)
    if assets_dir is not None:
        add_cleaning_tools(
            arena,
            assets_dir=assets_dir,
            table_height=table_height,
            show_collision=show_tool_collision,
            attached_broom_side=effective_attached_broom_side,
            attached_dustpan_side=effective_attached_dustpan_side,
        )
    add_global_camera(arena)

    left_arm = build_single_arm(
        arm_xml_path=ur5e_xml_path,
        gripper_xml_path=robotiq_xml_path,
        side="left",
        base_pos=(-0.48, 0.62, 0.0),
        base_quat=(0.7071068, 0.0, 0.0, -0.7071068),
        assets_dir=assets_dir,
        attach_broom=effective_attached_broom_side == "left",
        attach_dustpan=effective_attached_dustpan_side == "left",
        show_tool_collision=show_tool_collision,
    )
    right_arm = build_single_arm(
        arm_xml_path=ur5e_xml_path,
        gripper_xml_path=robotiq_xml_path,
        side="right",
        base_pos=(0.48, 0.62, 0.0),
        base_quat=(0.7071068, 0.0, 0.0, -0.7071068),
        assets_dir=assets_dir,
        attach_broom=effective_attached_broom_side == "right",
        attach_dustpan=effective_attached_dustpan_side == "right",
        show_tool_collision=show_tool_collision,
    )
    arena.worldbody.attach(left_arm)
    arena.worldbody.attach(right_arm)

    assets: dict[str, bytes] = {}
    for asset in arena.asset.all_children():
        if asset.tag == "mesh":
            mesh_file = asset.file
            assets[mesh_file.get_vfs_filename()] = mesh_file.contents

    return mujoco.MjModel.from_xml_string(arena.to_xml_string(), assets)
