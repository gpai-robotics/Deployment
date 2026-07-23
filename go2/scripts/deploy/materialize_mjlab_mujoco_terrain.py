#!/usr/bin/env python3
"""Materialize MJLab procedural terrain as MuJoCo XML scenes for sim2sim.

The output scene is a normal MuJoCo XML file, so existing commands can pass it
to ``scripts/deploy/run_sim2sim.py --model-path`` without runtime changes.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MJLAB_SRC = REPO_ROOT / "reference_repos" / "mjlab" / "src"
GO2_SCENE_XML = REPO_ROOT / "reference_repos" / "mujoco_menagerie" / "unitree_go2" / "scene.xml"
GO2_ASSET_DIR = REPO_ROOT / "reference_repos" / "mujoco_menagerie" / "unitree_go2" / "assets"
UNITREE_MJLAB_GO2_SCENE_XML = (
    REPO_ROOT
    / "reference_repos"
    / "unitree_rl_mjlab"
    / "src"
    / "assets"
    / "robots"
    / "unitree_go2"
    / "xmls"
    / "scene_go2.xml"
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / "artifacts" / "mujoco_mjlab_terrains"


def _ensure_mjlab_importable() -> None:
    mjlab_src = str(MJLAB_SRC)
    if mjlab_src not in sys.path:
        sys.path.insert(0, mjlab_src)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--terrain",
        default="rough",
        help=(
            "Terrain set or preset. Built-in sets: rough, stairs, all. "
            "Any mjlab.terrains.config preset name is also accepted, e.g. "
            "random_rough, pyramid_stairs, pyramid_stairs_inv, box_random_grid."
        ),
    )
    parser.add_argument(
        "--robot-model",
        choices=("menagerie", "unitree_mjlab"),
        default="menagerie",
        help=(
            "Go2 model contract to embed. Use unitree_mjlab for the C++ "
            "unitree_rl_mjlab simulator/controller bridge."
        ),
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--rows", type=int, default=10)
    parser.add_argument("--cols", type=int, default=20)
    parser.add_argument("--size-x", type=float, default=8.0)
    parser.add_argument("--size-y", type=float, default=8.0)
    parser.add_argument("--difficulty-min", type=float, default=0.0)
    parser.add_argument("--difficulty-max", type=float, default=1.0)
    parser.add_argument(
        "--spawn-row",
        type=int,
        default=0,
        help="Terrain row whose generated spawn origin is translated to world zero.",
    )
    parser.add_argument(
        "--spawn-col",
        type=int,
        default=0,
        help="Terrain column whose generated spawn origin is translated to world zero.",
    )
    parser.add_argument(
        "--curriculum",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Override MJLab curriculum mode. Default keeps named set behavior; "
            "isolated presets default to curriculum=True."
        ),
    )
    parser.add_argument("--border-width", type=float, default=None)
    parser.add_argument(
        "--keep-menagerie-floor",
        action="store_true",
        help="Keep the original Menagerie plane under the generated terrain.",
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--name", default="", help="Optional output filename stem.")
    return parser.parse_args()


def _terrain_cfg(args: argparse.Namespace):
    _ensure_mjlab_importable()
    from mjlab.terrains.config import (  # noqa: PLC0415
        ALL_TERRAIN_PRESETS,
        ALL_TERRAINS_CFG,
        ROUGH_TERRAINS_CFG,
        STAIRS_TERRAINS_CFG,
    )
    from mjlab.terrains.terrain_generator import TerrainGeneratorCfg  # noqa: PLC0415

    terrain_key = str(args.terrain)
    if terrain_key == "rough":
        cfg = copy.deepcopy(ROUGH_TERRAINS_CFG)
    elif terrain_key == "stairs":
        cfg = copy.deepcopy(STAIRS_TERRAINS_CFG)
    elif terrain_key == "all":
        cfg = copy.deepcopy(ALL_TERRAINS_CFG)
    elif terrain_key in ALL_TERRAIN_PRESETS:
        preset = copy.deepcopy(ALL_TERRAIN_PRESETS[terrain_key](proportion=1.0))
        cfg = TerrainGeneratorCfg(
            sub_terrains={terrain_key: preset},
            curriculum=True,
            size=(float(args.size_x), float(args.size_y)),
            num_rows=int(args.rows),
            num_cols=1,
            border_width=20.0,
            add_lights=True,
        )
    else:
        valid = ["rough", "stairs", "all", *sorted(ALL_TERRAIN_PRESETS.keys())]
        raise SystemExit(f"Unknown terrain '{terrain_key}'. Valid options: {valid}")

    cfg.seed = int(args.seed)
    cfg.size = (float(args.size_x), float(args.size_y))
    cfg.num_rows = int(args.rows)
    cfg.num_cols = int(args.cols)
    cfg.difficulty_range = (float(args.difficulty_min), float(args.difficulty_max))
    if args.curriculum is not None:
        cfg.curriculum = bool(args.curriculum)
    elif terrain_key not in ("rough", "stairs", "all"):
        cfg.curriculum = True
    if args.border_width is not None:
        cfg.border_width = float(args.border_width)
    return cfg


def _remove_menagerie_floor(spec) -> None:
    floor_geoms = [geom for geom in spec.worldbody.geoms if geom.name == "floor"]
    for geom in floor_geoms:
        spec.delete(geom)


def _set_terrain_contact_defaults(spec) -> None:
    """Make generated terrain easy for runtime contact audits to identify."""
    terrain_body = spec.body("terrain")
    for geom in terrain_body.geoms:
        geom.contype = 1
        geom.conaffinity = 1
        if geom.friction[0] <= 0.0:
            geom.friction = (1.0, 0.005, 0.0001)


def _use_portable_heightfield_materials() -> None:
    """Replace buffer-backed height coloring so generated scenes serialize to XML."""
    from mjlab.terrains import heightfield_terrains  # noqa: PLC0415

    def plain_heightfield_material(spec, noise, unique_id, physical_heights, texture_size=128):
        del noise, physical_heights, texture_size
        material_name = f"hf_material_{unique_id}"
        material = spec.add_material(name=material_name)
        material.rgba = (0.32, 0.38, 0.28, 1.0)
        return material_name

    heightfield_terrains.color_by_height = plain_heightfield_material


def main() -> None:
    _ensure_mjlab_importable()
    import mujoco  # noqa: PLC0415
    from mjlab.terrains.terrain_generator import TerrainGenerator  # noqa: PLC0415

    args = _parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.robot_model == "unitree_mjlab":
        _use_portable_heightfield_materials()

    cfg = _terrain_cfg(args)
    if args.robot_model == "unitree_mjlab":
        go2_scene_xml = UNITREE_MJLAB_GO2_SCENE_XML
        go2_asset_dir = UNITREE_MJLAB_GO2_SCENE_XML.parent / "assets"
    else:
        go2_scene_xml = GO2_SCENE_XML
        go2_asset_dir = GO2_ASSET_DIR

    spec = mujoco.MjSpec.from_file(str(go2_scene_xml))
    if go2_asset_dir is not None:
        spec.compiler.meshdir = str(go2_asset_dir)
        spec.compiler.texturedir = str(go2_asset_dir)
    if not args.keep_menagerie_floor:
        _remove_menagerie_floor(spec)

    generator = TerrainGenerator(cfg)
    generator.compile(spec)
    num_rows, num_cols = generator.terrain_origins.shape[:2]
    if not 0 <= args.spawn_row < num_rows:
        raise SystemExit(f"--spawn-row must be in [0, {num_rows - 1}], got {args.spawn_row}")
    if not 0 <= args.spawn_col < num_cols:
        raise SystemExit(f"--spawn-col must be in [0, {num_cols - 1}], got {args.spawn_col}")
    selected_origin = generator.terrain_origins[args.spawn_row, args.spawn_col].copy()
    spec.body("terrain").pos = -selected_origin
    _set_terrain_contact_defaults(spec)

    model_tag = "unitree_runtime" if args.robot_model == "unitree_mjlab" else "menagerie"
    stem = args.name or (
        f"go2_mjlab_{model_tag}_{args.terrain}_seed{args.seed}_"
        f"r{args.rows}_c{args.cols}_d{args.difficulty_min:g}-{args.difficulty_max:g}"
    )
    mjb_path = output_dir / f"{stem}.mjb"
    xml_path = output_dir / f"{stem}.xml"
    json_path = output_dir / f"{stem}.json"

    model = spec.compile()
    mujoco.mj_saveModel(model, str(mjb_path), None)
    model = mujoco.MjModel.from_binary_path(str(mjb_path))

    xml_saved = False
    xml_error = None
    try:
        xml_path.write_text(spec.to_xml(), encoding="utf-8")
        mujoco.MjModel.from_xml_path(str(xml_path))
        xml_saved = True
    except Exception as exc:
        xml_path.unlink(missing_ok=True)
        xml_error = str(exc)

    manifest = {
        "model_path": str(mjb_path),
        "mjb_path": str(mjb_path),
        "xml_path": str(xml_path) if xml_saved else None,
        "xml_serialization_error": xml_error,
        "source": "mjlab.terrains.terrain_generator",
        "robot_model": args.robot_model,
        "go2_scene_xml": str(go2_scene_xml),
        "go2_asset_dir": str(go2_asset_dir) if go2_asset_dir is not None else None,
        "terrain": args.terrain,
        "seed": int(args.seed),
        "size": list(cfg.size),
        "num_rows": int(cfg.num_rows),
        "num_cols_requested": int(cfg.num_cols),
        "num_cols_materialized": int(generator.terrain_origins.shape[1]),
        "curriculum": bool(cfg.curriculum),
        "difficulty_range": list(cfg.difficulty_range),
        "border_width": float(cfg.border_width),
        "kept_menagerie_floor": bool(args.keep_menagerie_floor),
        "terrain_origin_shape": list(generator.terrain_origins.shape),
        "terrain_origins": generator.terrain_origins.tolist(),
        "spawn_row": int(args.spawn_row),
        "spawn_col": int(args.spawn_col),
        "selected_unshifted_spawn_origin": selected_origin.tolist(),
        "selected_spawn_origin_after_shift": [0.0, 0.0, 0.0],
        "mujoco_model": {
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nbody": int(model.nbody),
            "ngeom": int(model.ngeom),
            "nhfield": int(model.nhfield),
            "timestep": float(model.opt.timestep),
            "integrator": int(model.opt.integrator),
            "solver": int(model.opt.solver),
            "cone": int(model.opt.cone),
            "impratio": float(model.opt.impratio),
        },
    }
    json_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"mjb: {mjb_path}")
    if xml_saved:
        print(f"xml: {xml_path}")
    else:
        print(f"xml: not emitted ({xml_error})")
    print(f"json: {json_path}")
    if args.robot_model == "unitree_mjlab" and xml_saved:
        print(
            "run_unitree_runtime: "
            "bash scripts/deploy/run_unitree_mjlab_sim_deploy.sh "
            f"sim {xml_path}"
        )
    else:
        print(
            "run_sim2sim: "
            f"python scripts/deploy/run_sim2sim.py --model-path {mjb_path} "
            "--execute-runtime --viewer"
        )


if __name__ == "__main__":
    main()
