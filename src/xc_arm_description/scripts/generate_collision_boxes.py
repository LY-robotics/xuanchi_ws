#!/usr/bin/env python3
"""Replace STL collision meshes in XC_urdf.urdf with AABB boxes.

The box origin is the center of the STL axis-aligned bounding box in the STL
modeling frame; its size is the bounding-box extent. The script edits the URDF
in place so it is the only generated artifact besides the script itself.
"""
from __future__ import annotations

import argparse
import re
import struct
from pathlib import Path


def read_stl_vertices(path: Path) -> tuple[list[tuple[float, float, float]], str, int]:
    data = path.read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            vertices: list[tuple[float, float, float]] = []
            for index in range(triangle_count):
                values = struct.unpack_from("<9f", data, 84 + index * 50 + 12)
                vertices.extend((values[0:3], values[3:6], values[6:9]))
            return vertices, "binary", triangle_count

    text = data.decode("utf-8", errors="ignore")
    matches = re.findall(
        r"vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", text,
    )
    if not matches or len(matches) % 3:
        raise ValueError(f"{path} is not a valid binary or ASCII STL")
    vertices = [tuple(map(float, match)) for match in matches]
    return vertices, "ascii", len(vertices) // 3


def bounding_box(path: Path) -> tuple[list[float], list[float], int, str]:
    vertices, file_format, triangles = read_stl_vertices(path)
    minimum = [min(point[axis] for point in vertices) for axis in range(3)]
    maximum = [max(point[axis] for point in vertices) for axis in range(3)]
    center = [(lo + hi) / 2.0 for lo, hi in zip(minimum, maximum)]
    size = [hi - lo for lo, hi in zip(minimum, maximum)]
    return center, size, triangles, file_format


def vector(values: list[float]) -> str:
    return " ".join(f"{value:.9f}" for value in values)


def collision_block(link_name: str, center: list[float], size: list[float], indent: str) -> str:
    return "\n".join([
        f'{indent}<collision name="{link_name}_aabb">',
        f'{indent}  <origin xyz="{vector(center)}" rpy="0 0 0" />',
        f'{indent}  <geometry>',
        f'{indent}    <box size="{vector(size)}" />',
        f'{indent}  </geometry>',
        f'{indent}</collision>',
    ])


def replace_link_collisions(urdf: str, link_name: str, block: str) -> str:
    link_pattern = re.compile(
        rf'(?P<open>^[ \t]*<link\s+name="{re.escape(link_name)}"\s*>)(?P<body>.*?)(?P<close>^[ \t]*</link>)',
        re.MULTILINE | re.DOTALL,
    )
    match = link_pattern.search(urdf)
    if not match:
        raise ValueError(f'URDF does not contain link "{link_name}"')
    body = match.group("body")
    collision_pattern = re.compile(r"^[ \t]*<collision\b.*?</collision>\s*", re.MULTILINE | re.DOTALL)
    if collision_pattern.search(body):
        body = collision_pattern.sub(block + "\n", body)
    else:
        inertial = re.search(r"^[ \t]*<inertial\b", body, re.MULTILINE)
        if inertial:
            body = body[:inertial.start()] + block + "\n" + body[inertial.start():]
        else:
            body = body + block + "\n"
    return urdf[:match.start("body")] + body + urdf[match.end("body"):]


def add_world(urdf: str, base_height: float) -> str:
    if re.search(r'<link\s+name="world"\s*/?>', urdf):
        return urdf
    marker = re.search(r'^[ \t]*<link\s+name="base_link"\s*>', urdf, re.MULTILINE)
    if not marker:
        raise ValueError('URDF does not contain link "base_link"')
    world = "\n".join([
        '  <link name="world" />',
        '  <joint name="world_to_base" type="fixed">',
        '    <parent link="world" />',
        '    <child link="base_link" />',
        f'    <origin xyz="0 0 {base_height:.6f}" rpy="0 0 0" />',
        '  </joint>',
        '',
    ])
    return urdf[:marker.start()] + world + urdf[marker.start():]


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=package_dir / "urdf" / "XC_urdf.urdf")
    parser.add_argument("--mesh-dir", type=Path, default=package_dir / "meshes")
    parser.add_argument("--base-height", type=float, default=0.8)
    args = parser.parse_args()

    urdf = args.urdf.read_text(encoding="utf-8")
    urdf = add_world(urdf, args.base_height)
    meshes = sorted((path for path in args.mesh_dir.iterdir() if path.suffix.lower() == ".stl"), key=lambda p: p.name.lower())
    if not meshes:
        raise SystemExit(f"No STL files found in {args.mesh_dir}")

    for mesh in meshes:
        link_name = mesh.stem
        center, size, triangles, file_format = bounding_box(mesh)
        indent_match = re.search(
            rf'^(?P<indent>[ \t]*)<link\s+name="{re.escape(link_name)}"\s*>',
            urdf,
            re.MULTILINE,
        )
        if not indent_match:
            raise ValueError(f'URDF has no link matching {mesh.name}')
        block = collision_block(link_name, center, size, indent_match.group("indent") + "  ")
        urdf = replace_link_collisions(urdf, link_name, block)
        print(f"{mesh.name}: center={vector(center)}, size={vector(size)}, triangles={triangles}, format={file_format}")

    args.urdf.write_text(urdf, encoding="utf-8", newline="\n")
    print(f"Updated {args.urdf}")


if __name__ == "__main__":
    main()
