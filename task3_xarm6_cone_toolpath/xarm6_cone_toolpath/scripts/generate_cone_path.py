#!/usr/bin/env python3

import csv
import math


# ============================================================
# xArm6 reachable cone placement
# ============================================================

CENTER_X = 0.300
CENTER_Y = 0.000

Z_START = 0.160
HEIGHT = 0.200

BASE_RADIUS = 0.060
TIP_RADIUS = 0.020

N_TURNS = 4.0
POINTS_PER_TURN = 80


def cone_radius(z):

    t = (z - Z_START) / HEIGHT

    return (
        BASE_RADIUS
        + t * (TIP_RADIUS - BASE_RADIUS)
    )


def generate_cone_path():

    total_points = int(
        N_TURNS * POINTS_PER_TURN
    )

    path = []

    dr_dz = (
        TIP_RADIUS - BASE_RADIUS
    ) / HEIGHT

    for i in range(total_points):

        u = i / (total_points - 1)

        z = Z_START + u * HEIGHT

        theta = (
            2.0
            * math.pi
            * N_TURNS
            * u
        )

        r = cone_radius(z)

        # ----------------------------------------------------
        # Position
        # ----------------------------------------------------

        x = (
            CENTER_X
            + r * math.cos(theta)
        )

        y = (
            CENTER_Y
            + r * math.sin(theta)
        )

        # ----------------------------------------------------
        # Surface normal
        # ----------------------------------------------------

        nx = math.cos(theta)
        ny = math.sin(theta)
        nz = -dr_dz

        norm = math.sqrt(
            nx * nx
            + ny * ny
            + nz * nz
        )

        nx /= norm
        ny /= norm
        nz /= norm

        path.append({
            'index': i,
            'x': x,
            'y': y,
            'z': z,
            'nx': nx,
            'ny': ny,
            'nz': nz,
            'theta': theta
        })

    return path


def save_path(path, filename):

    with open(
        filename,
        'w',
        newline=''
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            'index',
            'x',
            'y',
            'z',
            'nx',
            'ny',
            'nz',
            'theta'
        ])

        for p in path:

            writer.writerow([
                p['index'],
                f"{p['x']:.6f}",
                f"{p['y']:.6f}",
                f"{p['z']:.6f}",
                f"{p['nx']:.6f}",
                f"{p['ny']:.6f}",
                f"{p['nz']:.6f}",
                f"{p['theta']:.6f}"
            ])


def main():

    path = generate_cone_path()

    filename = 'cone_cartesian_path.csv'

    save_path(
        path,
        filename
    )

    xs = [p['x'] for p in path]
    ys = [p['y'] for p in path]
    zs = [p['z'] for p in path]

    print()
    print('===== CONE PATH GENERATED =====')
    print(f'Points       : {len(path)}')
    print(f'Center       : ({CENTER_X:.3f}, {CENTER_Y:.3f}) m')
    print(f'Height       : {HEIGHT:.3f} m')
    print(f'Base radius  : {BASE_RADIUS:.3f} m')
    print(f'Tip radius   : {TIP_RADIUS:.3f} m')
    print(f'Turns        : {N_TURNS:.1f}')
    print()
    print(
        f'X range      : '
        f'{min(xs):.3f} → {max(xs):.3f} m'
    )
    print(
        f'Y range      : '
        f'{min(ys):.3f} → {max(ys):.3f} m'
    )
    print(
        f'Z range      : '
        f'{min(zs):.3f} → {max(zs):.3f} m'
    )
    print()
    print(f'Output       : {filename}')
    print()

    print('First point:')
    print(path[0])

    print()
    print('Last point:')
    print(path[-1])


if __name__ == '__main__':
    main()
