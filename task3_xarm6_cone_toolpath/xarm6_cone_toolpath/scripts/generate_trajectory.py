#!/usr/bin/env python3

import csv
import math
import sys


INPUT_FILE = 'cone_ik_test.csv'
OUTPUT_FILE = 'cone_toolpath.csv'

JOINTS = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6'
]

# Conservative trajectory limits for the first simulation pass.
# These are intentionally below typical xArm6 capability so that
# the generated trajectory is slow and easy to validate in Gazebo.
MAX_VELOCITY = 0.50       # rad/s
MAX_ACCELERATION = 1.00   # rad/s^2

MIN_DT = 0.05


def read_ik_points():

    with open(INPUT_FILE, newline='') as f:

        reader = csv.DictReader(f)

        rows = list(reader)

    if not rows:
        raise RuntimeError(
            f'No trajectory points found in {INPUT_FILE}'
        )

    required = [
        'joint1',
        'joint2',
        'joint3',
        'joint4',
        'joint5',
        'joint6'
    ]

    for name in required:

        if name not in rows[0]:
            raise RuntimeError(
                f'Missing column: {name}'
            )

    points = []

    for row in rows:

        joints = [
            float(row[name])
            for name in JOINTS
        ]

        points.append(joints)

    return points


def calculate_segment_time(q0, q1):

    max_delta = max(
        abs(b - a)
        for a, b in zip(q0, q1)
    )

    if max_delta < 1e-12:
        return MIN_DT

    # Time required so that no joint exceeds the
    # configured velocity limit.
    dt_velocity = max_delta / MAX_VELOCITY

    # Additional conservative acceleration constraint.
    dt_acceleration = math.sqrt(
        2.0 * max_delta / MAX_ACCELERATION
    )

    dt = max(
        MIN_DT,
        dt_velocity,
        dt_acceleration
    )

    return dt


def calculate_velocities(points, times):

    velocities = []

    for i in range(len(points)):

        if i == 0:

            dt = times[1] - times[0]

            velocity = [
                (points[1][j] - points[0][j]) / dt
                for j in range(6)
            ]

        elif i == len(points) - 1:

            dt = times[-1] - times[-2]

            velocity = [
                (points[-1][j] - points[-2][j]) / dt
                for j in range(6)
            ]

        else:

            dt = times[i + 1] - times[i - 1]

            velocity = [
                (points[i + 1][j] - points[i - 1][j]) / dt
                for j in range(6)
            ]

        velocities.append(velocity)

    return velocities


def calculate_accelerations(velocities, times):

    accelerations = []

    for i in range(len(velocities)):

        if i == 0:

            dt = times[1] - times[0]

            acceleration = [
                (velocities[1][j] - velocities[0][j]) / dt
                for j in range(6)
            ]

        elif i == len(velocities) - 1:

            dt = times[-1] - times[-2]

            acceleration = [
                (velocities[-1][j] - velocities[-2][j]) / dt
                for j in range(6)
            ]

        else:

            dt = times[i + 1] - times[i - 1]

            acceleration = [
                (velocities[i + 1][j] - velocities[i - 1][j]) / dt
                for j in range(6)
            ]

        accelerations.append(acceleration)

    return accelerations


def validate(points, velocities, accelerations):

    max_velocity = 0.0
    max_acceleration = 0.0

    velocity_violation = False
    acceleration_violation = False

    for velocity in velocities:

        for value in velocity:

            magnitude = abs(value)

            max_velocity = max(
                max_velocity,
                magnitude
            )

            if magnitude > MAX_VELOCITY + 1e-6:

                velocity_violation = True

    for acceleration in accelerations:

        for value in acceleration:

            magnitude = abs(value)

            max_acceleration = max(
                max_acceleration,
                magnitude
            )

            if magnitude > MAX_ACCELERATION + 1e-6:

                acceleration_violation = True

    return (
        max_velocity,
        max_acceleration,
        velocity_violation,
        acceleration_violation
    )


def write_csv(points, times, velocities, accelerations):

    with open(
        OUTPUT_FILE,
        'w',
        newline=''
    ) as f:

        writer = csv.writer(f)

        writer.writerow([
            'index',
            'time_from_start',
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'joint5',
            'joint6',
            'velocity1',
            'velocity2',
            'velocity3',
            'velocity4',
            'velocity5',
            'velocity6',
            'acceleration1',
            'acceleration2',
            'acceleration3',
            'acceleration4',
            'acceleration5',
            'acceleration6'
        ])

        for i in range(len(points)):

            writer.writerow([
                i,
                f'{times[i]:.6f}',

                *[
                    f'{value:.8f}'
                    for value in points[i]
                ],

                *[
                    f'{value:.8f}'
                    for value in velocities[i]
                ],

                *[
                    f'{value:.8f}'
                    for value in accelerations[i]
                ]
            ])


def main():

    print()
    print('===== xARM6 TRAJECTORY GENERATOR =====')
    print()

    try:

        points = read_ik_points()

    except Exception as exc:

        print(f'ERROR: {exc}')
        sys.exit(1)

    print(f'Input points       : {len(points)}')
    print(f'Max velocity       : {MAX_VELOCITY:.3f} rad/s')
    print(f'Max acceleration   : {MAX_ACCELERATION:.3f} rad/s^2')

    # --------------------------------------------------------
    # Time allocation
    # --------------------------------------------------------

    times = [0.0]

    for i in range(1, len(points)):

        dt = calculate_segment_time(
            points[i - 1],
            points[i]
        )

        times.append(
            times[-1] + dt
        )

    # --------------------------------------------------------
    # Velocity / acceleration
    # --------------------------------------------------------

    velocities = calculate_velocities(
        points,
        times
    )

    accelerations = calculate_accelerations(
        velocities,
        times
    )

    # --------------------------------------------------------
    # Validation
    # --------------------------------------------------------

    (
        max_velocity,
        max_acceleration,
        velocity_violation,
        acceleration_violation
    ) = validate(
        points,
        velocities,
        accelerations
    )

    write_csv(
        points,
        times,
        velocities,
        accelerations
    )

    duration = times[-1]

    print()
    print('===== TRAJECTORY RESULT =====')
    print(f'Points             : {len(points)}')
    print(f'Duration           : {duration:.3f} s')
    print(f'Max velocity       : {max_velocity:.6f} rad/s')
    print(f'Max acceleration   : {max_acceleration:.6f} rad/s^2')
    print(f'Output             : {OUTPUT_FILE}')
    print()

    print('===== VALIDATION =====')

    if velocity_violation:

        print('Velocity limit     : FAIL')

    else:

        print('Velocity limit     : PASS')

    if acceleration_violation:

        print('Acceleration limit : FAIL')

    else:

        print('Acceleration limit : PASS')

    # Check timestamps.

    timestamps_ok = all(
        times[i] > times[i - 1]
        for i in range(1, len(times))
    )

    if timestamps_ok:

        print('Timestamps         : PASS')

    else:

        print('Timestamps         : FAIL')

    # Check finite values.

    finite = True

    for point in points:

        for value in point:

            if not math.isfinite(value):

                finite = False

    if finite:

        print('Joint values       : PASS')

    else:

        print('Joint values       : FAIL')

    print()

    if (
        not velocity_violation
        and not acceleration_violation
        and timestamps_ok
        and finite
    ):

        print(
            '===== TRAJECTORY VALIDATION PASSED ====='
        )

        return 0

    print(
        '===== TRAJECTORY VALIDATION FAILED ====='
    )

    return 1


if __name__ == '__main__':
    sys.exit(main())
