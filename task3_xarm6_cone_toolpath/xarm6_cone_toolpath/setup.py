from setuptools import find_packages, setup

package_name = 'xarm6_cone_toolpath'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/cone_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anand',
    maintainer_email='anandgawai123456@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    scripts=[
        'scripts/generate_cone_path.py',
        'scripts/solve_cone_ik.py',
        'scripts/generate_trajectory.py',
        'scripts/execute_cone_trajectory.py',
        'scripts/probe_workspace.py',
        'scripts/test_cone_pose.py',
        'scripts/test_moveit_ik.py',
        'scripts/diagnose_cone_ik.py',
    ],
)
