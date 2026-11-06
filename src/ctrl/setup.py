from setuptools import find_packages, setup

package_name = 'ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ctrl.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lin',
    maintainer_email='2152685808@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drop = ctrl.drop:main',
            'drop_over = ctrl.drop_over:main',
            'grab = ctrl.grab:main',
            'real_robot_bridge = ctrl.real_robot_bridge:main',
            'rviz_joint_bridge = ctrl.rviz_joint_bridge:main',
            'state_comparator = ctrl.state_comparator:main',
            (
                'raise_body_lower_arms_test = '
                'ctrl.raise_body_lower_arms_test:main'
            ),
        ],
    },
)
