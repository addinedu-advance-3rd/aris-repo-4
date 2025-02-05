import os
from setuptools import find_packages, setup

package_name = 'aris_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['aris_pkg', 'aris_pkg.*']),  # 모든 서브 패키지를 포함
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),  # 경로 체크 후 추가
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'media_pipe_finger = aris_pkg.motion_following.media_pipe_finger:main',
            'finger_control = aris_pkg.motion_following.finger_control:main',
            'warningRos = aris_pkg.prohibit.warningRos:main',
            'main_process = aris_pkg.main.main_process:main',
            'app_order_publisher = aris_pkg.app.app_order_publisher:main',
            'action_moveit = aris_pkg.moveit.action_moveit:main'
        ],
    },
)

