from setuptools import setup

package_name = 'drone_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luosury',
    maintainer_email='luosury@example.com',
    description='键盘遥控节点：按键 → /teleop/cmd 与 /mission/mode',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_node = drone_teleop.teleop_node:main',
        ],
    },
)
