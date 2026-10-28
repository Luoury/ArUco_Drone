from setuptools import setup

package_name = 'drone_tools'

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
    description='工程工具：流程自动验收 verify_flow',
    license='MIT',
    entry_points={
        'console_scripts': [
            'verify_flow = drone_tools.verify_flow:main',
        ],
    },
)
