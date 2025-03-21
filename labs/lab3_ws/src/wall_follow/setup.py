from setuptools import setup

package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),  # <-- This line ensures launch files are installed
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JY',
    maintainer_email='darkmoonz1004@gmail.com',
    description='f1tenth wall_follow lab',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follow_node = wall_follow.wall_follow_node:main',
        ],
    },
)