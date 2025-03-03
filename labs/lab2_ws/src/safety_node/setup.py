from setuptools import setup

package_name = 'safety_node'

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
    description='Automatic Emergency Braking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety_node.safety_node:main',
        ],
    },
)