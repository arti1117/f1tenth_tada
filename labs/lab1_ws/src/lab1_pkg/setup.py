from setuptools import setup

package_name = 'lab1_pkg'

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
    maintainer='JY',
    maintainer_email='darkmoonz1004@gmail.com',
    description='A package with both C++ and Python nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relay = lab1_pkg.relay:main',
        ],
    },
)
