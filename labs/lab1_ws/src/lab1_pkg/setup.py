from setuptools import setup

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JY',
    maintainer_email='darkmoonz1004@gmail.com',
    description='A package for F1TENTH Lab1',
    license='Apache License 2.0',
    test_requires=['pytest'],
    entry_points={
        'console_scripts': [
            'relay = lab1_pkg.relay:main',
        ],
    },
)
