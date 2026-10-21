from glob import glob

from setuptools import find_packages, setup

package_name = 'gcan_ethercat'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml', 'README.md']),
        (share + '/config', glob('config/*.yaml')),
        (share + '/config/drives', glob('config/drives/*.yaml')),
        (share + '/config/gateways', glob('config/gateways/*.yaml')),
        (share + '/config/profiles', glob('config/profiles/*.yaml')),
        (share + '/launch', ['launch/gcan.launch.py']),
        (share + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_test = gcan_ethercat.read_test:main',
            'enable_test = gcan_ethercat.enable_test:main',
            'can_tool = gcan_ethercat.can_tool:main',
        ],
    },
)
