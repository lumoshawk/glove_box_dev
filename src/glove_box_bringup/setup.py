from setuptools import setup

package_name = 'glove_box_bringup'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/config', ['config/bringup.yaml']),
        (share + '/launch', ['launch/all.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'read_all = glove_box_bringup.read_all:main',
            'enable_test = glove_box_bringup.enable_test:main',
        ],
    },
)
