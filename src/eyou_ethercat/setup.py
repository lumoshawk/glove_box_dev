from glob import glob

from setuptools import setup

package_name = 'eyou_ethercat'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.util'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/config', glob('config/*.yaml')),
        (share + '/config/drives', glob('config/drives/*.yaml')),
        (share + '/config/profiles', glob('config/profiles/*.yaml')),
        (share + '/launch', ['launch/eyou.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'read_test = eyou_ethercat.read_test:main',
            'read_all = eyou_ethercat.read_all:main',
            'pp_test = eyou_ethercat.pp_test:main',
            'pv_test = eyou_ethercat.pv_test:main',
            'csp_test = eyou_ethercat.csp_test:main',
            'ec_tool = eyou_ethercat.util.ec_tool:main',
            'plot_csv = eyou_ethercat.util.plot_csv:main',
        ],
    },
)
