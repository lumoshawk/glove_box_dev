from glob import glob

from setuptools import setup

package_name = 'dh_ethercat'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml', 'README.md']),
        (share + '/config', glob('config/*.yaml')),
        (share + '/config/drives', glob('config/drives/*.yaml')),
        (share + '/config/gateways', glob('config/gateways/*.yaml')),
        (share + '/config/profiles', glob('config/profiles/*.yaml')),
        (share + '/esi', glob('esi/*.xml')),
        (share + '/launch', ['launch/dh.launch.py']),
        (share + '/docs', glob('docs/*.md')),
        (share + '/resource', ['resource/EC盒子字典.xlsx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_test = dh_ethercat.read_test:main',
            'write_test = dh_ethercat.write_test:main',
            'return_zero_test = dh_ethercat.return_zero_test:main',
            'gen_esi = dh_ethercat.esi:main',
        ],
    },
)
