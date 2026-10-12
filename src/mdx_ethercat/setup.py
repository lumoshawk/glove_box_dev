from glob import glob

from setuptools import setup

package_name = 'mdx_ethercat'
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
        (share + '/launch', ['launch/mdx.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'read_test = mdx_ethercat.read_test:main',
            'pp_test = mdx_ethercat.pp_test:main',
            'csp_test = mdx_ethercat.csp_test:main',
            'ec_tool = mdx_ethercat.util.ec_tool:main',
            'plot_csv = mdx_ethercat.util.plot_csv:main',
        ],
    },
)
