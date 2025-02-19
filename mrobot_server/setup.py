from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mrobot_server'
submodules = 'mrobot_server/dependent_library'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml', 'mrobot_server/AGVServer_Setting.json']),
        ('share/' + package_name, ['package.xml', 'mrobot_server/billingModel.json']),
        (os.path.join('share', package_name, 'mp3s'), glob(os.path.join('mrobot_server', 'mp3s', '*.wav'))),
        (os.path.join('share', package_name, 'lib'), glob(os.path.join('mrobot_server', 'lib', '*.so'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lsd',
    maintainer_email='14718152756@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mrobot_server = mrobot_server.mrobot_server:main',
        ],
    },
)
