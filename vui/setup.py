from setuptools import find_packages, setup
import glob
import os

package_name = 'vui'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=find_packages(include=[
        'vui'
    ]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob.glob('resource/.env')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lhj',
    maintainer_email='hojun7889@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vui = vui.voice_interface:main',
        ],
    },
)
