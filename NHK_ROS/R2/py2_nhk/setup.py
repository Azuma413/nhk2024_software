from setuptools import find_packages, setup
from glob import glob

package_name = 'py2_nhk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        ('share/' + package_name, glob('weights/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humble',
    maintainer_email='hirekatsu0523@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_ball_node = py2_nhk.detect_ball_node:main',
            'trace_ball_node = py2_nhk.trace_ball_node:main',
        ],
    },
)
