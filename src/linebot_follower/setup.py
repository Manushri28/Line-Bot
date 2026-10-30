from setuptools import find_packages, setup

package_name = 'linebot_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='you@example.com',
    description='OpenCV + PID line follower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = linebot_follower.line_follower:main',
        ],
    },
)
