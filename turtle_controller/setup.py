from setuptools import setup

package_name = 'turtle_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IotaKataoka',
    maintainer_email='c1119576@planet.kanazawa-it.ac.jp',
    description='TODO: License declaration',
    license='TODO: Package description',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_controller = '+ package_name + '.turtle_controller:ros_main',
        ],
    },
)
