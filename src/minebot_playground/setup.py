from setuptools import find_packages, setup

package_name = 'minebot_playground'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phc2603',
    maintainer_email='pedro.caillaux@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points = {
        'console_scripts': [
            'control_auto = minebot_playground.control_auto:main',
            'gas_mock = minebot_playground.gas_mock:main',
            'user_controller = minebot_playground.user_controller:main',
            'usbcam_node = minebot_playground.usbcam_node:main',
        ],
    }
)
