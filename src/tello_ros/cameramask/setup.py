from setuptools import setup

package_name = 'cameramask'

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
    maintainer='elias',
    maintainer_email='elias@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = cameramask.cameramask:main',
        	'saveimg = cameramask.imagesaver:main',
        	'publishImg = cameramask.imagepublish:main',
            'moveTello = cameramask.gatefollower:main',
            'fly_to_gate = cameramask.fly_to_gate:main',
            'fly_through_gate = cameramask.fly_through_gate:main'
        ],
    },
)
