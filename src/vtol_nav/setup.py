from setuptools import find_packages, setup

package_name = 'vtol_nav'

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
    maintainer='dmitriyb51',
    maintainer_email='d.v.belinskiy51@mail.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_vtol = vtol_nav.hello_vtol:main',
            'leader_follower = vtol_nav.leader_follower:main',
            'swarm_navigate_server = vtol_nav.swarm_navigate_server:main',
            
        ],
    },
)
