from setuptools import setup

package_name = 'anas_ai'

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
    maintainer='yuuki',
    maintainer_email='huy2840@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav = anas_ai.anasai:main',
            'occ = anas_ai.occupancy:main',
            'pos = anas_ai.position:main',
            'follow = anas_ai.wallfollower:main',
            'state = anas_ai.state:main',       
        ],
    },
)
