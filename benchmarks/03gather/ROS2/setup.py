from setuptools import find_packages, setup

package_name = 'marl'

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
    maintainer='jackyk',
    maintainer_email='jackykwok@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env = marl.env:main',
            'policy1 = marl.policy1:main',
            'policy2 = marl.policy2:main',
            'policy3 = marl.policy3:main',
            'policy4 = marl.policy4:main',
        ],
    },
)
