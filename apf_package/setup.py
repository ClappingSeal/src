from setuptools import setup

package_name = 'apf_package'

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
    maintainer='ddsdol',
    maintainer_email='ddsdol@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_node = apf_package.apf_node:main',
            'apf_client = apf_package.apf_client:main',
            'apf_ppo_node = apf_package.apf_ppo_node:main',
            'apf_ppo_client = apf_package.apf_ppo_client:main',

        ],
    },
)
