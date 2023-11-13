from setuptools import setup

package_name = 'td3_package'

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
    maintainer='qwer',
    maintainer_email='qwer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'td3_node1 = td3_package.td3_node1:main',
            'td3_client1 = td3_package.td3_client1:main',
            'td3_node2 = td3_package.td3_node2:main',
            'td3_client2 = td3_package.td3_client2:main',
        ],
    },
)
