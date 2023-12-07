from setuptools import setup

package_name = 'ppo_package'

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
            'ppo_node1 = ppo_package.ppo_node1:main',
            'ppo_lstm_node1 = ppo_package.ppo_lstm_node1:main',
            'ppo_client1 = ppo_package.ppo_client1:main',
            'ppo_node2 = ppo_package.ppo_node2:main',
            'ppo_lstm_node2 = ppo_package.ppo_lstm_node2:main',
            'ppo_client2 = ppo_package.ppo_client2:main',
        ],
    },
)
