from setuptools import setup

package_name = 'conveyor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={package_name: []},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/conveyor.launch.py']),
        ('share/' + package_name + '/config', ['config/conveyor.yaml']),
    ],
    install_requires=['setuptools', 'asyncua'],
    zip_safe=True,
    author='Stanislav Ceman',
    author_email='cemanst@gmail.com',
    maintainer='Stanislav Ceman',
    maintainer_email='cemanst@gmail.com',
    url='https://github.com/cemanst/etf_maira_conveyor',
    description='Conveyor driver for ATV320 via OPC UA and mock mode',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'conveyor_node = conveyor_driver.conveyor_node:main',
            'mock_conveyor_driver = conveyor_driver.mock_conveyor_driver:main',
        ],
    },
)
