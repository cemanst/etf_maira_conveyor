from setuptools import setup

package_name = 'conveyor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Stanislav Ceman',
    author_email='cemanst@gmail.com',
    maintainer='Stanislav Ceman',
    maintainer_email='cemanst@gmail.com',
    url='https://github.com/cemanst/etf_maira_conveyor',
    description='Conveyor driver for ATV320 via OPC UA',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
