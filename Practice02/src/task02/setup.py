from setuptools import find_packages, setup

package_name = 'task02'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'task02']),
        ('share/' + 'task02', ['package.xml']),
        ('share/' + 'task02' + '/launch', ['launch/task02.launch']),  # Include the launch file
        ('share/' + 'task02' + '/config', ['config/task02.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = task02.publisher:main',
        ],
    },
)
