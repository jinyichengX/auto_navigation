from setuptools import find_packages, setup

package_name = 'node2'

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
    maintainer='jinyicheng',
    maintainer_email='jinyicheng@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'process_pub_novel = node2.novel_pub_node:main',
            'process_sub_novel = node2.novel_sub_node:main'
        ],
    },
)
