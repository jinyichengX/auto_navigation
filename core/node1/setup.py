from setuptools import find_packages, setup

package_name = 'node1'

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
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'process1 = node1.node_test:main',
            'process_learn_thread = node1.learn_thread:main',
            'process_download_novel = node1.download_novel:main'
        ],
    },
)
