from setuptools import setup

package_name = 'beebot_control'

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
    maintainer='ivan',
    maintainer_email='ivan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = beebot_control.simple_publisher:main',
            'nodo_beebot = beebot_control.nodo_beebot:main',
            'interfaz_pca = beebot_control.interfaz_pca:main',

        ],
    },
)
