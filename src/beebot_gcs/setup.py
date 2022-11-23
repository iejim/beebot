from setuptools import setup
from glob import glob

package_name = 'beebot_gcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name+".recursos"],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivan.jimenez@intec.edu.do',
    description='Paquete de nodos para correr en la computadora de control (GCS)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcs_gamepad = beebot_gcs.gcs_gamepad:main'
        ],
    },
)
