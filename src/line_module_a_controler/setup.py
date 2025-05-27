from setuptools import setup

package_name = 'line_module_a_controler'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/line_module_a_controler']),
        ('share/line_module_a_controler', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccj',
    maintainer_email='your_email@example.com',
    description='A模组位置控制和回零服务',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_module_a_controler = line_module_a_controler.line_module_a_controler:main',
            'line_module_a_home_service = line_module_a_controler.line_module_a_home_service:main',
        ],
    },
)
