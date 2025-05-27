from setuptools import setup
from glob import glob

package_name = 'line_module_b_controler'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/resource', ['line_module_b_controler/resource/line_module_b_controler']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # ✅ 添加这一行
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccj',
    maintainer_email='you@example.com',
    description='Line module B controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_module_b_controler = line_module_b_controler.line_module_b_controler:main',
            'line_module_b_home_service = line_module_b_controler.line_module_b_home_service:main',
        ],
    },
)
