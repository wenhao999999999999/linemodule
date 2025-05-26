from setuptools import setup

package_name = 'line_module_b_controler'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[('share/' + package_name, ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ccj',
    maintainer_email='your_email@example.com',
    description='B模组控制服务',
    license='MIT',
    entry_points={
        'console_scripts': [
            'line_module_b_home_service = line_module_b_controler.line_module_b_home_service:main',
            'line_module_b_controler = line_module_b_controler.line_module_b_controler:main'
        ],
    },
)
