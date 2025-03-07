from setuptools import setup

package_name = 'mercury_x1_control'

setup(
    name=package_name,
    version='2.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheeltec',
    maintainer_email='wheeltec@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mercury_keyboard = mercury_x1_control.mercury_keyboard:main',
            'slider_control = mercury_x1_control.slider_control:main',
            'slider_control_turing = mercury_x1_control.slider_control_turing:main',
        ],
    },
)
