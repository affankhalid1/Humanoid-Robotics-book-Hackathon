from setuptools import setup

package_name = 'service_example'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'service_server',
        'service_client',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Humanoid Robotics Book',
    maintainer_email='info@humanoid-robotics-book.example.com',
    description='Service example for the Physical AI Humanoid Robotics Book',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service_server = service_server:main',
            'service_client = service_client:main',
        ],
    },
)