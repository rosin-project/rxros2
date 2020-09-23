from setuptools import setup

package_name = 'rxros2_py'

setup(
    name=package_name,
    version='0.1.0',
    # the Python package name is different from the ROS package name. This is
    # on purpose, to avoid the '_py' suffix on the Python package (which isn't
    # needed). We do need the suffix on the ROS package, as there is also a C++
    # version of this package and we need to avoid nameclashes.
    packages=['rxros2'],
    url='https://github.com/rosin-project/rxros2',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Henrik Larsen',
    author_email='henrik7264@outlook.com',
    maintainer='Henrik Larsen <henrik7264@outlook.com>, '
        'Andrzej Wasowski <wasowski@itu.dk>, '
        'G.A. vd. Hoorn <g.a.vanderhoorn@tudelft.nl>',
    keywords=['ROS', 'RxPy', 'reactive'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Reactive programming in Python for ROS 2.',
    license='Apache License, Version 2.0',
)
