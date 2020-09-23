from setuptools import setup

package_name = 'rxros2_py_examples'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    keywords=['RxPy', 'reactive', 'examples'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Example nodes showing how to use rxros2_py.',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'listener_new_style = rxros2_py_examples.listener_new_style:main',
            'listener_old_style = rxros2_py_examples.listener_old_style:main',
            'talker_new_style = rxros2_py_examples.talker_new_style:main',
            'talker_old_style = rxros2_py_examples.talker_old_style:main',
        ],
    },
)
