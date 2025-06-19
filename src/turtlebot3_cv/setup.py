from setuptools import setup

package_name = 'turtlebot3_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owl',
    maintainer_email='zesmaeili85@gmail.com',
    description='TurtleBot3 computer vision package for detecting colored objects',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_detector = turtlebot3_cv.color_detector:main',
            'image_subscriber = turtlebot3_cv.image_subscriber:main',
        ],
    },
)

