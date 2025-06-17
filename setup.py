from setuptools import find_packages, setup

package_name = 'pharmacy_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='choin',
    maintainer_email='choin22222@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pharmacy_manager = pharmacy_main.pharmacy_manager:main',
            'pharmacy_gui = pharmacy_main.pharmacy_gui:main',
            'voice_input = pharmacy_main.voice_input:main',
            'symptom_matcher = pharmacy_main.symptom_matcher:main',
            'detector = pharmacy_main.detector:main',
            'robot_arm = pharmacy_main.robot_arm:main',
            'move_test = pharmacy_main.move_test:main',
        ],
    },
)
