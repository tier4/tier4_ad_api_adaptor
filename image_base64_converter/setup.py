from setuptools import setup

package_name = 'image_base64_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", ["launch/image_base64_converter.launch.xml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yabuta',
    maintainer_email='makoto.yabuta@tier4.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_base64_converter = image_base64_converter.image_base64_converter:main",
        ],
    },
)
