from setuptools import find_packages, setup

package_name = 'bt_thesis_agents'

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
    maintainer='shubh',
    maintainer_email='shubh@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception_agent_v2 = bt_thesis_agents.perception_agent_v2:main',
            'perception_agent = bt_thesis_agents.perception_agent:main',
            'skill_agent = bt_thesis_agents.skill_agent:main',
        ],
    },
)
