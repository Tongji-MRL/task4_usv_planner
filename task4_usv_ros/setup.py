from setuptools import setup


package_name = "task4_usv_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, "task4_usv_planner"],
    package_dir={
        package_name: package_name,
        "task4_usv_planner": "../task4_usv_planner",
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Task 4 USV Planner Maintainers",
    maintainer_email="maintainers@example.com",
    description="ROS 2 adapter for the pure Python Task 4 USV planner.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "task4_planner_node = task4_usv_ros.task4_planner_node:main",
            "task4_supervisor_node = task4_usv_ros.task4_supervisor_node:main",
            "task4_safety_monitor_node = task4_usv_ros.task4_safety_monitor_node:main",
        ],
    },
)
