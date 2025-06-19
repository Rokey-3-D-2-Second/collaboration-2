from setuptools import find_packages, setup

package_name = "ros2_controller"

setup(
    name=package_name,
    version="0.0.0",
    # ────────────────────────────── packages ────────────────────────────── #
    #   ros2_controller, 하위 모듈, 그리고 util 패키지까지 함께 설치
    packages=find_packages(
        include=[
            package_name,            # ros2_controller
            package_name + ".*",     # ros2_controller.*
            "util",                  # util
            "util.*",                # util.*
        ],
        exclude=["test"],
    ),
    # ────────────────────────────── 데이터 파일 ───────────────────────────── #
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    # ────────────────────────────── 의존 패키지 ───────────────────────────── #
    install_requires=[
        "setuptools",
        "pymongo",
        "pandas",
        "matplotlib",
    ],
    zip_safe=True,
    maintainer="lhj",
    maintainer_email="hojun7889@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    # ────────────────────────────── 실행 엔트리포인트 ───────────────────────────── #
    entry_points={
        "console_scripts": [
            "controller = ros2_controller.controller:main",
            "moveiter   = ros2_controller.moveiter:main",
        ],
    },
)
