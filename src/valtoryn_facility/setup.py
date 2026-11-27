from setuptools import find_packages, setup

package_name = "valtoryn_facility"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ram Bhogesara",
    maintainer_email="bhogesara.ram23@gmail.com",
    description="Valtoryn facility model, exporters, and fleet-coordination baseline.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
