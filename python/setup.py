import platform
import sys

import setuptools

if __name__ == "__main__":
    setuptools.setup(
        name="cepton_sdk",
        version="1.7.1",
        description="Cepton Python SDK",
        long_description=open("README.md").read(),
        url="https://github.com/ceptontech/cepton_sdk_redist",
        author="Cepton Technologies",
        classifiers=[
            "Development Status :: 5 - Production/Stable",
            "Programming Language :: Python :: 3",
        ],
        keywords="cepton sdk",
        python_requires=">=3.3",
        packages=setuptools.find_packages(),
        include_package_data=True,
        install_requires=[
            "numpy",
            "pyserial",
        ],
        extras_require={
            "capture": ["ipaddress", "netifaces"],
            "export": ["laspy", "uuid", "plyfile"],
            "plot": ["pyqt5", "vispy"],
        },
        scripts=[
            "samples/cepton_export",
            "samples/cepton_export_sora",
            "samples/cepton_list_sensors",
        ]
    )
