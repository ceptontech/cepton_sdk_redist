import platform
import sys

import setuptools

if __name__ == "__main__":
    setuptools.setup(
        name="cepton_sdk",
        version="1.14.0",
        description="Cepton Python SDK",
        long_description=open("README.md").read(),
        url="https://github.com/ceptontech/cepton_sdk_redist",
        author="Cepton Technologies",
        author_email="support@cepton.com",
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
            "capture": [
                "netifaces",
            ],
            "export": [
                "laspy",
                "uuid",
                "plyfile",
            ],
            "plot": [
                "pyqt5",
                "vispy",
            ],
        },
        scripts=[
            "samples/advanced/cepton_export_sora.py",
            "samples/cepton_export.py",
            "samples/cepton_list_sensors.py",
        ]
    )
