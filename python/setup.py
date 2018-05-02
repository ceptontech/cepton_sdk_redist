import platform
import sys

from setuptools import setup

if __name__ == "__main__":
    setup(
        name="cepton_sdk",
        version="0.1.2",
        description="Cepton Python SDK",
        long_description=open("README.md").read(),
        url="https://github.com/ceptontech/cepton_sdk_redist",
        author="Cepton Technologies",
        classifiers=[
            "Development Status :: 4 - Beta",
            "Programming Language :: Python :: 3",
        ],
        keywords="cepton sdk",
        python_requires=">=3.3",
        packages=["cepton_sdk"],
        include_package_data=True,
        install_requires=[
            "numpy",
            "uuid",
        ]
    )
