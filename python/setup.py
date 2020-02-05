import platform
import sys
import pathlib

import setuptools

if __name__ == "__main__":
    setuptools.setup(
        name="cepton_sdk",
        version=open("cepton_util/VERSION").read().strip(),
        description="Cepton Python SDK",
        long_description=open("README.md").read(),
        long_description_content_type="text/markdown",
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
            "laspy",
            "plyfile",
            "transforms3d"
        ],
        extras_require={
            "samples": [
                "imageio-ffmpeg",
                "imageio",
                "netifaces",
                "plyfile",
                "pyqt5",
                "uuid",
                "vispy",
            ],
        },
        scripts=[
            "tools/cepton_capture_gui.py",
            "tools/cepton_capture.py",
            "tools/cepton_clip.py",
            "tools/cepton_export_serial.py",
            "tools/cepton_export.py",
            "tools/cepton_georeference.py",
            "tools/cepton_list_sensors.py",
        ]
    )
