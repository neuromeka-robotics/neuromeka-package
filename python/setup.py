from setuptools import setup, find_packages
import sys

with open("../README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

if sys.version_info < (3, 12):
    print("Please build with python 3.12 or higher.")
else:
    step_requires = [
        "grpcio>=1.34.1, <=1.39.0",
        "grpcio-tools>=1.34.1, <=1.39.0",
        "protobuf==3.17.3",
        "requests==2.22.0",
        "Pillow>=8.4.0, <=9.5.0",
        "numpy>=1.19.4, <=1.21.6",
        "pyModbusTCP==0.2.1",
        "netifaces==0.11.0",
        "pyyaml==6.0.1",
    ]
    common_requires = [
        "grpcio==1.80.0",
        "grpcio-tools==1.80.0",
        "protobuf>=6.31.1, <7.0.0",
        "requests>=2.31.0, <=2.32.0",
        "urllib3>=2.0.7, <=2.2.2",
        "Pillow==11.3.0",
        # NumPy's Python 3.14 wheels require a newer series than Python 3.9 supports.
        "numpy==2.0.2; python_version < '3.13'",
        "numpy==2.3.5; python_version >= '3.13'",
        "pyModbusTCP==0.2.1",
        "netifaces2",
        "pyyaml==6.0.3",
    ]
    
    setup(
        name="neuromeka",
        version="3.5.0.11",
        license="MIT",
        author="Neuromeka",
        author_email="technical-support@neuromeka.com",
        description="Neuromeka client protocols for IndyDCP3, IndyEye, Moby, Ecat, and Motor",
        long_description=long_description,
        long_description_content_type="text/markdown",
        url="https://github.com/neuromeka-robotics/neuromeka-package",
        packages=find_packages(),
        classifiers=[
            "Development Status :: 3 - Alpha",
            "Intended Audience :: Developers",
            # "License :: OSI Approved :: MIT License",
            "Operating System :: OS Independent",
            "Programming Language :: Python :: 3.7",
            "Programming Language :: Python :: 3.8",
            "Programming Language :: Python :: 3.9",
            "Programming Language :: Python :: 3.10",
            "Programming Language :: Python :: 3.11",
            "Programming Language :: Python :: 3.12",
            "Programming Language :: Python :: 3.13",
            "Programming Language :: Python :: 3.14"
        ],
        python_requires=">=3.7",
        # install_requires=install_requires,
        extras_require={
        ':python_version < "3.9"': step_requires,
        ':python_version >= "3.9"': common_requires,
        },
    )
