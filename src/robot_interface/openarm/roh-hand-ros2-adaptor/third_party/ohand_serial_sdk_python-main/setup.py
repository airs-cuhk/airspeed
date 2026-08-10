from setuptools import setup, find_packages

VERSION = "25.12.16"

INSTALL_REQUIRES = [
    "pyserial==3.5",
    "python-can==4.5.0"
]

setup(
    name="ohand_serial_sdkpy",
    version=VERSION,
    author="oymotion",
    author_email="info@oymotion.com",
    long_description=open("README.md", "r", encoding="utf-8").read(),
    long_description_content_type="text/markdown",
    license="BSD-3-Clause",
    description="OHand Serial SDK for Python",
    url="https://github.com/openvmi/ohand_serial_sdk_python",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    install_requires=INSTALL_REQUIRES
)


