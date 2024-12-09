from setuptools import setup, find_packages

setup(
    name='agrotechsimapi',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        "msgpack-python==0.5.6",
        "msgpack-rpc-python==0.4.1",
        "numpy==2.2.0",
        "opencv-python==4.10.0.84",
        "tornado==4.5.3"
    ],
    description='description',
    author='agrotechavia',
    author_email='some@example.com',
    url='.',
)