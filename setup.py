from setuptools import setup, find_packages

setup(
    name='agrotechsimapi',
    version='0.2',
    packages=find_packages(),
    install_requires=[
        "contourpy==1.3.1",
        "cycler==0.12.1",
        "fonttools==4.55.3",
        "kiwisolver==1.4.7",
        "matplotlib==3.9.3",
        "msgpack-python==0.5.6",
        "msgpack-rpc-python==0.4.1",
        "numpy==1.26.4",
        "opencv-python==4.10.0.84",
        "packaging==24.2",
        "pillow==11.0.0",
        "pyparsing==3.2.0",
        "python-dateutil==2.9.0.post0",
        "six==1.17.0",
        "tornado==4.5.3"
    ],
    description='description',
    author='agrotechavia',
    author_email='some@example.com',
    url='.',
)