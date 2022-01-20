import setuptools

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

description = "Contains classes and functions to handle tracking of Aruco markers using OpenCV."

setuptools.setup(
    name="ArucoMarkerTracking",
    version="0.1.2",
    author="Nathan Rumsey",
    author_email="rumseyn@oregonstate.edu",
    description=description,
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/OSURoboticsClub/Aerial_2021_ArucoMarkerTracking",
    project_urls= {
        "Issues": "https://github.com/OSURoboticsClub/Aerial_2021_ArucoMarkerTracking/issues" 
    },
    packages=['ArucoMarkerTracking'],
    install_requires=['opencv-contrib-python', 'numpy'],
)
