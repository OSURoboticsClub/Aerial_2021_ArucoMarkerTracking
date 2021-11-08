import setuptools

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

description = ""

setuptools.setup(
    name="ArucoMarkerTracking",
    version="0.0.1",
    author="Nathan Rumsey",
    author_email="rumseyn@oregonstate.edu",
    description=description,
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/OSURoboticsClub/", #! Real repo name
    project_urls= {
        "Issues": "https://github.com/OSURoboticsClub/[]/issues" #! real repo name
    },
    packages=['ArucoMarkerTracking'],
    install_requires=['opencv-contrib-python', 'numpy'],
)