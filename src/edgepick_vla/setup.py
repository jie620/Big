from setuptools import setup

setup(name="edgepick_vla", version="0.1.0", packages=["edgepick_vla"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/edgepick_vla"]),
                  ("share/edgepick_vla", ["package.xml"])],
      entry_points={"console_scripts": ["smolvla_node=edgepick_vla.smolvla_node:main"]})
