### ROS Industrial Training

This repository contains practice code of [ros industrial training melodic branch](https://industrial-training-master.readthedocs.io/en/melodic/).

[![ROS CI Action Status](https://github.com/Yeshasvitvs/ros-industrial-training/workflows/ROS%20CI/badge.svg)](https://github.com/Yeshasvitvs/ros-industrial-training/actions?query=workflow%3A%22ROS+CI%22)

##### Observations

- Following [ROS-Setup](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/ROS-Setup.html), it is quite handy to go through [Wiki page of ROS Qt Creator plug-in](https://ros-qtc-plugin.readthedocs.io/en/latest/index.html). This will help new developers to get [Qt configured](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/Import-ROS-Workspace.html) with ROS Workspace, and handle the development of ROS projects from QtCreator.

- Under [Download and Build a Package from Source](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Installing-Existing-Packages.html#download-and-build-a-package-from-source), a ros package [fake_ar_publisher](https://github.com/ros-industrial/fake_ar_publisher) is suggested. This is a necessary package to work through the rest of the training. However, for the ros industrial training based on ros melodic, you need to clone to [master branch of fake_ar_publisher](https://github.com/ros-industrial/fake_ar_publisher/tree/master)

  `git clone -b master https://github.com/jmeyer1292/fake_ar_publisher.git`

- Concerning Continuous Integration (CI), there is an infrastructure [industrial_ci](https://github.com/ros-industrial/industrial_ci) from ros industrial. However, given the active development on [ros-tooling](https://github.com/ros-tooling), I decided to use the Github Actions based CI from ros-tooling. This is updated at [ros-ci.yml](.github/workflows/ros-ci.yml). 


- [ros industrial training melodic branch](https://industrial-training-master.readthedocs.io/en/melodic/) is based on [catkin build system](http://wiki.ros.org/catkin/Tutorials). However, the Continuous Integration (CI) based on [ros-tooling](https://github.com/ros-tooling) with [action-ros-ci](https://github.com/ros-tooling/action-ros-ci) considers [colcon build system](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/). Colcon build system needs ros packages to be installed in order to discover ros packages and use ros commands as pointed in [this reference from aws](https://docs.aws.amazon.com/robomaker/latest/dg/troubleshooting-colcon.html). Accordingly, the installation of `myworkcell_core` and `myworkcell_support` ros packates is updated in https://github.com/Yeshasvitvs/ros-industrial-training/commit/51dd5e232841e66f0653e7e41bb5ee7591459a55. 
