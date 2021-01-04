# ros-industrial-training

This repository contains practice code of [ros industrial training melodic branch](https://industrial-training-master.readthedocs.io/en/melodic/).

### Observations

- Following [ROS-Setup](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/ROS-Setup.html), it is quite handy to go through [Wiki page of ROS Qt Creator plug-in](https://ros-qtc-plugin.readthedocs.io/en/latest/index.html). This will help new developers to get [Qt configured](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/Import-ROS-Workspace.html) with ROS Workspace, and handle the development of ROS projects from QtCreator.

- Under [Download and Build a Package from Source](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Installing-Existing-Packages.html#download-and-build-a-package-from-source), a ros package [fake_ar_publisher](https://github.com/ros-industrial/fake_ar_publisher) is suggested. This is a necessary package to work through the rest of the training. However, for the ros industrial training based on ros melodic, you need to clone to [master branch of fake_ar_publisher](https://github.com/ros-industrial/fake_ar_publisher/tree/master)

  `git clone -b master https://github.com/jmeyer1292/fake_ar_publisher.git`

- Concerning Continuous Integration (CI), there is an infrastructure [industrial_ci](https://github.com/ros-industrial/industrial_ci) from ros industrial. However, given the active development on [ros-tooling](https://github.com/ros-tooling), I decided to use the Github Actions based CI from ros-tooling. This is updated at [ros-ci.yml](.github/workflows/ros-ci.yml). 
