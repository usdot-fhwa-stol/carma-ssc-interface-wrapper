| CicleCI Build Status | Sonar Code Quality | DockerHub Release | DockerHub Release Candidate | DockerHub Develop |
|------|-----|-----|-----|-----|
[![CircleCI](https://img.shields.io/circleci/build/gh/usdot-fhwa-stol/carma-ssc-interface-wrapper/develop?label=CircleCI)](https://app.circleci.com/pipelines/github/usdot-fhwa-stol/carma-ssc-interface-wrapper?branch=develop) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=usdot-fhwa-stol_CARMASscInterfaceWrapper&metric=alert_status)](https://sonarcloud.io/dashboard?id=usdot-fhwa-stol_CARMASscInterfaceWrapper) | [![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastol/carma-ssc-interface-wrapper?label=carma-ssc-interface-wrapper)](https://hub.docker.com/repository/docker/usdotfhwastol/carma-ssc-interface-wrapper) | [![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastolcandidate/carma-ssc-interface-wrapper?label=carma-ssc-interface-wrapper)](https://hub.docker.com/repository/docker/usdotfhwastolcandidate/carma-ssc-interface-wrapper) | [![Docker Cloud Build Status](https://img.shields.io/docker/cloud/build/usdotfhwastoldev/carma-ssc-interface-wrapper?label=carma-ssc-interface-wrapper)](https://hub.docker.com/repository/docker/usdotfhwastoldev/carma-ssc-interface-wrapper)

# CARMASscInterfaceWrapper
CARMASscInterfaceWrapper contains a driver wrapper for usage with the Autoware speed and steering control interface in combination with the AutonomouStuff controller and driver

To add this package to your CARMAPlatform project simply clone this repository into your Catkin workspace

# CARMAPlatform
The primary CARMA Platform repository can be found [here](https://github.com/usdot-fhwa-stol/carma-platform) and is part of the [USDOT FHWA STOL](https://github.com/usdot-fhwa-stol/)
github organization. Documentation on how the CARMA Platform functions, how it will evolve over time, and how you can contribute can be found at the above links as well.

## Current Status of ROS 1 and ROS 2 Packages
The current CARMA Platform system operates as a hybrid of ROS 1 Noetic and ROS 2 Foxy components. While nearly all components have been upgraded to ROS 2 Foxy, a small amount of ROS 1 Noetic content still exists within the system, including some packages in this repository. For more information, please see the relevant documentation in the [carma-config](https://github.com/usdot-fhwa-stol/carma-config?tab=readme-ov-file#current-status-of-hybrid-ros-1ros-2-system) repository.

## Contribution
Welcome to the CARMA contributing guide. Please read this guide to learn about our development process, how to propose pull requests and improvements, and how to build and test your changes to this project. [CARMA Contributing Guide](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Contributing.md)

## Code of Conduct
Please read our [CARMA Code of Conduct](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/Code_of_Conduct.md) which outlines our expectations for participants within the CARMA community, as well as steps to reporting unacceptable behavior. We are committed to providing a welcoming and inspiring community for all and expect our code of conduct to be honored. Anyone who violates this code of conduct may be banned from the community.

## Attribution
The development team would like to acknowledge the people who have made direct contributions to the design and code in this repository. [CARMA Attribution](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/ATTRIBUTION.txt)

## License
By contributing to the Federal Highway Administration (FHWA) Connected Automated Research Mobility Applications (CARMA), you agree that your contributions will be licensed under its Apache License 2.0 license. [CARMA License](https://github.com/usdot-fhwa-stol/carma-platform/blob/develop/docs/License.md)

### NOTE

Some of the packages within this repo contain different licensing restrictions. Consult the README in each package for details.

## Contact
Please click on the CARMA logo below to visit the Federal Highway Adminstration(FHWA) CARMA website.

[![CARMA Image](https://raw.githubusercontent.com/usdot-fhwa-stol/carma-platform/develop/docs/image/CARMA_icon.png)](https://highways.dot.gov/research/research-programs/operations/CARMA)
