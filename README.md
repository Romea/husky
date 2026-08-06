# husky

## Overview

`husky` groups the ROS2 packages that describe, launch and control the Husky mobile base in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `husky` | Metapackage that groups the Husky ROS2 packages. |
| `husky_description` | Robot-specific description layer for Husky, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `husky_bringup` | Main integration entry point for generating Husky configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `husky_hardware` | Live `ros2_control` hardware plugin for the Husky mobile base, built on the generic `4WD` hardware abstraction. |

## Usage

In most cases, start with `husky_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when a Husky model is selected from a mobile base meta-description.

The Husky stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `4WD` and it is commanded as a skid-steering robot; `husky_description` provides the concrete geometry and generated descriptions, `husky_hardware` provides the live hardware implementation, and `husky_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `husky` project was developed by Jean Laneurit in the context of the TIRREX ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
