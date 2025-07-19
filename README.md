> **TO THE 2026 BYU AGROBOTICS TEAM:** This repository is based on the structure of the BYU Mars Rover Capstone Team's repository available [here](https://github.com/snelsondurrant/marsrover_2.0). Refer to that repository for examples and best practices for ROS2 and Docker development.

[Get Started](https://github.com/byu-agrobotics/agrobot_2.0?tab=readme-ov-file#get-started)

[Essential Tutorials](https://github.com/byu-agrobotics/agrobot_2.0?tab=readme-ov-file#essential-tutorials)

[Contributing](https://github.com/byu-agrobotics/agrobot_2.0?tab=readme-ov-file#contributing)

--

### Get Started

> **NOTE:** Newer Macs that use ARM64 architecture (M1, M2, etc) are missing support for several popular ROS2 packages, and have not been extensively tested.

**Windows:**

- Install WSL2 on your Windows machine by following the instructions [here](https://docs.microsoft.com/en-us/windows/wsl/install).

- Install Docker Desktop on your Windows machine by following the instructions [here](https://docs.docker.com/desktop/), and enable the WSL2 backend by following the instructions [here](https://docs.docker.com/desktop/windows/wsl/).

- Open a WSL2 terminal and clone the agrobot_2.0 repo into your WSL2 environment using `git clone https://github.com/byu-agrobotics/agrobot_2.0.git`.

- Run `cd agrobot_2.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

**Linux:**

- Install Docker Engine on your Linux machine by following the instructions [here](https://docs.docker.com/engine/install/ubuntu/).

- Open a terminal and clone the agrobot_2.0 repo into your Linux environment using `git clone https://github.com/byu-agrobotics/agrobot_2.0.git`.

- Run `cd agrobot_2.0 && bash compose.sh` to pull and launch the latest Docker image from DockerHub.

--

### Essential Tutorials

> **NOTE:** <mark>We would **strongly encourage** each year's team to take a couple of months at the beginning of the first semester and simply work together through these tutorials before diving into software development.</mark> It may not seem romantic, but I promise it'll be worth it.

[Linux CLI Tutorial](https://linuxjourney.com/lesson/the-shell)

[GitHub Basics Tutorial](https://docs.github.com/en/get-started/start-your-journey/hello-world)

[Docker Concepts and Tutorials](https://docs.docker.com/get-started/introduction/whats-next/)

[ROS2 Concepts](https://docs.ros.org/en/humble/Concepts/Basic.html)

[ROS2 CLI Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html)*

[ROS2 Code Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries.html)*

[ROS2 Discovery Server Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Discovery-Server/Discovery-Server.html)*

[Robotics in ROS2 Tutorial](https://github.com/henki-robotics/robotics_essentials_ros2/tree/main)

**All of the dependencies for these tutorials are preinstalled in the Docker container, and we've mounted 'tutorial_ws' as a dedicated ROS2 tutorial workspace.*

--

### Contributing

- **Create a new branch.** This repository's main branch is protected, so you'll need to create a new branch. Name your branch with a combination of your name and the feature you are working on (e.g. nelson/repo-docs).

- **Make your changes.** Develop and debug your new feature or bug fix. Add good documentation.

    > **NOTE:** If you need to add dependencies, add them to the Dockerfile in your branch and test building the image locally. Once your pull request is merged into main, the GitHub CI workflow will automatically build and push the new Docker image to DockerHub.

- **Rebase your branch often.** Keep your branch up-to-date with main by rebasing. This will help prevent merge conflicts down the road.

- **Submit a pull request.** Once you have made and tested your changes, create a new pull request. Get another member of the team to review and approve it, and you can merge your new code into the main branch.

--

Created by Nelson Durrant, July 2025.
