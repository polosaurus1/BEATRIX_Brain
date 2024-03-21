ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Update and install necessary packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    python3-pip \
    curl \
    sudo \
    # Add any other dependencies your project might need here
    && rm -rf /var/lib/apt/lists/*

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Arduino CLI configuration and library installation
RUN arduino-cli config init --additional-urls http://downloads.arduino.cc/packages/package_index.json \
    && arduino-cli core update-index \
    && arduino-cli core install arduino:avr \
    && arduino-cli lib install "AccelStepper"

# Append source commands to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc \
    && echo "source /home/ws/install/setup.bash" >> /root/.bashrc

# Set the default shell
ENV SHELL /bin/bash

# Set the working directory (optional, can be anywhere accessible by root)
WORKDIR /root

CMD ["/bin/bash"]
