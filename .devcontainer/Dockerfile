# Use the official ROS image as the base
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

# Run everything as root (already default in most Docker images)
USER root

# Install system and graphical dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    python3-pyqt5 \
    pyqt5-dev-tools \
    libqt5webkit5-dev \
    qttools5-dev-tools \
    libserialport-dev \
    curl \
    sudo \
    alsa-utils \
    pulseaudio \
    pulseaudio-utils \
    libgl1-mesa-glx \
    libegl1 \
    mesa-utils \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libx11-dev \
    libxkbcommon-x11-0 \
    xauth \
    xvfb \
    ros-${ROS_DISTRO}-cv-bridge \  
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Setup Python virtual environment
RUN python3 -m pip install virtualenv \
    && virtualenv /env \
    && . /env/bin/activate

# Install Python packages
RUN pip install -v \
    opencv-python \
    PySide6 \
    pyserial \
    psutil \
    facenet-pytorch \
    torch \
    torchvision

# Install Arduino CLI
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh && \
    arduino-cli config init --additional-urls http://downloads.arduino.cc/packages/package_index.json && \
    arduino-cli core update-index && \
    arduino-cli core install arduino:avr && \
    arduino-cli lib install "AccelStepper"

# Append source commands to .bashrc for convenience
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo "source /env/bin/activate" >> /root/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc

# Set the default shell to bash
ENV SHELL /bin/bash

# Set the working directory to /root (or choose your preferred working directory)
WORKDIR /root

CMD ["/bin/bash"]
