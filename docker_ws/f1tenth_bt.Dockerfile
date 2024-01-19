# Base image
FROM ros:foxy

# Set the working directory
WORKDIR /

# Copy files from the host to the container
# COPY <source> <destination>

# Install dependencies
RUN apt-get update && \
    apt-get install -y \ 
    git \
    neovim \
    tmux \
    python3-pip

# f1tenth
RUN mkdir -p /mnt/f1tenth_ws/src && \
    cd /mnt/f1tenth_ws && \
    colcon build

# Startup
RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# Expose ports
# EXPOSE <port>

# Set environment variables
# ENV <key>=<value>

# Run commands when the container starts
# CMD <command>

# BUILD COMMAND
# docker build -f f1tenth_bt.Dockerfile -t bentjh01:f1tenth-foxy .
