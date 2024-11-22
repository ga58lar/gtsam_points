FROM ubuntu:22.04

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y software-properties-common locales && \
    add-apt-repository universe
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Create a non-root user
ARG USERNAME=local
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc

  RUN apt-get update && apt-get install -y -q --no-install-recommends \
  git \    
  vim \
  build-essential \
  cmake \
  make \
  gdb \
  wget \
  libboost-all-dev \
  libomp-dev \
  libtbb-dev \
  libglm-dev \
  libglfw3-dev \
  libpng-dev \
  libjpeg-dev \
  libgtest-dev

# Install specific version of Eigen
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz  && \
    tar -xf eigen-3.3.7.tar.gz && \
    cd eigen-3.3.7 && \
    mkdir build && cd build && \
    cmake .. && \
    make install

WORKDIR /dev_ws

# Irindescence
RUN apt-get update && apt-get install -y -q --no-install-recommends curl gnupg
RUN mkdir -m 0755 -p /etc/apt/keyrings/
RUN curl -fsSL https://koide3.github.io/ppa/ubuntu2204/KEY.gpg | gpg --dearmor -o /etc/apt/keyrings/koide3_ppa.gpg
RUN echo "deb [signed-by=/etc/apt/keyrings/koide3_ppa.gpg] https://koide3.github.io/ppa/ubuntu2204 ./" | tee /etc/apt/sources.list.d/koide3_ppa.list > /dev/null
RUN chmod 644 /etc/apt/keyrings/koide3_ppa.gpg && chmod 644 /etc/apt/sources.list.d/koide3_ppa.list
RUN apt-get update && apt-get install -y -q --no-install-recommends libiridescence-dev


ENV QT_DEBUG_PLUGINS=1

RUN git clone https://github.com/borglab/gtsam && \
    cd gtsam && \
    git checkout 4.2a9 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install

# RUN cd /dev_ws/ && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && \
#     make -j$(nproc) && \
#     make install

ENV LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH

CMD [ "bash" ]