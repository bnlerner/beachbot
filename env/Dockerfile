# syntax = docker/dockerfile:1.4.1
# note: the above comment has an effect on the way this file is read.
FROM osrf/ros2:nightly

ENV DEBIAN_FRONTEND=noninteractive
ENV NODE_MAJOR=20

# RUN apt-get update && apt-get install -y \
#     gnupg-agent=2.2.19-3* \
#     ca-certificates=20230311* \
#     curl=7.68.0-1* \
#     software-properties-common=0.99.9.12

# RUN mkdir -p /etc/apt/keyrings && \
#     # Yarn
#     curl -fsSL https://dl.yarnpkg.com/debian/pubkey.gpg | gpg --dearmor -o /etc/apt/keyrings/yarn.gpg && \
#     echo "deb [signed-by=/etc/apt/keyrings/yarn.gpg] https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list && \
#     # Docker
#     curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg && \
#     echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu focal stable" | tee /etc/apt/sources.list.d/docker.list && \
#     # Tailscale
#     curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.gpg | gpg --dearmor -o /etc/apt/keyrings/tailscale.gpg && \
#     curl -fsSL https://pkgs.tailscale.com/stable/ubuntu/focal.list | sed 's/^deb/deb [signed-by=\/etc\/apt\/keyrings\/tailscale.gpg]/' | tee /etc/apt/sources.list.d/tailscale.list && \
#     # NodeSource
#     curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg && \
#     echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list && \
#     # Deadsnakes
#     curl -fsSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xF23C5A6CF475977595C89F51BA6932366A755776' | gpg --dearmor -o /etc/apt/keyrings/deadsnakes.gpg && \
#     echo "deb [signed-by=/etc/apt/keyrings/deadsnakes.gpg] https://ppa.launchpadcontent.net/deadsnakes/ppa/ubuntu focal main" | tee /etc/apt/sources.list.d/deadsnakes.list

RUN apt-get update && apt-get install -y \
    pipx \
    python3-venv
    # build-essential=12.8* \
    # can-utils=2018.02.0-1* \
    # docker-ce-cli=5:20.10.17~3-0* \
    # flex=2.6.4-6.2 \
    # git=1:2.25.1-1* \
    # gosu=1.10-1* \
    # ipmiutil=3.1.5-1 \
    # iproute2=5.5.0-1* \
    # iputils-arping=3:20190709-3 \
    # iputils-ping=3:20190709-3 \
    # kmod=27-1* \
    # libboost-dev=1.71.0.0* \
    # libc++-dev=1:10.0-50~exp1 \
    # libcanberra-gtk3-module=0.30-7* \
    # libegl1=1.3.2-1* \
    # libffi-dev=3.3-4 \
    # libgl1=1.3.2-1* \
    # libglvnd0=1.3.2-1* \
    # libglx0=1.3.2-1* \
    # libjpeg-dev=8c-2* \
    # libnss3-dev=2:3.49.1-1* \
    # libpng16-16=1.6.37-2 \
    # libpq-dev=12.17-0* \
    # libssl-dev=1.1.1f-1* \
    # libssl1.1=1.1.1f-1* \
    # libx11-6=2:1.6.9-2* \
    # libxext6=2:1.3.4-0* \
    # linuxptp=1.9.2-1 \
    # nodejs=20.10.0* \
    # pciutils=1:3.6.4-1* \
    # postgresql-client=12+214* \
    # python3.11-full=3.11.7-1* \
    # python3.11-dev=3.11.7-1* \
    # redis-tools=5:5.0.7-2* \
    # socat=1.7.3.3-2 \
    # swig=4.0.1-5build1 \
    # # Don't pin bc we always want the latest version of Tailscale
    # tailscale \
    # xvfb=2:1.20.13-1* \
    # yarn=1.22.19-1

# RUN python3.11 -m ensurepip && \
#     python3.11 -m pip install pip-tools==7.3.0 pex==2.1.154 pip==23.3.2

# # This adds nvidia's PyPI repo to so that the tritonclient package can be found.
# RUN pip config --user set global.extra-index-url https://pypi.ngc.nvidia.com
# RUN pip config --user set global.trusted-host pypi.ngc.nvidia.com

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe

# Set working dir.
ARG DOCKER_PROJECT_DIR=beachbot
RUN mkdir -p ${DOCKER_PROJECT_DIR}
WORKDIR ${DOCKER_PROJECT_DIR}

# Install Python dependencies from requirements.txt.
COPY env/python_requirements/requirements.txt .

RUN python3 -m venv .botenv

# RUN pipx ensurepath
RUN python3 -m /botenv/bin/pip install \
    odrive==0.6.8

# Install Lucid camera dependencies.
# COPY env/scripts/install_lucid_dependencies.sh /tmp/install_lucid_dependencies.sh
# RUN chmod +x /tmp/install_lucid_dependencies.sh && bash /tmp/install_lucid_dependencies.sh

# Install our Python packages.
# RUN mkdir -p src/python/potomac && \
#     mkdir -p src/python/sinai && \
#     mkdir -p src/python/marianactl && \
#     mkdir -p src/python/built_lib && \
#     mkdir -p src/python/cirrus

# COPY src/python/setup.py src/python
# RUN cd src/python && python3.11 -m pip install -e .

# Build ap225.
# RUN mkdir -p src/c/acromag/AP225 && \
#     mkdir -p src/c/acromag/apcommon
# COPY src/c/acromag src/c/acromag

# RUN cd src/c/acromag/AP225 && \
#     python3.11 -m pip install . && \
#     cd

# ENV PATH src/ts/everest/node_modules/.bin:$PATH

# Set up IPython-Mariana.
# RUN echo 'alias ipython-mariana="python3.11 -m IPython -i \
#     /mariana/env/conf_files/ipython_mariana_startup.py"' >> /etc/bash.bashrc

# Set python3.11 as default for 'python3', but keep system default python3 as an alternative.
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 2
# RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1
