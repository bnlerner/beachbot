#!/bin/bash
sudo add-apt-repository ppa:deadsnakes/ppa && \
sudo apt update && sudo apt-get update && \
sudo apt install terminator software-properties-common apt-transport-https wget -y && \
sudo apt-get install curl python3-pip python3.12 python3.12-dev python3.12-tk gpsd gpsd-clients jq -y && \
sudo apt install python3.7-distutils python3.8-distutils python3.9-distutils python3.10-distutils python3.11-distutils python3-12-distutils && \
curl -sS https://bootstrap.pypa.io/get-pip.py | python3.12 && \
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
sudo update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.12 2 && \
sudo update-alternatives --config python3 && \
sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hostname && \
sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hosts && \
sudo curl -fsSL https://tailscale.com/install.sh | sh && \
sudo tailscale up && \
sudo gpasswd -a $USER docker && \
wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
sudo add-apt-repository "deb [arch=$(dpkg --print-architecture)] https://packages.microsoft.com/repos/vscode stable main" && \
sudo apt install code && \
sudo mkdir -p -m 755 /etc/apt/keyrings && \
wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && \
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
sudo apt update && sudo apt install gh -y && \
gh auth login && \
gh repo clone havenrobotics/beachbot && \
git config --global user.email "bnlerner@gmail.com" && \
git config --global user.name "Brian Lerner" && \
cd beachbot && \ pip3 install -r env/python_requirements/requirements.txt && \
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger" && \
sudo usermod -a -G dialout brianlerner && \
sudo mkdir -p /var/log/beachbot && \
sudo chown brianlerner /var/log/beachbot && \
sudo reboot
