#!/bin/bash
sudo add-apt-repository ppa:deadsnakes/ppa && \
sudo apt update && sudo apt-get update && \
sudo apt install terminator software-properties-common apt-transport-https wget screen libqt5core5a -y && \
sudo apt-get install curl python3-pip python3.12 python3.12-dev python3.12-tk gpsd gpsd-clients jq proj-bin zstd -y && \
sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hostname && \
sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hosts && \
sudo reboot
sudo curl -fsSL https://tailscale.com/install.sh | sh && \
sudo tailscale up && \
# Install anaconda3
curl -sS https://bootstrap.pypa.io/get-pip.py | python3.12 && \
sudo gpasswd -a $USER docker && \
sudo mkdir -p -m 755 /etc/apt/keyrings && \
wget -qO- https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && \
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && \
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
sudo apt update && sudo apt install gh -y && \
gh auth login && \
gh repo clone havenrobotics/beachbot && \
git config --global user.email "bnlerner@gmail.com" && \
git config --global user.name "Brian Lerner" && \
cd beachbot && python3 -m pip install -r env/python_requirements/requirements.txt && \
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger" && \
sudo usermod -a -G dialout brianlerner && \
sudo mkdir -p /var/log/beachbot && \
sudp mkdir -p var/locks && \
sudo chown brianlerner /var/log/beachbot && \
sudo nmcli connection modify BrianPhone ipv4.route-metric 100 && \
sudo nmcli connection modify BrianPhone connection.autoconnect-priority 100 && \
sudo nmcli connection modify "starlink was better" ipv4.route-metric 50 && \
sudo nmcli connection modify "starlink was better" connection.autoconnect-priority 50 && \
wget https://stereolabs.sfo2.cdn.digitaloceanspaces.com/utils/drivers/ZEDX/1.2.1/R35.4/stereolabs-zedlink-duo_1.2.1-LI-MAX96712-ALL-L4T35.4.1_arm64.deb && \
sudo dpkg -i stereolabs-zedlink-duo_1.2.1-LI-MAX96712-ALL-L4T35.4.1_arm64.deb && \
wget -O ZED_SDK_Tegra_L4T35.4_v4.2.2.zstd.run https://download.stereolabs.com/zedsdk/4.2/l4t35.4/jetsons?_gl=1*12hkj8x*_gcl_au*OTQwMDMzNDAxLjE3MzMzMzc1NjU. && \
chmod +x ZED_SDK_Tegra_L4T35.4_v4.2.2.zstd.run && \
./ZED_SDK_Tegra_L4T35.4_v4.2.2.zstd.run && \
sudo cp ~/beachbot/env/scripts/setup_can.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/setup_can.sh && \
sudo cp env/systemd/*.service /etc/systemd/system/ && \
sudo systemctl enable can_setup.service && sudo systemctl enable beachbot_startup.service && \
sudo pip3 install -U jetson-stats && \
wget https://ompl.kavrakilab.org/core/install-ompl-ubuntu.sh && chmod u+x install-ompl-ubuntu.sh && \
./install-ompl-ubuntu.sh --python && \
sudo reboot
