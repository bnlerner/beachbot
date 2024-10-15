#!/bin/bash
sudo apt update && \
       sudo apt-get update && \
       sudo apt install terminator software-properties-common apt-transport-https wget -y && \
       sudo apt-get install curl -y && \
       sudo curl -fsSL https://tailscale.com/install.sh | sh && \
       sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hostname && \
       sudo sed -i 's/ubuntu/beachbot-1/g' /etc/hosts && \
       sudo tailscale up && \
       sudo gpasswd -a $USER docker && \
       wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add - && \
       sudo add-apt-repository "deb [arch=arm64] https://packages.microsoft.com/repos/vscode stable main" && \
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
       sudo reboot 
