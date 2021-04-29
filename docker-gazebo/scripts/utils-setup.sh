# TMUX
cd && \
git clone https://github.com/gpakosz/.tmux.git  && \
ln -s -f .tmux/.tmux.conf && \
cp .tmux/.tmux.conf.local .


# ZSH
sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.1.1/zsh-in-docker.sh)" -- \
	-p git \
	-p https://github.com/zsh-users/zsh-autosuggestions \
	-p https://github.com/zsh-users/zsh-completions \
	-p https://github.com/zsh-users/zsh-syntax-highlighting 
echo "unsetopt share_history" >> ~/.zsh_profile
echo "unsetopt share_history" >> ~/.zshrc


# VSCODE
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -o root -g root -m 644 packages.microsoft.gpg /etc/apt/trusted.gpg.d/
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
rm -f packages.microsoft.gpg
sudo apt install -y libasound2 apt-transport-https
sudo apt update
sudo apt install -y code
