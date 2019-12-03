echo "Preparing kotyambaCar"

export kotyambaCar_path="$(pwd)"
echo "Settting kotyambaCar_path to $kotyambaCar_path"

echo "preparing environment"
sudo apt install python-pip
pip --version
pip install virtualenv
echo 'PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
. ~/.bashrc
virtualenv env
. /env/bin/activate

echo "Using systemd to setup starting of Tornado web server on boot"
sudo cp src/control_server/start_server.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable start_server.service --now

# check service status:  
systemctl status start_server.service
