echo "Preparing kotyambaCar"

# TODO consider using virtualenv, but how to reboot for changes to take effect
echo "1. Install all required packages"

#echo "preparing environment"
sudo apt-get update -y
sudo apt install python-pip -y
python -m pip install tornado
python -m pip install RPi.GPIO
sudo apt-get install motion -y

#pip --version
#pip install virtualenv
#echo 'PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc
#. ~/.bashrc
#virtualenv env
#. /env/bin/activate

echo "Using systemd to setup starting of Tornado web server on boot"
sudo cp src/control_server/start_server.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable start_server.service --now

# check service status:  
systemctl status start_server.service
