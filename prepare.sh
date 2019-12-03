echo "Preparing kotyambaCar"

export kotyambaCar_path $(pwd)
echo "Settting kotyambaCar_path to $kotyambaCar_path"

echo "Using systemd to setup starting of Tornado web server on boot"
sudo cp src/control_motors/start_server.service /etc/systemd/system
sudo systemctl daemon-reload
# make service running on system boot:  
sudo systemctl enable start_server.service --now

# check service status:  
systemctl status start_server.service
