import os
import subprocess
from time import sleep

class RPi_access_point:
  def start(self):
    log=open("commonLog", "w",0)

    print "### starting {} ###".format(self.__class__.__name__)
    log=open("{}_log".format(self.__class__.__name__), "w",0)
    ### add AP interface
    ap0_address = "192.168.27.1/24"
    try:
      print "### Adding virtual wireless interface of type AccessPoint"
      log.write( subprocess.check_output(["iw", "phy", "phy0", "interface", "add", "ap0", "type", "__ap"]) )

      print "### Setting it's address. Making sure its subnet matches dhcp-range of ips assigned by dnsmasq"
      log.write( subprocess.check_output(["ip", "addr", "add", ap0_address,  "dev", "ap0"]) )

      print "Bringing ap0 interface UP"
      log.write( subprocess.check_output(["ip", "link", "set", "ap0", "up"]) )

    except subprocess.CalledProcessError,e:
      print "Error in start"
      log.write(str(e))
      return 0
    
    ### hostapd
    print "### Starting hostAPD daemon, to facilitate connections to Access Point"
    hostapd_log=open("hostAPD_log", "w",0)
    hostAPD_proc = subprocess.Popen(["hostapd", "-dd", "/configs/hostapd.conf"], stdout=subprocess.PIPE, stderr=hostapd_log)
    
    print "### waiting for Access Point to be enabled"
    while(True):
      output = hostAPD_proc.stdout.readline()
      if("AP-ENABLED" in output):
        print "Everything is OK: AP-enabled"
        print output
        break;

    print "### Starting dnsmaqs to enable DHCP for clients connected to AP"
    dnsmasq_log=open("dnsmasq_log", "w",0)
    # print "### Setting default gateway to be used in dnsmasq config file"
    # defGateway ="192.168.27.1"
    # print "default gateway: {}".format(defGateway)
    # print "### appending default gateway to dnsmasq.conf"
    # gatewayStr= "dhcp-option=ap0,3,{}".format(defGateway)
    # os.system("echo '{}' >> /configs/dnsmasq.conf".format(gatewayStr))
    print "### starting dnsmasq daemon. To facilitate connection of AP clients"

    dnsmasq_proc = subprocess.Popen(["dnsmasq", "--log-queries", "-C", "/configs/dnsmasq.conf"], stdout=dnsmasq_log, stderr=dnsmasq_log)
     
    
    ### wifi client
    #wifiClient_log=open("wifiClient_log", "w",0)
    #print "### Starting wpa_supplicant, to enable connection to wifi device"
    #wifi_proc = subprocess.Popen(["wpa_supplicant", "-d", "-Dnl80211", "-iwlan0" ,"-c", "/etc/wpa_supplicant/wpa_supplicant_iotwifi.conf"],stdout=subprocess.PIPE, stderr=wifiClient_log)
    #print "### waiting for established connection with WiFi"

    #while(True):
      #output = wifi_proc.stdout.readline()
      #if("CTRL-EVENT-CONNECTED" in output):
        #print "connection to the wifi was established"
        #print output
        #break;

    print '\n'*2
    print "Waiting for termination of started children processes"
    #wifi_proc.wait() 
    hostAPD_proc.wait()
    dnsmasq_proc.wait()

RPi_ap = RPi_access_point()
RPi_ap.start()
