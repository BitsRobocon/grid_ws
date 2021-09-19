# Reference - https://github.com/KebabLord/esp2python	

import urllib.request
import time

esp_ip = "192.168.137.158"	#change IP address
def setIP(ipaddress):
	global esp_ip
	esp_ip = ipaddress
	return

root_url = "http://" + esp_ip  

def sendRequest(url):
	n = urllib.request.urlopen(url) # send request to ESP

def sendCommand(L_Value, R_Value, Servo_Value):
	print(root_url+"/"+str(L_Value).zfill(4)+str(R_Value).zfill(4)+str(Servo_Value))
	sendRequest(root_url+"/"+str(L_Value).zfill(4)+str(R_Value).zfill(4)+str(Servo_Value))

# sendCommand(50,50,0)
# time.sleep(1)
# sendCommand(50,-50,0)
# time.sleep(1)
# sendCommand(-50,-50,0)
# time.sleep(1)
# sendCommand(100,100,1)
# time.sleep(1)
# sendCommand(0,0,0)
	

