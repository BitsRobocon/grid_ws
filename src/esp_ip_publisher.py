# Reference - https://github.com/KebabLord/esp2python	

import urllib.request
import time

esp_ip = "192.168.xx.xx"
root_url = "http://" + esp_ip  

def sendRequest(url):
	n = urllib.request.urlopen(url) # send request to ESP

def sendCommand(command_string): # command_string = LEFT/RIGHT/FRONT/BACK/STOP
	sendRequest(root_url+"/"+command_string)
	print("Command Sent = "+command_string) # Comment if not needed

##########

# Basic Tele-Operation Code:-
while True:
	input_key = input("Input w/a/s/d for Teleop. Input q to stop. Input f to exit.\nPress enter after input!\n")
	if (input_key.upper() == "W"):
		sendCommand("FRONT")		
	if (input_key.upper() == "S"):
		sendCommand("BACK")
	if (input_key.upper() == "A"):
		sendCommand("LEFT")
	if (input_key.upper() == "D"):
		sendCommand("RIGHT")
	if (input_key.upper() == "Q"):
		sendCommand("STOP")
	if (input_key.upper() == "F"):
		exit()

##########

#Basic Pre-Defined Test

# time.sleep(1)
# sendCommand("LEFT")
# time.sleep(1)
# sendCommand("RIGHT")
# time.sleep(1)
# sendCommand("FRONT")
# time.sleep(1)
# sendCommand("BACK")
# time.sleep(1)
# sendCommand("STOP")


	
