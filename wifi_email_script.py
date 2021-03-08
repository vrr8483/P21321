#!/usr/bin/env python

import smtplib

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

import subprocess
import time
import re
from datetime import datetime

# True if Wifi connected, False otherwise
def check_for_wifi():
    ps = subprocess.Popen(['iwgetid'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        network_output = subprocess.check_output(('grep', 'ESSID'), stdin=ps.stdout)
        # print(network_output)

        return True

    except subprocess.CalledProcessError:
        # grep did not match any lines
        # print("No wireless networks connected")
        return False

# returns 0 on success, -1 otherwise
def send_email():

    output = subprocess.run(
            ['ifconfig','wlan0'],
            check=True,
            stdout=subprocess.PIPE,
            universal_newlines=True
            )
    
    #ip_line = subprocess.check_output(('grep', 'ESSID'), stdin=output.stdout)
    ip_line = re.findall("\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}", output.stdout)[0]

    print(output.stdout)
    print(ip_line)
    
    if ip_line is None or len(ip_line) == 0:
        print("No IPv4 address found on wlan0.")
        return -1;

    now = datetime.now() # current date and time

    date_time = now.strftime("%m/%d/%Y, %H:%M:%S")
    print("date and time: ",date_time)	

    fromaddr = "p21321rov@gmail.com"
    toaddr = "jimbosned@gmail.com"

    msg = MIMEMultipart()
    msg['From'] = fromaddr
    msg['To'] = toaddr
    msg['Subject'] = "ROVert found the internet: " + date_time
    body = ip_line
    msg.attach(MIMEText(body, 'plain'))

    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(fromaddr, "WNtwht9YxwetGxL")
    text = msg.as_string()
    server.sendmail(fromaddr, toaddr, text)
    server.quit()

    return 0;


wifi_connected_previously = False

# send_email()

while(1):
	
    if wifi_connected_previously:
        time.sleep(10)
    else:
        time.sleep(5)
    
    wifi_connected_now = check_for_wifi()
    
    if (not wifi_connected_previously) and wifi_connected_now:
        if send_email():
            wifi_connected_now = False
        
    wifi_connected_previously = wifi_connected_now
		
		
