#!/usr/bin/env python

import smtplib

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

import subprocess
import time

def check_for_wifi():
    ps = subprocess.Popen(['iwgetid'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        network_output = subprocess.check_output(('grep', 'ESSID'), stdin=ps.stdout)
        print(network_output)

        output = subprocess.run(
            ['ifconfig','wlan0'],
            check=True, 
            stdout=subprocess.PIPE, 
            universal_newlines=True
            )

        print(output.stdout)

        fromaddr = "p21321rov@gmail.com"
        toaddr = "jimbosned@gmail.com"

        msg = MIMEMultipart()
        msg['From'] = fromaddr
        msg['To'] = toaddr
        msg['Subject'] = "ROVert found the internet!"
        body = output.stdout
        msg.attach(MIMEText(body, 'plain'))

        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()
        server.login(fromaddr, "WNtwht9YxwetGxL")
        text = msg.as_string()
        server.sendmail(fromaddr, toaddr, text)
        server.quit()

        return 0

    except subprocess.CalledProcessError:
        # grep did not match any lines
        # print("No wireless networks connected")
        return -1

while(1):
    time.sleep(5)
