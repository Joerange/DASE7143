#################################################
#         (C) Keysight Technologies 2019 
#
# PROPRIETARY RIGHTS of Keysight Technologies are 
# involved in the subject matter of this software. 
# All manufacturing, reproduction, use, and sales 
# rights pertaining to this software are governed 
# by the license agreement. The recipient of this 
# code implicitly accepts the terms of the license. 
#
###################################################
#
# FILE NAME  : M1_L2_E5b_Cloud_LCD.py      
# DESCRIPTION: This program gets weather data from
#			   the internet and prints it to the LCD
# #################################################
import os
import subprocess
import time

UPDATE_INTERVAL = 3.0  # seconds between the start of temperature updates
SCRIPT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "weather_lcd.js")

while True:
    started_at = time.monotonic()
    subprocess.call(["node", SCRIPT_PATH])
    # Include the weather request and LCD write time in the 3-second cycle.
    elapsed = time.monotonic() - started_at
    time.sleep(max(0.0, UPDATE_INTERVAL - elapsed))
