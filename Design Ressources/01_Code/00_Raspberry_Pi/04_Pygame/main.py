# ========================================
#
# Copyright University of Auckland Ltd, 2021
# All Rights Reserved
# UNPUBLISHED, LICENSED SOFTWARE.
#
# Metadata
# Written by    : Nathanaël Esnault
# Verified by   : Nathanaël Esnault
# Creation date : 2021-07-27
# Version       : 0.1 (finished on 2021-..-..)
# Modifications :
# Known bugs    :
#
#
# Possible Improvements
#
# Notes
#
#
# Ressources (Boards + Libraries Manager)
# https://gist.github.com/illume/83cf4efcfc7988ade95f7dfeaad40171
#
# TODO
# What about everything from GUI to serial port handeling
# ========================================


# -------------------------- IMPORT --------------------------
import pygame as pg         # install: pip3 install pygame
import serial               # install: pip3 install pyserial

# -------------------------- GLOBAL VARIABLES --------------------------
SOM = "$$"
EOM = "\r\n"

XIAO_COM_PORT = "/dev/tty/AMA0"
XIAO_BAUDRATE = 921600

WINDOW_SIZE_WIDTH   = 320
WINDOW_SIZE_HEIGHT  = 200

# -------------------------- ISR/EVENTS --------------------------
# custom event type is an int based on USEREVENT.
SERIAL = pg.USEREVENT + 2

# -------------------------- DEF --------------------------
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.



# -------------------------- MAIN --------------------------
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('PyCharm')

    ser = serial.Serial(XIAO_COM_PORT, XIAO_BAUDRATE, timeout=0)
    serial_buffer = ''

    _, going = pg.init(), pg.display.set_mode((WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT))
    pg.display.set_caption('Shake Table Control')
    while going:
        # loop whilst there is data to be read from the serial port
        serial_data = ser.read()
        while serial_data:
            serial_buffer += serial_data
            # If there is a new line, we send the event.
            if '\r\n' in serial_buffer:
                evt = pg.event.Event(SERIAL, line=serial_buffer)
                pg.event.post(evt)
                serial_buffer = ''
            serial_data = ser.read()



# END OF THE FILE
