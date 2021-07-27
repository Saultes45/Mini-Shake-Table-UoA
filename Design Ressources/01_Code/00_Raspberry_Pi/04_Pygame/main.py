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
#   https://gist.github.com/illume/83cf4efcfc7988ade95f7dfeaad40171
#   F:\Dropbox\01_PYTHON\01_RPI\USV2\SimuFromScratch.py <<--- for logs, serial and .ini files
#
#
#
# TODO
# What about everything from GUI to serial port handeling
# ========================================


# -------------------------- IMPORT --------------------------
import pygame         # install: pip3 install pygame
import serial               # install: pip3 install pyserial
import warnings             ## for removing the warnings we get


# -------------------------- GLOBAL VARIABLES --------------------------
SOM = "$$"
EOM = "\r\n"

XIAO_COM_PORT = "/dev/tty/AMA0"
XIAO_COM_PORT = "COM19"
XIAO_BAUDRATE = 921600

background_colour = (255,255,255)
(WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT) = (300, 200)

# -------------------------- ISR/EVENTS --------------------------


# -------------------------- DEF --------------------------
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi there, {name}')  # Press Ctrl+F8 to toggle the breakpoint.



# -------------------------- MAIN --------------------------
# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    print_hi('User')

    ##------------------Warning------------------------
    warnings.filterwarnings("ignore")

    ##------------------ Boolean states ------------------------
    InitialisationIsDone    = False
    SerialPortIsOpened      = False

    ##------------------ Window/GUI ------------------------
    screen = pygame.display.set_mode((WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT))
    pygame.display.set_caption('Shake Table Control')
    screen.fill(background_colour)
    pygame.display.flip()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    ##------------------Serial Port------------------------
    # ser = serial.Serial(XIAO_COM_PORT, XIAO_BAUDRATE, timeout=0)
    # serial_buffer = ''
    #
    # _, going = pg.init(), pg.display.set_mode((WINDOW_SIZE_WIDTH, WINDOW_SIZE_HEIGHT))
    # pg.display.set_caption('Shake Table Control')
    # while going:
    #     # loop whilst there is data to be read from the serial port
    #     serial_data = ser.read()
    #     while serial_data:
    #         serial_buffer += serial_data
    #         # If there is a new line, we send the event.
    #         if '\r\n' in serial_buffer:
    #             evt = pg.event.Event(SERIAL, line=serial_buffer)
    #             pg.event.post(evt)
    #             serial_buffer = ''
    #         serial_data = ser.read()



# END OF THE FILE
