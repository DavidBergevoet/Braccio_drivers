# Braccio_drivers
Drivers for the arduino Braccio. Arduino and Linux code included

# Arduino Firmware

In the folder Braccio_firmware is the firmware for the Arduino Uno to process the incoming messages. The protocol is based of the protocol of the SSC-32U. The messages for moving servos is almost the same of the SSC-32U but with less functionalities. The only functionalities handled now are the moving messages and stop messages.

## Moving the servos

An example of the protocol is listed below.

	#<servo>P<pwm>T<time in ms>\r

## Stopping servos

To stop the servos the stop must be called. An example is listed below

    STOP<nr>\r
    
## Other functionalities

There can also be a message sent that can check if the arm is connected. This command is listed below and it returns a message with 'CONNECTED\n';

    --connect\r

An other message is the protocol help message. This can help with the protocol and format. This command is listed below and it returns the help in the serial interface.

    --help
