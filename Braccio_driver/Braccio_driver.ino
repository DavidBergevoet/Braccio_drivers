//#define DEBUG_MODE

#include "BraccioArm.hpp"
#include "Parser.hpp"

#define BAUDRATE 9600

BraccioArm arm;
//0: ServoNr  1: PWM  2:  Milliseconds
#define SERVONR 0
#define PWMNR 1
#define MSNR 2
unsigned long arr[COMMAND_LENGTH];
Parser p;

void setup() {
  Serial.begin(BAUDRATE);
  for (size_t i = 0; i < COMMAND_LENGTH; ++i) {
    arr[i] = 0;
  }
  arm.init();
  debugln(F("Started"));
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil(END);
    if (command.equals(HELP_COMMAND))
    {
      p.printHelp();
    }
    else if (command.equals(CONNECT_COMMAND))
    {
      p.printConnect();
    }
    else if (p.singleCommand(command + END, arr))
    {
      arm.setTarget(braccio::Servos(arr[SERVONR]), arr[PWMNR], arr[MSNR]);
    }
  }
  arm.update();
}
