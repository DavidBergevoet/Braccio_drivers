#include "Parser.hpp"

#define STOP_COMMAND "STOP"

bool Parser::singleCommand(const String& cmd, uint16_t ret[COMMAND_LENGTH])const
{
  for (size_t i = 0; i < cmd.length(); ++i)
  {
    switch (cmd[i])
    {
      case NR:
        {
          String nr;
          nr += cmd[i + 1];
          if (!is_number(nr))
          {
            return false;
          }
          ret[0] = nr.toInt();
          break;
        }
      case PWM:
        {
          String pwm;
          for (size_t j = i + 1; j < cmd.length(); ++j)
          {
            if (cmd[j] == TIME)
            {
              if (!is_number(pwm))
              {
                return false;
              }
              ret[1] = pwm.toInt();
              break;
            }
            else
            {
              pwm += cmd[j];
            }
          }
          break;
        }
      case TIME:
        {
          String t;
          for (size_t j = i + 1; j < cmd.length(); ++j)
          {
            if (cmd[j] == END)
            {
              if (!is_number(t))
              {
                return false;
              }
              ret[2] = t.toInt();
              break;
            }
            else
            {
              t += cmd[j];
            }
          }
          break;
        }
    }
  }
  return true;
}

bool Parser::stopCommand(const String& cmd, uint16_t& ret)const
{
  String stopCmd = STOP_COMMAND;
  for (size_t i = 0; i < stopCmd.length(); ++i)
  {
    if (stopCmd[i] != cmd[i])
    {
      return false;
    }
  }
  String servoNr = cmd.substring(stopCmd.length(), cmd.length());
  if(!is_number(servoNr)){
    return false;
  }
  ret = servoNr.toInt();
  return true;
}

bool Parser::is_number(const String& s)const
{
  for (char c : s)
  {
    if (!isdigit(c))
    {
      return false;
    }
  }
  return true;
}

void Parser::printHelp()const
{
  Serial.print("PROTOCOL HELP\n"
               "   " + String(NR) +  ": The number of the servo\n"
               "   " + String(PWM) +  ": The pwm signal of the servo\n"
               "   " + String(TIME) + ": The duration of the movement in ms\n"
               "   /r: The end character of the command\n"
               "   \n"
               "   "+String(STOP_COMMAND)+"<nr>: To stop a certain servo immediately\n");
}

void Parser::printConnect()const
{
  Serial.print("CONNECTED\n");
}
