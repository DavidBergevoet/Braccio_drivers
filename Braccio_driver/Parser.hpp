#ifndef PARSER_HPP_
#define PARSER_HPP_

#include <Arduino.h>

#define NR '#'
#define END '\r'
#define PWM 'P'
#define TIME 'T'

#define COMMAND_LENGTH 3

#define HELP_COMMAND "--help"
#define CONNECT_COMMAND "--connect"

class Parser
{
  public:
    Parser() = default;
    virtual ~Parser() = default;

    bool singleCommand(const String& cmd, unsigned long ret[COMMAND_LENGTH]);
    void printHelp()const;
    void printConnect()const;
  private:
    bool is_number(const String& s)const;
};
#endif /* PARSER_HPP_ */
