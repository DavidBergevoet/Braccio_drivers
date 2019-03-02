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

/**
 * @brief A class to parse incoming commands
 */
class Parser
{
  public:
    /**
     * @brief Default Contructor
     */
    Parser() = default;

    /**
     * @brief Default Destructor
     */
    virtual ~Parser() = default;

    /**
     * @brief Parses a command for a single servo.
     * @param cmd The command which needs to be parsed
     * @param ret The returnarray to store the retreived data from the command
     * @return True if the command has been parsed succesfully, false if not
     */
    bool singleCommand(const String& cmd, uint16_t ret[COMMAND_LENGTH])const;

    /**
     * @brief Parses a command to stop a servo
     * @param cmd The command which needs to be parsed
     * @param ret The return array to store the retreived data from the command
     * @return True if the command has been parsed correctly, false if not
     */
    bool stopCommand(const String& cmd,uint16_t& ret)const;

    /**
     * @brief This prints a help for the used protocol into the Serial
     */
    void printHelp()const;

    /**
     * @brief This will print a connect in the Serial
     */
    void printConnect()const;
  private:

    /**
     * @brief This will check whether or not the String is a number
     * @param s The string which needs to be checked
     * @return True if the string is a number, false if not
     */
    bool is_number(const String& s)const;
};
#endif /* PARSER_HPP_ */
