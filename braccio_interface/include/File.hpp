#ifndef FILE_H_
#define FILE_H_

#define COMMENT_CHAR '#'
#define DELIMITER_CHAR ' '

#include <string>
#include <fstream>
#include <vector>
#include <iostream>

/**
 * @class File
 * @brief A file reading class which extracts the values and puts them into vectors. This data can be asked for
 */
class File
{
public:
  /**
   * @brief Default constructor
   */
  File();

  /**
   * @brief Default destructor
   */
  ~File();

  /**
   * @brief Open the found file
   * @param fileName the path to the file you want to open
   * @return True or false based on if the file is found
   */
  bool open(const std::string& fileName);

  /**
   * @brief Parses the found file into the fileVector
   */
  void extract();

  /**
   * @brief Finds a variable and put it into a string vector
   * @param variableName The name of the variable you're trying to find
   * @param returnVector vector in which the found information is stored
   * @return True if variable is found. False if not
   */
  bool findVariable(const std::string& variableName, std::vector<std::string>& returnVector);

  /**
   * @brief Finds a variable and put it into a int vector
   * @param variableName The name of the variable you're trying to find
   * @param returnVector The vector in which the found information is stored
   * @return True if variable is found. False if not
   */
  bool findVariable(const std::string& variableName, std::vector<int32_t>& returnVector);

  /**
   * @brief Find a variable with an index and put it into the returnVector
   * @param variablePosition The index of the variable
   * @param returnVector The vector in which the found information is stored
   * @return True or false base on if the variable is found
   */
  bool getVariable(const uint32_t variablePosistion, std::vector<std::string>& returnVector);

  /**
   * @brief returns if the variable exists within fileVector
   * @return True if the variable exists, false if not
   */
  bool variableExists(const std::string& variableName);

  /**
   * @brief Prints the file variables to the console
   */
  void printToConsole();

private:
  std::string fileName;                             /*!< The name of the configuration file */
  std::ifstream file;                               /*!< The file instance which reads the file */
  std::vector<std::vector<std::string>> fileVector; /*!< The member variable where all the file variables are stored */
};

#endif  // FILE_H_
