#ifndef FILE_H_
#define FILE_H_

#define COMMENT_CHAR '#'
#define DELIMITER_CHAR ' '

#include <string>
#include <fstream>
#include <vector>
#include <iostream>


class File{
	public:
		//Constructor and destructor
		File();
		~File();
		
		/**
		 * Description: Open the found file
		 * Parameters
		 * fileName: the path to the file you want to open
		 * Return: true or false based on if the file is found
		 */
		bool open(const std::string& fileName);
		
		/**
		 * Description: Parses the found file into the fileVector
		 */
		void extract();
		
		/**
		 * Description: Finds a variable and put it into a string vector
		 * Parameters
		 * variableName = name of the thing you're trying to find
		 * returnVector = vector in which the found information is put
		 * Return: a true if variable is found. False if not
		 */
		bool findVariable(const std::string& variableName,std::vector<std::string>& returnVector);
		
		/**
		 * Description: Finds a variable and put it into a int vector
		 * Parameters
		 * variableName = name of the thing you're trying to find
		 * returnVector = vector in which the found information is put
		 * Return: a true if variable is found. False if not
		 */
		bool findVariable(const std::string& variableName,std::vector<int32_t>& returnVector);
		
		/** 
		 * Description: Find a variable with an index and put it into the returnVector
		 * Parameters
		 * variablePosition: The index of the variable
		 * returnVector: the vector in which the found information is put
		 * Return: returns true or false if the variable is found
		 */
		bool getVariable(const uint32_t variablePosistion,std::vector<std::string>& returnVector);
		
		/**
		 * Description: returns if the variable exists within fileVector
		 * return: returns true or false if the variable exists
		 */
		bool variableExists(const std::string& variableName);

		void printToConsole();		
	private:
		std::string fileName;
		std::ifstream file;
		std::vector<std::vector<std::string>> fileVector;
};

#endif //FILE_H_
