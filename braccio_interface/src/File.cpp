#include "File.hpp"

File::File(){
}

File::~File(){
}

bool File::open(const std::string& fileName){
	this->fileName = fileName;
	file.open(fileName);
	if(file){
		return true;
	}
	std::cout<<"Cant open "<<fileName<<std::endl;
	return false;	
}

void File::extract(){
	std::string line;

	//Get all the lines of an file
	while(std::getline(this->file,line)){
		std::string tempStr;
		std::vector<std::string> tempVec;

		//Loop through a line of characters
		for(size_t i=0;i<line.length();++i){
			if(line[i]==COMMENT_CHAR){
				break;
			}
			if(line[i] == DELIMITER_CHAR){
				tempVec.push_back(tempStr);
				tempStr="";
			}else{
				tempStr+=line[i];
			}
		}
		//If the end is not empty
		if(tempStr!=""){
			tempVec.push_back(tempStr);
		}
		if(tempVec.size()>0){
			this->fileVector.push_back(tempVec);
		}
	}
}

bool File::findVariable(const std::string& variableName,std::vector<std::string>& returnArr){
	returnArr.clear();
	for(size_t i=0;i<this->fileVector.size();++i){
		if(this->fileVector.at(i).at(0).compare(variableName)==0){
			for(size_t j=1;j<this->fileVector.at(i).size();++j){
				returnArr.push_back(this->fileVector.at(i).at(j));
			}
			return true;
		}
	}
	throw std::invalid_argument("VariableName " + variableName + " is not in "+ fileName);
	return false;
}

bool File::findVariable(const std::string& variableName,std::vector<int32_t>& returnVector){
	returnVector.clear();
	std::vector<std::string> tempVec;
	bool returnValue = this->findVariable(variableName,tempVec);
	for(size_t i=0;i<tempVec.size();++i){
		returnVector.push_back((int32_t)std::stoi(tempVec.at(i)));
	}
	return returnValue;
}

bool File::variableExists(const std::string& variableName){
	for(size_t i=0;i<this->fileVector.size();++i){
		if(fileVector.at(i).at(0).compare(variableName)==0){
			return true;
		}
	}
	return false;
}

bool File::getVariable(const uint32_t variablePosition,std::vector<std::string>& returnVector){
	if(variablePosition>this->fileVector.size()){
		return false;
	}
	returnVector.clear();
	for(size_t i =0;i<this->fileVector.at(variablePosition).size();++i){
		returnVector.push_back(this->fileVector.at(variablePosition).at(i));
	}
	return true;
}

void File::printToConsole(){
	std::cout<<"---"<<fileName<<"---"<<std::endl;
	for(size_t i =0;i<this->fileVector.size();++i){
		for(size_t j=0;j<this->fileVector.at(i).size();++j){
			std::cout<<fileVector.at(i).at(j)<<" ";
		}
		std::cout<<std::endl;
	}
}
