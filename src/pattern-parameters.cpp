/*
 * pattern-parameters.cpp
 *
 *  Created on: Sep 1, 2020
 *      Author: atabb
 */


#include "pattern-parameters.hpp"

bool readAprilTagSpecificationFile(string filename, patternParameters &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["type"] >> params.type;
    fs["squaresX"] >> params.squaresX;
    fs["squaresY"] >> params.squaresY;
    fs["squareLength"] >> params.squareLength;
    fs["margins"] >> params.margins;
    fs["tagSpace"] >> params.tagSpace;
    fs["numberBoards"] >> params.numberBoards;
    fs["april_family"] >> params.april_family;
    return true;
}


bool readCharucoSpecificationFile(string filename, patternParameters &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
   // fs["type"] >> params.type;
    fs["squaresX"] >> params.squaresX;
    fs["squaresY"] >> params.squaresY;
    fs["squareLength"] >> params.squareLength;
    fs["markerLength"] >> params.markerLength;
    fs["numberBoards"] >> params.numberBoards;
    fs["arcCode"] >> params.arc_code;
    fs["margins"] >> params.margins;
    return true;
}


