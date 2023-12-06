/*
 * pattern-parameters.hpp
 *
 *  Created on: Sep 1, 2020
 *      Author: atabb
 */

#ifndef PATTERN_PARAMETERS_HPP_
#define PATTERN_PARAMETERS_HPP_

#include "Includes.hpp"


enum patterns{charuco, arucop, april};

class patternParameters{
public:
    patterns type;
    int squaresX;
    int squaresY;
    int squareLength;
    int markerLength;
    int arc_code;
    string april_family;
    int numberBoards;
    float margins;
    float tagSpace;
    float squareLength_mm;
    float tagSpace_mm;
    AprilTagObject ATObject;


    patternParameters(){
        type = charuco;
        squaresX = 0;
        squaresY = 0;
        squareLength = 0;
        markerLength = 0;
        arc_code = 0;
        numberBoards = 0;
        margins = 0;
        tagSpace = 0;
        squareLength_mm = 0;
        tagSpace_mm = 0;

        april_family = "tagCodes36h11";
    }

    bool FigureOutAprilCase(); // returns true if the case is space == squarelength

    void Print();

};

bool readCharucoSpecificationFile(string filename, patternParameters &params);

bool readAprilTagSpecificationFile(string filename, patternParameters &params);

#endif /* PATTERN_PARAMETERS_HPP_ */
