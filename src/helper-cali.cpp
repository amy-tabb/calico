/*
 * helper-cali.cpp
 *
 *  Created on: Nov 4, 2022
 *      Author: atabb
 */

#include "Includes.hpp"
#include "helper-cali.hpp"
#include "helper.hpp"

// used 11/24
bool readDetectorParameters(const string& filename, Ptr<aruco::DetectorParameters> &params) {

    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()){
        return false;
    }

    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}

//used 11/24
int ConvertAprilMarkerIdIndexToGridPointIndex(int squaresX, int markerID, int index_in_marker, int starting_marker_id_group){

     //array of object points of all the marker corners in the board each marker include its 4 corners in CCW order. For M markers, the size is Mx4.

    // number of grid points is squaresx *2, squares y *2.

    // two resolutions of grids -- markers, and then points.

    int adjusted_markerid = markerID - starting_marker_id_group;

    // grid ids within aruco marker groups.
    int gridx0 = adjusted_markerid % squaresX;
    int gridy0 = adjusted_markerid / squaresX;

    // then the next level grid -- depends which index it is.
    int gridx1 = 0;
    int gridy1 = 0;

    gridx1 = gridx0*2;
    gridy1 = gridy0*2;

    switch (index_in_marker){
    case 0:{
        gridx1 = gridx0*2;
        gridy1 = gridy0*2;
    } break;
    case 1:{
        gridx1 = gridx0*2 + 1;
        gridy1 = gridy0*2;
    } break;
    case 2: {
        gridx1 = gridx0*2 + 1;
        gridy1 = gridy0*2 + 1;
    } break;
    case 3: {
        gridx1 = gridx0*2;
        gridy1 = gridy0*2 + 1;
    }break;
    }

    int gridID = squaresX*2*gridy1 + gridx1;


    return gridID;

}




