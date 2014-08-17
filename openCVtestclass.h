//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.h
//
// Author:  Martina Di Rita
//
// Description: Class provides OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimObject.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimTieMeasurementGeneratorInterface.h>
#include "ossimIvtGeomXform.h"

#include <opencv/cv.h>

#include <ctime>
#include <vector>
#include <iostream>

class openCVtestclass
{
public:
	openCVtestclass();
	~openCVtestclass(){};
	openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave); 
	bool execute();
	bool writeDisparity(double conv_factor);
	bool computeDSM(double conv_factor, ossimElevManager* elev, ossimImageGeometry* master_geom);
   
	cv::Mat master_mat, slave_mat;
	cv::vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<cv::DMatch > good_matches;
	cv::Mat out_disp; 
};


             
