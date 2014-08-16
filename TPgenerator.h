//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: TPgenerator.h
//
// Author:  Martina Di Rita
//
// Description: Class provides a TPs generator
//
//----------------------------------------------------------------------------

#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include <ossim/base/ossimObject.h>
#include <ossim/base/ossimDpt.h>
#include <ossim/base/ossimString.h>
#include <ossim/base/ossimTieMeasurementGeneratorInterface.h>
#include "ossimIvtGeomXform.h"

#include <opencv/cv.h>

#include <ctime>
#include <vector>
#include <iostream>

class TPgenerator
{
public:
	TPgenerator();
	TPgenerator(cv::Mat master, cv::Mat slave); 
	void run();
	void TPgen();
	void TPdraw();
	cv::Mat estRT(std::vector<cv::Point2f> master, std::vector<cv::Point2f> slave);
	cv::Mat warp(cv::Mat slave_16bit);  
	 
	cv::Mat master_mat, slave_mat;
	cv::vector<cv::KeyPoint> keypoints1, keypoints2;
	vector<cv::DMatch > good_matches;
	double slave_x, slave_y, master_x, master_y;
};


