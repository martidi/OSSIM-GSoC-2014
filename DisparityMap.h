//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: DisparityMap.h
//
// Author:  Martina Di Rita
//
// Description: Class provides Disparity Map extraction
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

class DisparityMap
{
public:
   DisparityMap();
   cv::Mat execute(cv::Mat master_mat, cv::Mat slave_mat);   

};



               
