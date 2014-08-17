//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: openCVtestclass.cpp
//
// Author:  Martina Di Rita
//
// Description: Class provides OpenCV functions for DSM extraction
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimString.h>
#include <ossim/base/ossimNotify.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimIrect.h>
#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimConstants.h>
#include <ossim/elevation/ossimElevManager.h>
#include <ossim/imaging/ossimImageData.h>
#include <ossim/imaging/ossimImageSource.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include "openCVtestclass.h"
#include "TPgenerator.h"
#include "DisparityMap.h"

#include <opencv/highgui.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/flann/flann.hpp>
#include <opencv2/legacy/legacy.hpp>
// Note: These are purposely commented out to indicate non-use.
// #include <opencv2/nonfree/nonfree.hpp>
// #include <opencv2/nonfree/features2d.hpp>
// Note: These are purposely commented out to indicate non-use.
#include <vector>
#include <iostream>

openCVtestclass::openCVtestclass()
{
	
}

openCVtestclass::openCVtestclass(ossimRefPtr<ossimImageData> master, ossimRefPtr<ossimImageData> slave)
{
	// Create the OpenCV images
	master_mat.create(cv::Size(master->getWidth(), master->getHeight()), CV_16UC1);
    slave_mat.create(cv::Size(slave->getWidth(), slave->getHeight()), CV_16UC1);
	
	memcpy(master_mat.ptr(), (void*) master->getUshortBuf(), 2*master->getWidth()*master->getHeight());
	memcpy(slave_mat.ptr(), (void*) slave->getUshortBuf(), 2*slave->getWidth()*slave->getHeight());
	
	cout << "OSSIM->OpenCV image conversion done" << endl;
	
	// Rotation for along-track images
	cv::transpose(master_mat, master_mat);
	cv::flip(master_mat, master_mat, 1);
	
	cv::transpose(slave_mat, slave_mat);
	cv::flip(slave_mat, slave_mat, 1);	
}


bool openCVtestclass::execute()
{
	double minVal_master, maxVal_master, minVal_slave, maxVal_slave;
	cv::Mat master_mat_8U;
	cv::Mat slave_mat_8U;  
      
   	minMaxLoc( master_mat, &minVal_master, &maxVal_master );
   	minMaxLoc( slave_mat, &minVal_slave, &maxVal_slave );
	master_mat.convertTo( master_mat_8U, CV_8UC1, 255.0/(maxVal_master - minVal_master), -minVal_master*255.0/(maxVal_master - minVal_master));
	slave_mat.convertTo( slave_mat_8U, CV_8UC1, 255.0/(maxVal_slave - minVal_slave), -minVal_slave*255.0/(maxVal_slave - minVal_slave)); 
	
	TPgenerator* TPfinder = new TPgenerator(master_mat_8U, slave_mat_8U);
	TPfinder->run();
	
	cv::Mat slave_mat_warp = TPfinder->warp(slave_mat);
	
	//cv::Ptr<cv::CLAHE> filtro = cv::createCLAHE();
	//filtro->apply(master_mat_8U, master_mat_8U); 
	//filtro->apply(slave_mat_warp, slave_mat_warp);
    
	//cv::imwrite("Master_8bit_bSGM.tif",  master_mat_8U);
	//cv::imwrite("Slave_8bit_bSGM.tif",  slave_mat_warp);
    	
	DisparityMap* dense_matcher = new DisparityMap();
	
	//***
	// Abilitate for computing disparity on different scales 
	/*
	double fscale = 1.0/1.0;
	cv::resize(master_mat_8U, master_mat_8U, cv::Size(), fscale, fscale, cv::INTER_AREA );
	cv::resize(slave_mat_warp, slave_mat_warp, cv::Size(), fscale, fscale, cv::INTER_AREA );	
	cv::namedWindow( "Scaled master", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled master", master_mat_8U);
	cv::namedWindow( "Scaled slave", CV_WINDOW_NORMAL );
	cv::imshow( "Scaled slave", slave_mat_warp);
	*/
	//***
	
	out_disp = dense_matcher->execute(master_mat_8U, slave_mat_warp);
	
	return true;
}

bool openCVtestclass::computeDSM(double conv_factor, ossimElevManager* elev, ossimImageGeometry* master_geom)
{
	cv::transpose(out_disp, out_disp);
	cv::flip(out_disp, out_disp, 0);
    
	cv::Mat out_16bit_disp = cv::Mat::zeros (out_disp.size(),CV_64F);
	out_disp.convertTo(out_disp, CV_64F);
	out_16bit_disp = (out_disp/16.0) * conv_factor;
	
	cout<< "DSM GENERATION \t wait few minutes ..." << endl;

	for(int i=0; i< out_16bit_disp.rows; i++)
	{
		for(int j=0; j< out_16bit_disp.cols; j++)
		{
			ossimDpt image_pt(j,i);
			ossimGpt world_pt;     
			master_geom->localToWorld(image_pt, world_pt);
			ossim_float64 hgtAboveEllipsoid =  elev->getHeightAboveEllipsoid(world_pt);
			out_16bit_disp.at<double>(i,j) += hgtAboveEllipsoid;
		}
	}
 
	cv::Mat intDSM; 
	// Conversion from float to integer to write and show
	out_16bit_disp.convertTo(intDSM, CV_16U);
 
	cv::imwrite("Temp_DSM.tif", intDSM);
	
	double minVal, maxVal;
	minMaxLoc( intDSM, &minVal, &maxVal );
	intDSM.convertTo( intDSM, CV_8UC1, 255/(maxVal - minVal), -minVal*255/(maxVal - minVal));   
	
	cv::namedWindow("Temp_DSM", CV_WINDOW_NORMAL );
	cv::imshow("Temp_DSM", intDSM);
	cv::waitKey(0);	
	
	return true;
}

bool openCVtestclass::writeDisparity(double conv_factor)
{
	cv::transpose(out_disp, out_disp);
	cv::flip(out_disp, out_disp, 0);
    
	out_disp = (out_disp/16.0) * conv_factor;
	cv::imwrite("mDisparity.jpg", out_disp);
	
	return true;
}
