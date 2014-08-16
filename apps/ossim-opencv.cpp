//----------------------------------------------------------------------------
//
// License:  See top level LICENSE.txt file.
//
// File: ossim-opencv.cpp
//
// Author:  Martina Di Rita
//
// Description: This plugIn is able to extract a geocoded Digital Surface Model
//				using a stereopair.
//
//----------------------------------------------------------------------------

#include <ossim/base/ossimArgumentParser.h>
#include <ossim/base/ossimApplicationUsage.h>
#include <ossim/base/ossimConstants.h> 
#include <ossim/base/ossimException.h>
#include <ossim/base/ossimNotify.h>

#include <ossim/base/ossimRefPtr.h>
#include <ossim/base/ossimTimer.h>
#include <ossim/base/ossimTrace.h>
#include <ossim/base/ossimGpt.h>
#include <ossim/base/ossimDpt.h>

#include <ossim/init/ossimInit.h>

#include <ossim/util/ossimChipperUtil.h>

#include "ossim/imaging/ossimImageHandlerRegistry.h"
#include "ossim/imaging/ossimImageHandler.h"
#include "ossim/imaging/ossimImageGeometry.h"
#include "ossim/imaging/ossimImageFileWriter.h"
#include "ossim/imaging/ossimImageWriterFactoryRegistry.h"

#include <ossim/elevation/ossimElevManager.h>

#include "TPgenerator.h"
#include "openCVtestclass.h"
#include "DisparityMap.h"
#include "ossimTieMeasurementGenerator.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>

#include <iostream>
#include <sstream>
#include <cstdlib> /* for exit */
#include <iomanip>

#define C_TEXT( text ) ((char*)std::string( text ).c_str())

using namespace std;

bool ortho (ossimArgumentParser argPars)
{
	// Make the generator
	ossimRefPtr<ossimChipperUtil> chipper = new ossimChipperUtil;

	try
    {      
		bool continue_after_init = chipper->initialize(argPars);
		if (continue_after_init)
		{      
			// ossimChipperUtil::execute can throw an exception
			chipper->execute();
            
			ossimNotify(ossimNotifyLevel_NOTICE)
			<< "elapsed time in seconds: "
			<< std::setiosflags(ios::fixed)
			<< std::setprecision(3)
			<< ossimTimer::instance()->time_s() << endl;
		}
	}
	catch (const ossimException& e)
	{
		ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
		exit(1);
	}
	return true;
}

static ossimTrace traceDebug = ossimTrace("ossim-chipper:debug");

int main(int argc,  char* argv[])
{
	// Initialize ossim stuff, factories, plugin, etc.
	ossimTimer::instance()->setStartTick();
   	ossimArgumentParser ap(&argc, argv);
	ossimInit::instance()->initialize(ap);
	try
	{ 
        char* argv_master[10];
        char* argv_slave[10];
        
        cout << "MASTER DIRECTORY:" << " " << argv[1] << endl;
        cout << "SLAVE DIRECTORY:"  << " " << argv[2] << endl;

        // Making fake argv master & slave

		argv_master[0] = "ossim-chipper";
		argv_master[1] = "--op";
		argv_master[2] = "ortho";
		argv_master[3] = argv[1];
		argv_master[4] = argv[3];

		argv_slave[0] =  "ossim-chipper";
		argv_slave[1] =  "--op";
		argv_slave[2] =  "ortho";
		argv_slave[3] = argv[2];
		argv_slave[4] = argv[4];

		int originalArgCount = 5;
		int originalArgCount2 = 5;

		if(argc == 10) 
		{
			argv_master[5] = argv[5];
			argv_master[6] = argv[6];
			argv_master[7] = argv[7];
			argv_master[8] = argv[8];
			argv_master[9] = argv[9];

			argv_slave[5] = argv[5];
			argv_slave[6] = argv[6];
			argv_slave[7] = argv[7];
			argv_slave[8] = argv[8];
			argv_slave[9] = argv[9];

			originalArgCount = 10;
			originalArgCount2 = 10;

			cout << "TILE CUT:" << " " << "Lat_min" << " " << argv[6] 
        						<< " " << "Lon_min" << " " << argv[7]
        						<< " " << "Lat_max" << " " << argv[8]
        						<< " " << "Lon_max" << " " << argv[9] << endl;
		}	

		// Orthorectification
		cout << "Start master orthorectification" << endl;
		ossimArgumentParser ap_master(&originalArgCount, argv_master);
		ortho(ap_master); 
	
		cout << "Start slave orthorectification" << endl;
		ossimArgumentParser ap_slave(&originalArgCount2, argv_slave);
		ortho(ap_slave);
		
		//Elevation manager instance
        ossimElevManager* elev = ossimElevManager::instance();		
  
		// ImageHandlers & ImageGeometry instance
        ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[3]));             
        ossimImageHandler* slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[4]));
  
        ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[1]));
        ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[2]));
                      
    				
		if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave  
		{
			// Load ortho images
			ossimIrect bounds_master = master_handler->getBoundingRect(0); 			//gli sto dicendo di darmi i confini delle immagini
			ossimIrect bounds_slave = slave_handler->getBoundingRect(0);   
			ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);           //noti i confini delle immagini, sto facendo un tile di tutta l'immagine   
			ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0); 

			// TPs generation 
			openCVtestclass *test = new openCVtestclass(img_master, img_slave) ; 
   			test->execute();

			// Conversion factor computing
			ossimRefPtr<ossimImageGeometry> raw_master_geom = raw_master_handler->getImageGeometry();    
			ossimRefPtr<ossimImageGeometry> raw_slave_geom = raw_slave_handler->getImageGeometry(); 
			
			ossimGpt ul,ur,lr,ll;
			raw_master_geom->getCornerGpts(ul, ur, lr, ll);
					
			double Dlon = (ur.lon - ul.lon)/2.0;
			double Dlat = (ul.lat - ll.lat)/2.0;
        
			double conv_factor = 0.0;
        
			for (int i=0 ; i<3 ; i++) //LAT
			{
				for (int j=0 ; j<3 ; j++) //LON
				{
					ossimGpt punto_terra(ur.lat-i*Dlat,ul.lon+j*Dlon,200.00);
					ossimGpt punto_terra_up(ur.lat-i*Dlat,ul.lon+j*Dlon,300.00);	
					ossimDpt punto_img(0.,0.);
					ossimDpt punto_img_up(0.,0.);
				
					raw_master_geom->worldToLocal(punto_terra,punto_img);              
					raw_master_geom->worldToLocal(punto_terra_up,punto_img_up);   
					
					double DeltaI_Master = punto_img_up.x - punto_img.x;
					double DeltaJ_Master = punto_img_up.y - punto_img.y;
        
					raw_slave_geom->worldToLocal(punto_terra,punto_img);       
					raw_slave_geom->worldToLocal(punto_terra_up,punto_img_up);    
					
					double DeltaI_Slave = punto_img_up.x - punto_img.x;
					double DeltaJ_Slave = punto_img_up.y - punto_img.y;
					
					cout << DeltaI_Master << "\t"<< DeltaJ_Master <<"\t" <<  DeltaI_Slave << "\t" << DeltaJ_Slave << "\t" 
					     << DeltaI_Master-DeltaI_Slave << "\t" << DeltaJ_Master - DeltaJ_Slave  <<endl;
										
					conv_factor += DeltaJ_Slave - DeltaJ_Master;
				}			
			}	        
        
			conv_factor = conv_factor/(9.0*100.0);
			cout << "Conversion factor \t"<< conv_factor << endl;
			
			// From Disparity to DSM
			ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();			
			test->computeDSM(conv_factor, elev, master_geom);
			
			// Geocoded DSM generation
			ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("DSM.tif"));
            handler_disp->setImageGeometry(master_geom);       
            ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimFilename("Geocoded_DSM.tif"));
            writer->connectMyInputTo(0, handler_disp);
            writer->execute();
            
			delete writer;
			delete test;				
		}
	}   
		   
	catch (const ossimException& e)
	{
     	 ossimNotify(ossimNotifyLevel_WARN) << e.what() << endl;
      	return 1;
	}
  
	return 0;
}

/*./bin/ossim-opencv ../../../../img_data/po_3800808_0000000/po_3800808_pan_0000000.tif ../../../../img_data/po_3800808_0010000/po_3800808_pan_0010000.tif ../../../../img_data/risultati_epipolar/ortho_ritaglio1.jpg ../../../../img_data/risultati_epipolar/ortho_ritaglio2.jpg --cut-bbox-ll 44.603 11.816 44.623 11.851 */
/*./bin/ossim-opencv ../../../../Prove_OpenCV/TEST_SGM/Argenta/Argenta_Left.tif ../../../../Prove_OpenCV/TEST_SGM/Argenta/Argenta_Right.tif ../../../../Prove_OpenCV/TEST_SGM/risultati/Argenta_Right_ortho.jpg ../../../../Prove_OpenCV/TEST_SGM/risultati/Argenta_Right_ortho.jpg --cut-bbox-ll 44.603 11.816 44.623 11.851*/
