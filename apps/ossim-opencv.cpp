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
			<< ossimTimer::instance()->time_s() << endl << endl;
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
        char* argv_master[12];
        char* argv_slave[12];
        
        cout << endl << "MASTER DIRECTORY:" << " " << argv[1] << endl;
        cout << "SLAVE DIRECTORY:"  << " " << argv[2] << endl << endl;

        // Making fake argv master & slave	
	
/*	
   string tempString;
   ossimArgumentParser::ossimParameter stringParam(tempString);
   ossimArgumentParser argumentParser(&argc, argv);
   ossimInit::instance()->addOptions(argumentParser);
   ossimInit::instance()->initialize(argumentParser);

   if(traceDebug())
   {
      ossimNotify(ossimNotifyLevel_DEBUG) << "entered main" << std::endl;
   }
   
   argumentParser.getApplicationUsage()->setApplicationName(argumentParser.getApplicationName());
   argumentParser.getApplicationUsage()->setDescription(argumentParser.getApplicationName()+" takes a spec file as input and produces a product");
   argumentParser.getApplicationUsage()->setCommandLineUsage(argumentParser.getApplicationName()+" [options] <spec_file>");
   argumentParser.getApplicationUsage()->addCommandLineOption("-t or --thumbnail", "thumbnail resolution");
   argumentParser.getApplicationUsage()->addCommandLineOption("-h or --help","Display this information");
 

   if(argumentParser.read("-h") ||
      argumentParser.read("--help")||
      argumentParser.argc() <2)
   {
      argumentParser.getApplicationUsage()->write(std::cout);
      exit(0);
   }

   ossimRefPtr<ossimIgen> igen = new ossimIgen;
   double start=0, stop=0;
   
   ossimMpi::instance()->initialize(&argc, &argv);
   start = ossimMpi::instance()->getTime();

   ossimKeywordlist kwl;
   kwl.setExpandEnvVarsFlag(true);
   
   while(argumentParser.read("-t", stringParam)   ||
         argumentParser.read("--thumbnail", stringParam));
   
   if(ossimMpi::instance()->getRank() > 0)
   {
      // since this is not the master process
      // then it will set the keyword list form the master
      // so set this to empty
      //
      igen->initialize(ossimKeywordlist());
   }
   else if(argumentParser.argc() > 1)
   {
      if(kwl.addFile(argumentParser.argv()[1]))
      {
         if(tempString != "")
         {
            kwl.add("igen.thumbnail",
                    "true",
                    true);
            kwl.add("igen.thumbnail_res",
                    tempString.c_str(),
                    true);
         }
         else
         {
            kwl.add("igen.thumbnail",
                    "false",
                    true);
         }
         kwl.add("igen.thumbnail_res",
                 tempString.c_str(),
                 true);

         igen->initialize(kwl);
      }
   }

*/	
	
		if( argc < 6)
		{
			cout << "Usage: ossim-opencv <input_left_image> <input_right_image> <output_ortho_left_image> <output_ortho_right_image> <output_DSM> [options]" << endl;
			cout << "Options:" << endl;
			cout << "--cut-bbox-ll <min_lat> <min_lon> <max_lat> <max_lon> \t Specify a bounding box with the minimum"   << endl;   
			cout << "\t\t\t\t\t\t\tlatitude/longitude and max latitude/longitude" << endl; 
			cout << "\t\t\t\t\t\t\tin decimal degrees." << endl; 
			return -1;
		}
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

		if(argc == 13) 
		{
			argv_master[5] = argv[5];			
			argv_master[6] = argv[6];
			argv_master[7] = argv[7];
			argv_master[8] = argv[8];
			argv_master[9] = argv[9];
			argv_master[10] = argv[10];
			argv_master[11] = argv[11];
			argv_master[12] = argv[12];

			argv_slave[5] = argv[5];
			argv_slave[6] = argv[6];
			argv_slave[7] = argv[7];
			argv_slave[8] = argv[8];
			argv_slave[9] = argv[9];
			argv_slave[10] = argv[10];
			argv_slave[11] = argv[11];	
			argv_slave[12] = argv[12];									

			originalArgCount = 13;
			originalArgCount2 = 13;

			cout << "TILE CUT:" << " " << "Lat_min" << " " << argv[7] 
        						<< " " << "Lon_min" << " " << argv[8]
        						<< " " << "Lat_max" << " " << argv[9]
        						<< " " << "Lon_max" << " " << argv[10] << endl << endl;
		}	

		// Orthorectification
		cout << "Start master orthorectification" << endl;
		ossimArgumentParser ap_master(&originalArgCount, argv_master);
		ortho(ap_master); 
	
		cout << "Start slave orthorectification" << endl;
		ossimArgumentParser ap_slave(&originalArgCount2, argv_slave);
		ortho(ap_slave);
		
		// Elevation manager instance
		ossimElevManager* elev = ossimElevManager::instance();		
  
		// ImageHandlers & ImageGeometry instance
		ossimImageHandler* master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[3]));             
		ossimImageHandler* slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[4]));
  
		ossimImageHandler* raw_master_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[1]));
		ossimImageHandler* raw_slave_handler = ossimImageHandlerRegistry::instance()->open(ossimFilename(argv[2]));
                      
    				
		if(master_handler && slave_handler && raw_master_handler && raw_slave_handler) // enter if exist both master and slave  
		{
			// Load ortho images
			ossimIrect bounds_master = master_handler->getBoundingRect(0); 			
			ossimIrect bounds_slave = slave_handler->getBoundingRect(0);   
			ossimRefPtr<ossimImageData> img_master = master_handler->getTile(bounds_master, 0);          
			ossimRefPtr<ossimImageData> img_slave = slave_handler->getTile(bounds_slave, 0); 

			// TPs generation 
			openCVtestclass *test = new openCVtestclass(img_master, img_slave) ; 					
   			test->execute();

			// Conversion factor (from pixels to meters) computation
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
					ossimGpt punto_terra_up(ur.lat-i*Dlat,ul.lon+j*Dlon,2200.00);	
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
					
					//cout << DeltaI_Master << "\t"<< DeltaJ_Master <<"\t" <<  DeltaI_Slave << "\t" << DeltaJ_Slave << "\t" 
							//<< DeltaI_Master-DeltaI_Slave << "\t" << DeltaJ_Master - DeltaJ_Slave  <<endl;
										
					conv_factor += DeltaJ_Slave - DeltaJ_Master;
				}			
			}	        
        
			conv_factor = conv_factor/(9.0*2000.0);
			cout << "Conversion factor \t"<< conv_factor << endl << endl;
			
			// From Disparity to DSM
			ossimImageGeometry* master_geom = master_handler->getImageGeometry().get();			
			test->computeDSM(conv_factor, elev, master_geom);
			
			// Geocoded DSM generation
			ossimImageHandler *handler_disp = ossimImageHandlerRegistry::instance()->open(ossimFilename("Temp_DSM.tif"));
			handler_disp->setImageGeometry(master_geom);       
			ossimImageFileWriter* writer = ossimImageWriterFactoryRegistry::instance()->createWriter(ossimFilename(argv[5]));
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
