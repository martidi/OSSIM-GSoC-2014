OSSIM-GSoC-2014
===============

Photogrammetric image processing: DSM generation tool for OSSIM

===============

This is the repository for OSSIM GSoc 2014: it contains the code for the development of an OSSIM App for Digital Surface Models (DSMs) generation.

For more information about the project see http://www.google-melange.com/gsoc/proposal/public/google/gsoc2014/martidi/5629499534213120

This repository only contains the new and updated files, with reference to the structure of the OSSIM repository (http://trac.osgeo.org/ossim/browser/trunk/ossim).

In order to compile and install this OSSIM Plug-In use the following instructions:

	1. Install and compile the latest OSSIM version 
	2. Open a terminal window in the OSSIM_DEV_HOME/ossim_plugins/opencv/
	3. In this folder delete the old CMakeLists.txt file ($rm CMakeLists.txt)
	4. Use the following git commands to update the ossim opencv plugin
		$git init 
		$git remote add origin https://github.com/martidi/OSSIM-GSoC-2014.git
		$git pull origin master
	5. Re-compile the OSSIM version enabling the OPENCV plugin option
	
For any doubts or issues please email me: martina.dirita@uniroma1.it


