set(LIB_NAME ossimopencv_plugin)
MESSAGE( "************** LIBRARY SETUP FOR ossimopencv_plugin ******************")

# Include ossim cmake stuff:
include(OssimVersion)
include(OssimCommonVariables)
include(OssimUtilities)

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(requiredLib)

find_package( OpenCV )
if( OPENCV_FOUND )
	include_directories( ${OPENCV_INCLUDE_DIR} )
	set( requiredLib ${requiredLib} ${OPENCV_LIBRARIES} )
else( OPENCV_FOUND )
	message( FATAL_ERROR "Could not find OpenCV package!" )
endif( OPENCV_FOUND )

MESSAGE( STATUS "OPENCV_LIBRARY = ${OPENCV_LIBRARIES}" )
MESSAGE( STATUS "OPENCV_INCLUDE = ${OPENCV_INCLUDE_DIR}" )

FILE(GLOB_RECURSE OSSIMPLUGIN_SRCS *.cpp)
FILE(GLOB_RECURSE OSSIMPLUGIN_HEADERS *.h)

####
# OSSIMPLUGINSMAKINGDLL controls dll linkage on windows.  
# Adding this sets OSSIM_PLUGINS_DLL #define TO "__declspec(dllexport)".
###
add_definitions("-DOSSIMPLUGINSMAKINGDLL")

# Adjust the install library path:
if(OSSIM_INSTALL_PLUGINS_WITH_VERSION)
   set(INSTALL_LIBRARY_DIR   "${INSTALL_LIBRARY_DIR}/ossim-${OSSIM_VERSION}/plugins")
else(OSSIM_INSTALL_PLUGINS_WITH_VERSION)
   set(INSTALL_LIBRARY_DIR   "${INSTALL_LIBRARY_DIR}/ossim/plugins")
endif(OSSIM_INSTALL_PLUGINS_WITH_VERSION)

OSSIM_LINK_LIBRARY(${LIB_NAME}
                   COMPONENT_NAME ossim TYPE "${OSSIM_PLUGIN_LINK_TYPE}"
		   LIBRARIES ${OSSIM_LIBRARIES} ${requiredLib} 
		   HEADERS "${OSSIMPLUGIN_HEADERS}"
		   SOURCE_FILES "${OSSIMPLUGIN_SRCS}"
                   INSTALL_LIB)


# Add the apps dir:
add_subdirectory(apps)


# ---
# ossim-opencv app:
# ---

# Add the executable:
#add_executable(ossim-main-opencv ossim-main-opencv.cpp)

# Set the output dir:
#set_target_properties(ossim-main-opencv  PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# Link the app and ossim libraries:
#message( STATUS "Required libs = ${requiredLibs}" )
#target_link_libraries( ossim-main-opencv  ${requiredLib} )

# Install app:
#install(TARGETS ossim-main-opencv DESTINATION bin )


