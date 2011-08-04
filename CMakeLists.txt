cmake_minimum_required(VERSION 2.6)

# Specify where to copy executables when 'make install' is run
SET( INSTALL_DIR ${CMAKE_INSTALL_PREFIX} )

PROJECT(VTKPCLConversions)

######### Libraries #########
# VTK
FIND_PACKAGE(VTK REQUIRED)

# PCL
FIND_PACKAGE(PCL 1.1 REQUIRED)
INCLUDE(${VTK_USE_FILE})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})

######### Examples #########
ADD_EXECUTABLE(VTPtoPCDPointCloudExample VTPtoPCDPointCloudExample.cpp)
TARGET_LINK_LIBRARIES(VTPtoPCDPointCloudExample vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(VTPtoPCDPointCloudExample PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS VTPtoPCDPointCloudExample RUNTIME DESTINATION ${INSTALL_DIR} )

ADD_EXECUTABLE(VTPtoPCDMeshExample VTPtoPCDMeshExample.cpp)
TARGET_LINK_LIBRARIES(VTPtoPCDMeshExample vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(VTPtoPCDMeshExample PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS VTPtoPCDMeshExample RUNTIME DESTINATION ${INSTALL_DIR} )


ADD_EXECUTABLE(PCDtoVTPPointCloudExample PCDtoVTPPointCloudExample.cpp PCLtoVTK.cpp)
TARGET_LINK_LIBRARIES(PCDtoVTPPointCloudExample vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(PCDtoVTPPointCloudExample PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS PCDtoVTPPointCloudExample RUNTIME DESTINATION ${INSTALL_DIR} )


ADD_EXECUTABLE(PCDtoVTPMeshExample PCDtoVTPMeshExample.cpp PCLtoVTK.cpp)
TARGET_LINK_LIBRARIES(PCDtoVTPMeshExample vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(PCDtoVTPMeshExample PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS PCDtoVTPMeshExample RUNTIME DESTINATION ${INSTALL_DIR} )

######### Batch Conversions #########
ADD_EXECUTABLE(BatchConvertPCDtoVTP BatchConvertPCDtoVTP.cpp)
TARGET_LINK_LIBRARIES(BatchConvertPCDtoVTP vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(BatchConvertPCDtoVTP PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS BatchConvertPCDtoVTP RUNTIME DESTINATION ${INSTALL_DIR} )

ADD_EXECUTABLE(BatchConvertVTPtoPCD BatchConvertVTPtoPCD.cpp)
TARGET_LINK_LIBRARIES(BatchConvertVTPtoPCD vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
set_target_properties(BatchConvertVTPtoPCD PROPERTIES COMPILE_DEFINITIONS "${DEFINITIONS}")
INSTALL( TARGETS BatchConvertVTPtoPCD RUNTIME DESTINATION ${INSTALL_DIR} )

######### Instantiations #########
# Instantiate all of the possible template functions even if they are not used in real examples
# to make sure there are no compiler errors.
ADD_EXECUTABLE(TestInstantiate TestInstantiate.cxx)
TARGET_LINK_LIBRARIES(TestInstantiate vtkHybrid ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
