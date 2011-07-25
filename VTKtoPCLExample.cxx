// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSmartPointer.h>

#include "VTKtoPCL.h"

int main (int argc, char** argv)
{
  // Verify arguments
  if(argc < 3)
    {
    std::cerr << "Required arguments: input.vtp output.pcd" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  // Output arguments
  std::cout << "Reading " << inputFileName << " and writing " << outputFileName << std::endl;
  
  // Read the VTP file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputFileName.c_str());
  reader->Update();
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  VTKtoPCL<pcl::PointXYZ>(reader->GetOutput(), cloud);
 
  pcl::io::savePCDFileASCII (outputFileName.c_str(), *cloud);

  return EXIT_SUCCESS;
}
