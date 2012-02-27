// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLStructuredGridReader.h>

#include "VTKtoPCL.h"

int main (int argc, char** argv)
{
  // Verify arguments
  if(argc < 3)
    {
    std::cerr << "Required arguments: input.vts output.pcd" << std::endl;
    return EXIT_FAILURE;
    }

  std::string inputVTS = argv[1];
  std::string outputPCD = argv[2];

  std::cout << "Reading " << inputVTS << " and writing " << outputPCD << std::endl;

  // Read the VTP file
  vtkSmartPointer<vtkXMLStructuredGridReader> reader = vtkSmartPointer<vtkXMLStructuredGridReader>::New();
  reader->SetFileName(inputVTS.c_str());
  reader->Update();

  //typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  typedef pcl::PointCloud<pcl::PointXYZRGBNormal> CloudType;
  CloudType::Ptr cloud(new CloudType);

  VTKtoPCL(reader->GetOutput(), *cloud);

  pcl::io::savePCDFileASCII (outputPCD.c_str(), *cloud);

  return EXIT_SUCCESS;
}
