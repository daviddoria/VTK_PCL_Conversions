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

  std::string inputVTP = argv[1];
  std::string outputPCD = argv[2];

  std::cout << "Reading " << inputVTP << " and writing " << outputPCD << ".pcd" << std::endl;

  // Read the VTP file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(inputVTP.c_str());
  reader->Update();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  VTKtoPCL(reader->GetOutput(), *cloud);

  pcl::io::savePCDFileASCII (outputPCD.c_str(), *cloud);

  return EXIT_SUCCESS;
}
