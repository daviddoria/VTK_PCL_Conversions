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
  if(argc < 2)
    {
    std::cerr << "Required arguments: input.vtp [input2.vtp input3.vtp ... inputN.vtp]" << std::endl;
    return EXIT_FAILURE;
    }

  std::vector<std::string> input_files;
  
  // Parse and output arguments
  for (int i = 1; i < argc; ++i) {
    std::cout << "Reading " << argv[i] << " and writing " << argv[i] << ".pcd" << std::endl;
  
    // Read the VTP file
    vtkSmartPointer<vtkXMLPolyDataReader> reader = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    reader->SetFileName(argv[i]);
    reader->Update();
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    VTKtoPCL(reader->GetOutput(), *cloud);

    std::string outputFileName = argv[i];
    outputFileName += ".pcd";
 
    pcl::io::savePCDFileASCII (outputFileName.c_str(), *cloud);
  }
  return EXIT_SUCCESS;
}
