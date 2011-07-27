// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>

#include "PCLtoVTK.h"

int main (int argc, char*argv[])
{
  // Verify arguments
  if(argc < 3)
    {
    std::cerr << "Required arguments: input.pcd output.vtp" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  // Output arguments
  std::cout << "Reading " << inputFileName << " and writing " << outputFileName << std::endl;
  
  // Read the PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << pcl::getFieldsList<pcl::PointXYZ>(*cloud) << std::endl;

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (inputFileName.c_str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return EXIT_FAILURE;
  }

  // Create a polydata object and add the points to it.
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  PCLtoVTK<pcl::PointXYZ>(cloud, polydata);
 
  // Write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer =
    vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  writer->SetInputConnection(polydata->GetProducerPort());
  writer->Write();
  
  return EXIT_SUCCESS;
}
