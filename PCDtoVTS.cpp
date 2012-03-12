// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>

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
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

  CloudType::Ptr cloud (new CloudType);

  if (pcl::io::loadPCDFile<CloudType::PointType> (inputFileName.c_str(), *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return EXIT_FAILURE;
  }

  // Create a polydata object and add the points to it.
  vtkSmartPointer<vtkStructuredGrid> structuredGrid = vtkSmartPointer<vtkStructuredGrid>::New();

  PCLtoVTK(*cloud, structuredGrid.GetPointer());
  
  std::cout << "Input cloud has " << cloud->width * cloud->height << " points." << std::endl;
  std::cout << "Output cloud has " << structuredGrid->GetNumberOfPoints() << " points." << std::endl;
 
  // Write the file
  vtkSmartPointer<vtkXMLStructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLStructuredGridWriter>::New();
  writer->SetFileName(outputFileName.c_str());
  //writer->SetInputConnection(structuredGrid->GetProducerPort());
  writer->SetInputData(structuredGrid);
  writer->Write();

  std::cout << "Finished." << std::endl;
  return EXIT_SUCCESS;
}
