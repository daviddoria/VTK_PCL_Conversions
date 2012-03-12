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
  if(argc < 2)
    {
    std::cerr << "Required arguments: input.pcd [input2.pcd input3.pcd ... inputN.pcd]" << std::endl;
    return EXIT_FAILURE;
    }
 
  std::vector<std::string> input_files;

  // Parse and output arguments
  for (int i = 1; i < argc; ++i) 
  {
    std::cout << "Reading " << argv[i] << " and writing " << argv[i] << ".vtp" << std::endl;
  
    // Read the PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::cout << pcl::getFieldsList<pcl::PointXYZ>(*cloud) << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ> (argv[i], *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file \n");
      return EXIT_FAILURE;
    }

    // Create a polydata object and add the points to it.
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

    PCLtoVTK(*cloud, polydata);
 
    // Write the file
    vtkSmartPointer<vtkXMLPolyDataWriter> writer =
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();

    std::string outputFileName = argv[i];
    outputFileName += ".vtp";

    writer->SetFileName(outputFileName.c_str());
    //writer->SetInputConnection(polydata->GetProducerPort());
    writer->SetInputData(polydata);
    writer->Write();
  }
  
  return EXIT_SUCCESS;
}
