// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

// VTK
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSmartPointer.h>

#include "VTKtoPCL.h"

int main (int argc, char** argv)
{
  // Verify arguments
  if(argc < 2)
    {
    std::cerr << "Required arguments: input.vtp output.pcd" << std::endl;
    return EXIT_FAILURE;
    }

  std::string input_file = argv[1];
  std::string output_file = argv[2];
  
  std::cout << "Reading " << input_file << " and writing " << output_file << std::endl;

  // Read the PCD file
  pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
  
  if (pcl::io::loadPCDFile<pcl::PolygonMesh> (input_file.c_str(), *mesh) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return EXIT_FAILURE;
  }
  
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  int result = PolygonMeshToPolyData(polygonMesh, polyData);
  
  // Read the VTP file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName(output_file.c_str());
  writer->SetInput(polyData);
  writer->Update();

  return EXIT_SUCCESS;
}
