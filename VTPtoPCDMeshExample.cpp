// STL
#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

  // TODO: Not yet implemented !!!

  return EXIT_SUCCESS;
}
