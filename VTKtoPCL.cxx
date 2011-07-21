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
  
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width = reader->GetOutput()->GetNumberOfPoints();
  cloud.height = 1; // This indicates that the point cloud is unorganized
  cloud.is_dense = false;
  cloud.points.resize (cloud.width);

  for (size_t i = 0; i < cloud.points.size (); ++i)
  {
    double p[3];
    reader->GetOutput()->GetPoint(i,p);
    cloud.points[i].x = p[0];
    cloud.points[i].y = p[1];
    cloud.points[i].z = p[2];
  }

  pcl::io::savePCDFileASCII (outputFileName.c_str(), cloud);

  return EXIT_SUCCESS;
}
