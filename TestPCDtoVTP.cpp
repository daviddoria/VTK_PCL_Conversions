// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "PCLtoVTK.h"

static void XYZ();
static void XYZRGB();

int main (int argc, char*argv[])
{
  std::cout << "running XYZ()" << std::endl;
  XYZ();
  std::cout << "running XYZRGB()" << std::endl;
  XYZRGB();

  std::cout << "done." << std::endl;
  return EXIT_SUCCESS;
}

void XYZ()
{
  typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
  CloudType::Ptr cloud (new CloudType);

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  PCLtoVTK(*cloud, polydata);
}

void XYZRGB()
{
  typedef pcl::PointCloud<pcl::PointXYZRGB> CloudType;
  CloudType::Ptr cloud (new CloudType);

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  PCLtoVTK(*cloud, polydata);
}
