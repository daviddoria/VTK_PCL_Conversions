// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr             CloudPointPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointColorPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointColorNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;
typedef vtkSmartPointer<vtkPoints>                      VTKPointsPtr;
typedef vtkSmartPointer<vtkPolyData>                    VTKPolyDataPtr;

#include "VTKtoPCL.h"
#include "PCLtoVTK.h"

int main (int argc, char** argv)
{
  CloudPointPtr cloudXYZ;
  CloudPointColorPtr cloudXYZRGB;
  CloudPointColorNormalPtr cloudXYZRGBNormal;
  CloudPointNormalPtr cloudXYZNormal;
  
  VTKPolyDataPtr polydata = VTKPolyDataPtr::New();
  
  PCLtoVTK(cloudXYZ.get(), polydata);
  PCLtoVTK(cloudXYZRGB.get(), polydata);
  PCLtoVTK(cloudXYZRGBNormal.get(), polydata);
  PCLtoVTK(cloudXYZNormal.get(), polydata);
  
  VTKtoPCL(polydata, cloudXYZRGB.get());
  VTKtoPCL(polydata, cloudXYZ.get());
  VTKtoPCL(polydata, cloudXYZRGBNormal.get());
  VTKtoPCL(polydata, cloudXYZNormal.get());
 
  return EXIT_SUCCESS;
}
