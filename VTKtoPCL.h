#ifndef VTKtoPCL_H
#define VTKtoPCL_H

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointXYZRGBNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;

typedef pcl::PointCloud<pcl::PointXYZRGB>          CloudPointXYZRGBType;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>    CloudPointXYZRGBNormalType;
typedef pcl::PointCloud<pcl::PointNormal>          CloudPointNormalType;

typedef vtkSmartPointer<vtkPoints>                      VTKPointsPtr;
typedef vtkSmartPointer<vtkPolyData>                    VTKPolyDataPtr;

//Template function declarations for inserting points of arbitrary dimension
template <typename CloudT>
void VTKtoPCL(vtkPolyData* const polydata, CloudT* const cloud)
{
  // This generic template will convert any VTK PolyData
  // to a coordinate-only PointXYZ PCL format.

  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
    }
}

// Points and normals
template <> 
void VTKtoPCL<CloudPointNormalType> (vtkPolyData* const polydata, CloudPointNormalType* const cloud);

// Points and colors
template <>
void VTKtoPCL<CloudPointXYZRGBType> (vtkPolyData* const polydata, CloudPointXYZRGBType* const cloud);

// Points, normals, and colors
template <> 
void VTKtoPCL<CloudPointXYZRGBNormalType> (vtkPolyData* const polydata, CloudPointXYZRGBNormalType* const cloud);

template <typename CloudT>
void VTKtoPCL(vtkStructuredGrid* const structuredGrid, CloudT* const cloud)
{
  // This generic template will convert a vtkStructuredGrid
  // to a coordinate-only PointXYZ PCL format.

  int dimensions[3];
  structuredGrid->GetDimensions(dimensions);
  cloud->width = dimensions[0];
  cloud->height = dimensions[1]; // This indicates that the point cloud is organized
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->width; ++i)
  {
    for (size_t j = 0; j < cloud->height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
      double p[3];
      if(structuredGrid->IsPointVisible(pointId))
      {
        structuredGrid->GetPoint(pointId, p);
        (*cloud)(i, j).x = p[0];
        (*cloud)(i, j).y = p[1];
        (*cloud)(i, j).z = p[2];
      }
      else
      {
        (*cloud)(i, j).x = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).y = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).z = std::numeric_limits< float >::quiet_NaN();
      }
    }
  }
}

// Points, normals, and colors
template <>
void VTKtoPCL<CloudPointXYZRGBNormalType> (vtkStructuredGrid* const structuredGrid, CloudPointXYZRGBNormalType* const cloud);

#endif
