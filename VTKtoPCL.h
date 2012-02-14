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
void VTKtoPCL(vtkPolyData* polydata, CloudT* cloud)
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

template <typename CloudT>
void VTKtoPCL(vtkStructuredGrid* structuredGrid, CloudT* cloud)
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


//Specialization for points with RGB values
template <>
void VTKtoPCL<CloudPointXYZRGBType> (vtkPolyData* polydata, CloudPointXYZRGBType* cloud)
{
  vtkUnsignedCharArray* colors = 
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup colors
    unsigned char color[3];
    colors->GetTupleValue(i,color);
    cloud->points[i].r = color[0];
    cloud->points[i].g = color[1];
    cloud->points[i].b = color[2];
    }
}

template <> 
void VTKtoPCL<CloudPointXYZRGBNormalType> (vtkPolyData* polydata, CloudPointXYZRGBNormalType* cloud)
{
  vtkFloatArray* normals = 
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  vtkUnsignedCharArray* colors = 
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup colors
    unsigned char color[3];
    colors->GetTupleValue(i,color);
    cloud->points[i].data_c[0] = color[0];
    cloud->points[i].data_c[1] = color[1];
    cloud->points[i].data_c[2] = color[2];
  
    // Setup normals
    float normal[3];
    normals->GetTupleValue(i,normal);
    cloud->points[i].normal_x = normal[0];
    cloud->points[i].normal_y = normal[1];
    cloud->points[i].normal_z = normal[2];
    }
}


template <> 
void VTKtoPCL<CloudPointNormalType> (vtkPolyData* polydata, CloudPointNormalType* cloud)
{
  vtkFloatArray* normals = vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
    
  cloud->width = polydata->GetNumberOfPoints();
  cloud->height = 1; // This indicates that the point cloud is unorganized
  cloud->is_dense = false;
  cloud->points.resize(cloud->width);

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    // Setup points
    double p[3];
    polydata->GetPoint(i,p);
    cloud->points[i].x = p[0];
    cloud->points[i].y = p[1];
    cloud->points[i].z = p[2];
  
    // Setup normals
    float normal[3];
    normals->GetTupleValue(i,normal);
    cloud->points[i].normal_x = normal[0];
    cloud->points[i].normal_y = normal[1];
    cloud->points[i].normal_z = normal[2];
    }
}

#endif
