#ifndef VTKtoPCL_H
#define VTKtoPCL_H

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
#include <vtkXMLPolyDataReader.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointColorPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointColorNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;
typedef vtkSmartPointer<vtkPoints>                      VTKPointsPtr;
typedef vtkSmartPointer<vtkPolyData>                    VTKPolyDataPtr;

//Template function declarations for inserting points of arbitrary dimension
template <typename PointT> 
void VTKtoPCL(vtkPolyData* polydata, typename pcl::PointCloud<PointT>::Ptr cloud)
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


//Specialization for points with RGB values
template <>
void VTKtoPCL<pcl::PointXYZRGB> (vtkPolyData* polydata, CloudPointColorPtr cloud)
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
void VTKtoPCL<pcl::PointXYZRGBNormal> (vtkPolyData* polydata, CloudPointColorNormalPtr cloud)
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
void VTKtoPCL<pcl::PointNormal> (vtkPolyData* polydata, CloudPointNormalPtr cloud)
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
