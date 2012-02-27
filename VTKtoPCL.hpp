#ifndef VTKtoPCL_HPP
#define VTKtoPCL_HPP

#include "VTKtoPCL.h" // Appease syntax parser

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

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointXYZRGBNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;

typedef pcl::PointCloud<pcl::PointXYZRGB>          CloudPointXYZRGBType;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>    CloudPointXYZRGBNormalType;
typedef pcl::PointCloud<pcl::PointNormal>          CloudPointNormalType;

typedef vtkSmartPointer<vtkPoints>                      VTKPointsPtr;
typedef vtkSmartPointer<vtkPolyData>                    VTKPolyDataPtr;

template <typename CloudT>
void VTKtoPCL(vtkPolyData* const polydata, CloudT& cloud)
{
  // This generic template will convert any VTK PolyData
  // to a coordinate-only PointXYZ PCL format.

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

  cloud.width = polydata->GetNumberOfPoints();
  cloud.height = 1; // This indicates that the point cloud is unorganized
  cloud.is_dense = false;
  cloud.points.resize(cloud.width);

  typename CloudT::PointType testPoint = cloud.points[0];

  bool has_x = false; bool has_y = false; bool has_z = false;
  float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

  // Set the coordinates of the pcl::PointCloud (if the pcl::PointCloud supports coordinates)
  if(has_x && has_y && has_z)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      double coordinate[3];
      polydata->GetPoint(i,coordinate);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", coordinate[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", coordinate[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", coordinate[2]));
      cloud.points[i] = p;
      }
  }

  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkPolyData has normals)
  bool has_x_normal = false; bool has_y_normal = false; bool has_z_normal = false;
  float x_normal_val = 0.0f; float y_normal_val = 0.0f; float z_normal_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "x_normal", has_x_normal, x_normal_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "y_normal", has_y_normal, y_normal_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "z_normal", has_z_normal, z_normal_val));

  vtkFloatArray* normals =
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
  if(has_x && has_y && has_z && normals)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      float normal[3];
      normals->GetTupleValue(i,normal);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x_normal", normal[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y_normal", normal[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z_normal", normal[2]));
      cloud.points[i] = p;
      }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkPolyData has colors)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0.0f; unsigned char g_val = 0.0f; unsigned char b_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "b", has_b, b_val));

  vtkUnsignedCharArray* colors =
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetNormals());
  if(has_x && has_y && has_z && colors)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      unsigned char color[3];
      colors->GetTupleValue(i,color);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", color[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", color[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", color[2]));
      cloud.points[i] = p;
      }
  }

}

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
