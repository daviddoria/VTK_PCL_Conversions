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
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "y_normal", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "z_normal", has_normal_z, normal_z_val));

  vtkFloatArray* normals =
    vtkFloatArray::SafeDownCast(polydata->GetPointData()->GetNormals());
  if(has_normal_x && has_normal_y && has_normal_z && normals)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      float normal[3];
      normals->GetTupleValue(i,normal);
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_x", normal[0]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_y", normal[1]));
      pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_z", normal[2]));
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
    vtkUnsignedCharArray::SafeDownCast(polydata->GetPointData()->GetScalars());
  if(has_r && has_g && has_b && colors)
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
void VTKtoPCL(vtkStructuredGrid* const structuredGrid, CloudT& cloud)
{
  int dimensions[3];
  structuredGrid->GetDimensions(dimensions);
  cloud.width = dimensions[0];
  cloud.height = dimensions[1]; // This indicates that the point cloud is organized
  cloud.is_dense = true;
  cloud.points.resize(cloud.width * cloud.height);

  typename CloudT::PointType testPoint = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;
  
  bool has_x = false; bool has_y = false; bool has_z = false;
  float x_val = 0.0f; float y_val = 0.0f; float z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "x", has_x, x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "y", has_y, y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "z", has_z, z_val));

  if(has_x && has_y && has_z)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        double coordinate[3];
        if(structuredGrid->IsPointVisible(pointId))
        {
          structuredGrid->GetPoint(pointId, coordinate);
          typename CloudT::PointType p = cloud(i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "x", coordinate[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "y", coordinate[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "z", coordinate[2]));
          cloud(i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Set the normals of the pcl::PointCloud (if the pcl::PointCloud supports normals and the input vtkStructuredGrid has normals)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "x_normal", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "y_normal", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint,
                                                                                            "z_normal", has_normal_z, normal_z_val));

  vtkFloatArray* normals = vtkFloatArray::SafeDownCast(structuredGrid->GetPointData()->GetNormals());

  if(has_x && has_y && has_z)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        float normal[3];
        if(structuredGrid->IsPointVisible(pointId))
        {
          normals->GetTupleValue(i,normal);
          typename CloudT::PointType p = cloud(i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_x", normal[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_y", normal[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "normal_z", normal[2]));
          cloud(i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }

  // Set the colors of the pcl::PointCloud (if the pcl::PointCloud supports colors and the input vtkStructuredGrid has colors)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0.0f; unsigned char g_val = 0.0f; unsigned char b_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint,
                                                                                            "b", has_b, b_val));
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast(structuredGrid->GetPointData()->GetArray("Colors"));

  if(has_r && has_g && has_b && colors)
  {
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        unsigned char color[3];
        if(structuredGrid->IsPointVisible(pointId))
        {
          colors->GetTupleValue(i,color);
          typename CloudT::PointType p = cloud(i, j);
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "r", color[0]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "g", color[1]));
          pcl::for_each_type<FieldList> (pcl::SetIfFieldExists<typename CloudT::PointType, float> (p, "b", color[2]));
          cloud(i, j) = p;
        }
        else
        {
          // Fill the point with an "empty" point?
        }
      }
    }
  }
}


#endif
