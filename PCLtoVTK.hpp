#ifndef PCLtoVTK_HPP
#define PCLtoVTK_HPP

// PCL
#include <pcl/common/common.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

template <typename CloudT>
void PCLtoVTK(const CloudT& cloud, vtkPolyData* const pdata)
{
  typename CloudT::PointType testPoint = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;

  // Coordiantes (always must have coordinates)
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (size_t i = 0; i < cloud.points.size (); ++i)
    {
    double p[3];
    p[0] = cloud.points[i].x;
    p[1] = cloud.points[i].y;
    p[2] = cloud.points[i].z;
    points->InsertNextPoint (p);
    }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);

  // Normals (optional)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_y", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_z", has_normal_z, normal_z_val));
  if(has_normal_x && has_normal_y && has_normal_z)
  {
    typename CloudT::PointType testPoint = cloud.points[0];
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
    normals->SetNumberOfTuples(cloud.points.size());
    normals->SetName("Normals");

    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      typename CloudT::PointType p = cloud.points[i];
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "x_normal", has_normal_x, normal_x_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "y_normal", has_normal_y, normal_y_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "z_normal", has_normal_z, normal_z_val));
      float normal[3] = {normal_x_val, normal_y_val, normal_z_val};
      normals->SetTupleValue(i, normal);
      }
    tempPolyData->GetPointData()->SetNormals(normals);
  }

  // Colors (optional)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0; unsigned char g_val = 0; unsigned char b_val = 0;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "b", has_b, b_val));
  if(has_r && has_g && has_b)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(cloud.points.size());
    colors->SetName("RGB");

    for (size_t i = 0; i < cloud.points.size (); ++i)
      {
      typename CloudT::PointType p = cloud[i];
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", has_r, r_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", has_g, g_val));
      pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", has_b, b_val));
      unsigned char color[3] = {r_val, g_val, b_val};
      colors->SetTupleValue(i, color);
      }
    tempPolyData->GetPointData()->SetScalars(colors);
  }

  // Add 0D topology to every point
  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  //vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->SetInputData(tempPolyData);
  vertexGlyphFilter->Update();

  pdata->DeepCopy(vertexGlyphFilter->GetOutput());
}


template <typename CloudT>
void PCLtoVTK(const CloudT& cloud, vtkStructuredGrid* const structuredGrid)
{
  typename CloudT::PointType testPoint = cloud.points[0];

  typedef typename pcl::traits::fieldList<typename CloudT::PointType>::type FieldList;
  
  int dimensions[3] = {cloud.width, cloud.height, 1};
  structuredGrid->SetDimensions(dimensions);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(cloud.width * cloud.height);

  unsigned int numberOfInvalidPoints = 0;

  for (size_t i = 0; i < cloud.width; ++i)
  {
    for (size_t j = 0; j < cloud.height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
      typename CloudT::PointType point = cloud(i,j);

      if(pcl::isFinite(point))
      {
        float p[3] = {point.x, point.y, point.z};
        points->SetPoint(pointId, p);
      }
      else
      {

      }
    }
  }

  structuredGrid->SetPoints(points);

  // Normals (optional)
  bool has_normal_x = false; bool has_normal_y = false; bool has_normal_z = false;
  float normal_x_val = 0.0f; float normal_y_val = 0.0f; float normal_z_val = 0.0f;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_x", has_normal_x, normal_x_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_y", has_normal_y, normal_y_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (testPoint, "normal_z", has_normal_z, normal_z_val));
  if(has_normal_x && has_normal_y && has_normal_z)
  {
    vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
    normals->SetNumberOfComponents(3); // Note this must come before the SetNumberOfTuples calls
    normals->SetNumberOfTuples(cloud.width * cloud.height);
    normals->SetName("Normals");
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        typename CloudT::PointType point = cloud(i,j);

        if(pcl::isFinite(point))
        {
          typename CloudT::PointType p = cloud.points[i];
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "x_normal", has_normal_x, normal_x_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "y_normal", has_normal_y, normal_y_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, float> (p, "z_normal", has_normal_z, normal_z_val));
          float normal[3] = {normal_x_val, normal_y_val, normal_z_val};
          normals->SetTupleValue(pointId, normal);
        }
        else
        {

        }
      }
    }

    structuredGrid->GetPointData()->SetNormals(normals);
  }

  // Colors (optional)
  bool has_r = false; bool has_g = false; bool has_b = false;
  unsigned char r_val = 0; unsigned char g_val = 0; unsigned char b_val = 0;
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "r", has_r, r_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "g", has_g, g_val));
  pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (testPoint, "b", has_b, b_val));
  if(has_r && has_g && has_b)
  {
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3); // Note this must come before the SetNumberOfTuples calls
    colors->SetNumberOfTuples(cloud.width * cloud.height);
    colors->SetName("Colors");
    for (size_t i = 0; i < cloud.width; ++i)
    {
      for (size_t j = 0; j < cloud.height; ++j)
      {
        int queryPoint[3] = {i, j, 0};
        vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
        typename CloudT::PointType point = cloud(i,j);

        if(pcl::isFinite(point))
        {
          typename CloudT::PointType p = cloud[i];
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "r", has_r, r_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "g", has_g, g_val));
          pcl::for_each_type<FieldList> (pcl::CopyIfFieldExists<typename CloudT::PointType, unsigned char> (p, "b", has_b, b_val));
          unsigned char color[3] = {r_val, g_val, b_val};
          colors->SetTupleValue(i, color);
        }
        else
        {

        }
      }
    }
    structuredGrid->GetPointData()->AddArray(colors);
  }
  
  std::cout << "There were " << numberOfInvalidPoints << " invalid points." << std::endl;

}

#endif
