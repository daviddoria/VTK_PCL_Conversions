#include "VTKtoPCL.h"

template <>
void VTKtoPCL<CloudPointXYZRGBNormalType> (vtkStructuredGrid* const structuredGrid, CloudPointXYZRGBNormalType* const cloud)
{
  int dimensions[3];
  structuredGrid->GetDimensions(dimensions);
  cloud->width = dimensions[0];
  cloud->height = dimensions[1]; // This indicates that the point cloud is organized
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);

  vtkFloatArray* normals = vtkFloatArray::SafeDownCast(structuredGrid->GetPointData()->GetNormals());
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast(structuredGrid->GetPointData()->GetArray("Colors"));
  
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
        
        if(normals)
        {
          float n[3];
          normals->GetTupleValue(pointId, n);
          (*cloud)(i, j).normal_x = n[0];
          (*cloud)(i, j).normal_y = n[1];
          (*cloud)(i, j).normal_z = n[2];
        }
        
        if(colors)
        {
          unsigned char c[3];
          colors->GetTupleValue(pointId, c);
          (*cloud)(i, j).r = c[0];
          (*cloud)(i, j).g = c[1];
          (*cloud)(i, j).b = c[2];
        }
      }
      else
      {
        (*cloud)(i, j).x = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).y = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).z = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).r = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).g = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).b = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).normal_x = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).normal_y = std::numeric_limits< float >::quiet_NaN();
        (*cloud)(i, j).normal_z = std::numeric_limits< float >::quiet_NaN();
      }
    }
  }
}
