#include "VTKtoPCL.h"

int PolyDataToPolygonMesh(vtkPolyData* polyData, pcl::PolygonMesh &polygonMesh)
{
  pcl::PointCloud < pcl::PointXYZ > cloud;
  cloud.points.resize (polyData->GetNumberOfPoints ());
  for (vtkIdType i = 0; i < polyData->GetNumberOfPoints (); ++i)
  {
    double p[3];
    polyData->GetPoint (i, p);
    cloud.points[i].x = p[0];
    cloud.points[i].y = p[1];
    cloud.points[i].z = p[2];
  }
  pcl::toROSMsg (cloud, polygonMesh.cloud);
  polygonMesh.polygons.resize (polyData->GetNumberOfCells ());

  vtkCellArray *vtk_newcells = polyData->GetPolys ();
  vtk_newcells->InitTraversal ();

  for (vtkIdType i = 0; i < vtk_newcells->GetNumberOfCells (); ++i)
  {
    pcl::Vertices v;
    vtkIdType num_points = 0;
    vtkIdType *points = 0;
    vtk_newcells->GetNextCell (num_points, points);
    v.vertices.resize (num_points);
    for (vtkIdType j = 0; j < num_points; ++j)
    {
      v.vertices[j] = points[j];
    }
    polygonMesh.polygons[i] = v;

  }
}
