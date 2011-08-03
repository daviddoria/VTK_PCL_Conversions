#include "PCLtoVTK.h"

#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

int PolygonMeshToPolyData(const pcl::PolygonMesh& polygonMesh, vtkPolyData* polyData)
{
  if (polygonMesh.cloud.data.empty ())
  {
    PCL_ERROR ("[pcl::surface::convertToVTK] Input point cloud has no data!\n");
    return (-1);
  }
  vtkSmartPointer<vtkPoints> vtk_pts = vtkSmartPointer<vtkPoints>::New ();

  int nr_points  = polygonMesh.cloud.width * polygonMesh.cloud.height;
  int point_size = polygonMesh.cloud.data.size () / nr_points;
  for (int i = 0; i < nr_points; ++i)
  {
    int xyz = 0;
    float value[3];
    for (size_t d = 0; d < polygonMesh.cloud.fields.size (); ++d)
    {
      int count = polygonMesh.cloud.fields[d].count;
      if (count == 0)
        count = 1;
      int c = 0;
      if ((polygonMesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
          polygonMesh.cloud.fields[d].name == "x" ||
          polygonMesh.cloud.fields[d].name == "y" ||
          polygonMesh.cloud.fields[d].name == "z"))
      {
        memcpy (&value[xyz], &polygonMesh.cloud.data[i * point_size + polygonMesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
        if (++xyz == 3)
          break;
      }
    }
    if (xyz != 3)
    {
      PCL_ERROR ("[pcl::io::saveVTKFile] Input point cloud has no XYZ data!\n");
      return (-2);
    }
    vtk_pts->InsertPoint(i,value);
  }

  vtkSmartPointer<vtkCellArray> vtk_cells = vtkSmartPointer<vtkCellArray>::New ();
  for (size_t i = 0; i < polygonMesh.polygons.size (); ++i)
  {
    vtk_cells->InsertNextCell (polygonMesh.polygons[i].vertices.size ());
    size_t j = 0;
    for (j = 0; j < polygonMesh.polygons[i].vertices.size (); ++j)
      vtk_cells->InsertCellPoint (polygonMesh.polygons[i].vertices[j]);
  }

  polyData->SetPoints(vtk_pts);
  polyData->SetPolys(vtk_cells);
  polyData->Update();

  vtkSmartPointer<vtkTriangleFilter> vtk_triangles = vtkSmartPointer<vtkTriangleFilter>::New ();
  vtk_triangles->SetInput (polyData);
  vtk_triangles->Update();

  polyData = vtk_triangles->GetOutput ();

  return 1;
}


//Specialization for points with RGB values
template <>
void PCLtoVTK<pcl::PointXYZRGB> (CloudPointColorPtr cloud, VTKPolyDataPtr pdata)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetNumberOfTuples(cloud->points.size());
  colors->SetName("RGB");

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    points->InsertNextPoint ( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    unsigned char color[3] = {static_cast<unsigned char>(cloud->points[i].r),
                              static_cast<unsigned char>(cloud->points[i].g),
                              static_cast<unsigned char>(cloud->points[i].b)};
    colors->SetTupleValue(i, color);
    }

  // Add points to the points to a temporary polydata
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);
  tempPolyData->GetPointData()->SetScalars(colors);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->Update();

  pdata->ShallowCopy(vertexGlyphFilter->GetOutput());
}

//Specialization for points with RGB values and normals
template <>
void PCLtoVTK<pcl::PointXYZRGBNormal> (CloudPointColorNormalPtr cloud, VTKPolyDataPtr pdata)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetNumberOfTuples(cloud->points.size());
  colors->SetName("RGB");

  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  normals->SetNumberOfTuples(cloud->points.size());
  normals->SetName("Normals");

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    points->InsertNextPoint ( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    unsigned char color[3] = {static_cast<unsigned char>(cloud->points[i].data_c[0]),
                              static_cast<unsigned char>(cloud->points[i].data_c[1]),
                              static_cast<unsigned char>(cloud->points[i].data_c[2])};
    colors->SetTupleValue(i, color);

    float normal[3] = {cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z};
    normals->SetTupleValue(i, normal);
    }

  // Add the points to a temporary polydata
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);
  tempPolyData->GetPointData()->SetScalars(colors);
  tempPolyData->GetPointData()->SetNormals(normals);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->Update();

  pdata->ShallowCopy(vertexGlyphFilter->GetOutput());
}


//Specialization for points with normals only
template <>
void PCLtoVTK<pcl::PointNormal> (CloudPointNormalPtr cloud, VTKPolyDataPtr pdata)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
  normals->SetNumberOfTuples(cloud->points.size());
  normals->SetName("Normals");

  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    points->InsertNextPoint ( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);

    float normal[3] = {cloud->points[i].normal_x, cloud->points[i].normal_y, cloud->points[i].normal_z};
    normals->SetTupleValue(i, normal);
    }

  // Add the points to a temporary polydata
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);
  tempPolyData->GetPointData()->SetNormals(normals);

  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->Update();

  pdata->ShallowCopy(vertexGlyphFilter->GetOutput());
}
