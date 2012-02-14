#include "PCLtoVTK.h"

#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkTriangleFilter.h>

//Specialization for points with RGB values
template <>
void PCLtoVTK<CloudPointXYZRGBType> (CloudPointXYZRGBType* const cloud, vtkPolyData* const pdata)
{
  std::cout << "PointXYZRGB" << std::endl;
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

  pdata->DeepCopy(vertexGlyphFilter->GetOutput());
}

//Specialization for points with RGB values and normals
template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkPolyData* const pdata)
{
  std::cout << "PointXYZRGBNormal" << std::endl;
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

  pdata->DeepCopy(vertexGlyphFilter->GetOutput());
}


//Specialization for points with normals only
template <>
void PCLtoVTK<CloudPointNormalType> (CloudPointNormalType* const cloud, vtkPolyData* const pdata)
{
  std::cout << "PointNormal" << std::endl;
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

  pdata->DeepCopy(vertexGlyphFilter->GetOutput());
}

// Coordinates, colors, and normals
template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkStructuredGrid* const structuredGrid)
{
  // This generic template will convert any PCL point type with .x, .y, and .z members
  // to a coordinate-only vtkPolyData.
  std::cout << "CloudPointXYZRGBNormalType specialization" << std::endl;

  int dimensions[3] = {cloud->width, cloud->height, 1};
  structuredGrid->SetDimensions(dimensions);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(cloud->width * cloud->height);

  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3); // Note this must come before the SetNumberOfTuples calls
  colors->SetNumberOfTuples(cloud->width * cloud->height);
  colors->SetName("Colors");

  vtkSmartPointer<vtkFloatArray> normals = vtkSmartPointer<vtkFloatArray>::New();
  normals->SetNumberOfComponents(3); // Note this must come before the SetNumberOfTuples calls
  normals->SetNumberOfTuples(cloud->width * cloud->height);
  normals->SetName("Normals");

  unsigned int numberOfInvalidPoints = 0;

  for (size_t i = 0; i < cloud->width; ++i)
  {
    for (size_t j = 0; j < cloud->height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
      CloudPointXYZRGBNormalType::PointType point = (*cloud)(i,j);

      if(pcl::isFinite(point))
      {
        float p[3] = {point.x, point.y, point.z};
        points->SetPoint(pointId, p);

        unsigned char c[3] = {point.r, point.g, point.b};
        colors->SetTupleValue(pointId, c);

        float n[3] = {point.normal_x, point.normal_y, point.normal_z};
        normals->SetTupleValue(pointId, n);
      }
      else
      {
        numberOfInvalidPoints++;
        float p[3] = {0,0,0};
        points->SetPoint(pointId, p);

        unsigned char c[3] = {0,0,0};
        colors->SetTupleValue(pointId, c);

        float n[3] = {0,0,0};
        normals->SetTupleValue(pointId, n);

        structuredGrid->BlankPoint(pointId);
      }
    }
  }

  std::cout << "There were " << numberOfInvalidPoints << " invalid points." << std::endl;

  structuredGrid->SetPoints(points); 
  structuredGrid->GetPointData()->AddArray(colors);
  structuredGrid->GetPointData()->SetNormals(normals);

}
