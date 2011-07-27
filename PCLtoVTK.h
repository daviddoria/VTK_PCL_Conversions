#ifndef PCLtoVTK_H
#define PCLtoVTK_H

// STL
#include <iostream>
#include <vector>

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
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointColorPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointColorNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;
typedef vtkSmartPointer<vtkPoints>                      VTKPointsPtr;
typedef vtkSmartPointer<vtkPolyData>                    VTKPolyDataPtr;

//Template function declarations for inserting points of arbitrary dimension
template <typename PointT> 
void PCLtoVTK(typename pcl::PointCloud<PointT>::Ptr cloud, VTKPolyDataPtr pdata)
{
  // This generic template will convert any PCL point type with .x, .y, and .z members
  // to a coordinate-only vtkPolyData.
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    points->InsertNextPoint ( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z );
    }
    
  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);
    
  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->Update();
  
  pdata->ShallowCopy(vertexGlyphFilter->GetOutput());
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

#endif
