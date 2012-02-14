#ifndef PCLtoVTK_H
#define PCLtoVTK_H

// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/common/common.h>

// VTK
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataWriter.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointColorPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointColorNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;

typedef pcl::PointCloud<pcl::PointXYZRGB>         CloudPointXYZRGBType;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>   CloudPointXYZRGBNormalType;
typedef pcl::PointCloud<pcl::PointNormal>         CloudPointNormalType;

//Template function declarations for inserting points of arbitrary dimension
template <typename CloudT>
void PCLtoVTK(CloudT* const cloud, vtkPolyData* const pdata)
{
  // This generic template will convert any PCL point type with .x, .y, and .z members
  // to a coordinate-only vtkPolyData.
  std::cout << "Generic" << std::endl;
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (size_t i = 0; i < cloud->points.size (); ++i)
    {
    std::cout << cloud->points[i].x << " " << cloud->points[i].y << " " <<  cloud->points[i].z << std::endl;
    //points->InsertNextPoint ( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z );
    double p[3];
    p[0] = cloud->points[i].x;
    p[1] = cloud->points[i].y;
    p[2] = cloud->points[i].z;
    points->InsertNextPoint ( p);
    }

  // Create a temporary PolyData and add the points to it
  vtkSmartPointer<vtkPolyData> tempPolyData = vtkSmartPointer<vtkPolyData>::New();
  tempPolyData->SetPoints(points);
    
  vtkSmartPointer<vtkVertexGlyphFilter> vertexGlyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexGlyphFilter->AddInputConnection(tempPolyData->GetProducerPort());
  vertexGlyphFilter->Update();

  pdata->DeepCopy(vertexGlyphFilter->GetOutput());
}

//Template function declarations for inserting points of arbitrary dimension
template <typename CloudT>
void PCLtoVTK(CloudT* const cloud, vtkStructuredGrid* const structuredGrid)
{
  // This generic template will convert any PCL point type with .x, .y, and .z members
  // to a coordinate-only vtkPolyData.
  std::cout << "Generic" << std::endl;
  
  int dimensions[3] = {cloud->width, cloud->height, 1};
  structuredGrid->SetDimensions(dimensions);

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetNumberOfPoints(cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->width; ++i)
  {
    for (size_t j = 0; j < cloud->height; ++j)
    {
      int queryPoint[3] = {i, j, 0};
      vtkIdType pointId = vtkStructuredData::ComputePointId(dimensions, queryPoint);
      typename CloudT::PointType point = (*cloud)(i,j);
      
      if(pcl::isFinite(point))
      {
        float p[3] = {point.x, point.y, point.z};
        points->SetPoint(pointId, p);
      }
      else
      {
        float p[3] = {0,0,0};
        points->SetPoint(pointId, p);
        structuredGrid->BlankPoint(pointId);
      }
    }
  }

  structuredGrid->SetPoints(points);

}

template <>
void PCLtoVTK<CloudPointXYZRGBType> (CloudPointXYZRGBType* const cloud, vtkPolyData* const pdata);

template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkPolyData* const pdata);

template <>
void PCLtoVTK<CloudPointNormalType> (CloudPointNormalType* const cloud, vtkPolyData* const pdata);

#endif
