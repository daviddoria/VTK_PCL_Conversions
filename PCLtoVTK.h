#ifndef PCLtoVTK_H
#define PCLtoVTK_H

// STL
#include <iostream>
#include <vector>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>

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

int PolygonMeshToPolyData(const pcl::PolygonMesh &triangles, vtkPolyData* polyData);

//Template function declarations for inserting points of arbitrary dimension
template <typename PointT> 
void PCLtoVTK(typename pcl::PointCloud<PointT>::Ptr cloud, VTKPolyDataPtr pdata)
{
  // This generic template will convert any PCL point type with .x, .y, and .z members
  // to a coordinate-only vtkPolyData.
  //vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkPoints* points = vtkPoints::New();
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
  
  pdata->ShallowCopy(vertexGlyphFilter->GetOutput());
}


#endif
