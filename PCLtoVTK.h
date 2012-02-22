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

// PCL to vtkPolyData
template <typename CloudT>
void PCLtoVTK(CloudT* const cloud, vtkPolyData* const pdata);

template <>
void PCLtoVTK<CloudPointXYZRGBType> (CloudPointXYZRGBType* const cloud, vtkPolyData* const pdata);

template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkPolyData* const pdata);

template <>
void PCLtoVTK<CloudPointNormalType> (CloudPointNormalType* const cloud, vtkPolyData* const pdata);

// PCL to vtkStructuredGrid
template <typename CloudT>
void PCLtoVTK(CloudT* const cloud, vtkStructuredGrid* const structuredGrid);

template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkStructuredGrid* const structuredGrid);

#include "PCLtoVTK.hpp"

#endif
