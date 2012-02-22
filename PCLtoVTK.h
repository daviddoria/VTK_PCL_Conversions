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
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLPolyDataWriter.h>

//Some shorthand notation
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr          CloudPointColorPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr    CloudPointColorNormalPtr;
typedef pcl::PointCloud<pcl::PointNormal>::Ptr          CloudPointNormalPtr;

typedef pcl::PointCloud<pcl::PointXYZRGB>         CloudPointXYZRGBType;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal>   CloudPointXYZRGBNormalType;
typedef pcl::PointCloud<pcl::PointNormal>         CloudPointNormalType;

/** This function will convert a PCL PointCloud of any point type 
 * to a vtkPolyData. The PointCloud must have .x,.y,.z members.
 * If the PointCloud has .x_normal, .y_normal, and .z_normal members,
 * a Normals array will be created on the PointData. If the PointCloud
 * has .r, .g, and .b members, a Colors array will be created on the PointData.
 */
template <typename CloudT>
void PCLtoVTK(const CloudT& cloud, vtkPolyData* const pdata);

// PCL to vtkStructuredGrid
template <typename CloudT>
void PCLtoVTK(CloudT* const cloud, vtkStructuredGrid* const structuredGrid);

template <>
void PCLtoVTK<CloudPointXYZRGBNormalType> (CloudPointXYZRGBNormalType* const cloud, vtkStructuredGrid* const structuredGrid);

#include "PCLtoVTK.hpp"

#endif
