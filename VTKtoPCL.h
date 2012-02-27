#ifndef VTKtoPCL_H
#define VTKtoPCL_H

// VTK
class vtkPolyData;
class vtkStructuredGrid;

/** Convert a vtkPolyData to a pcl::PointCloud. */
template <typename CloudT>
void VTKtoPCL(vtkPolyData* const polydata, CloudT& cloud);

/** Convert a vtkStructuredGrid to a pcl::PointCloud. */
template <typename CloudT>
void VTKtoPCL(vtkStructuredGrid* const structuredGrid, CloudT& cloud);

#include "VTKtoPCL.hpp"

#endif
