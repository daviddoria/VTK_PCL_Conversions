#ifndef PCLtoVTK_H
#define PCLtoVTK_H

// VTK
class vtkPolyData;
class vtkStructuredGrid;

template <typename CloudT>
void PCLtoVTK(const CloudT& cloud, vtkPolyData* const pdata);

// PCL to vtkStructuredGrid
template <typename CloudT>
void PCLtoVTK(const CloudT& cloud, vtkStructuredGrid* const structuredGrid);

#include "PCLtoVTK.hpp"

#endif
