#ifndef VTKtoPCL_H
#define VTKtoPCL_H

// VTK
class vtkPolyData;
class vtkStructuredGrid;

template <typename CloudT>
void VTKtoPCL(vtkPolyData* const polydata, CloudT& cloud);

template <typename CloudT>
void VTKtoPCL(vtkStructuredGrid* const structuredGrid, CloudT* const cloud);

#include "VTKtoPCL.hpp"

#endif
