%module libaipudrv
%{
#define SWIG_FILE_WITH_INIT
#include "include/swig_cpp2py_api.hpp"
%}

%include "std_vector.i"
#if (defined BUILD_WITH_NUMPY)
%include "3rdparty/numpy/numpy.i"

%init %{
   import_array();
%}

%apply (int* IN_ARRAY1, int DIM1) {(int* data, int bytes)};
%apply (int DIM2, int* IN_ARRAY1, int DIM1) {(int id, int* data, int bytes)};
#endif

%template(IntVector) std::vector<int>;

%include "include/swig_cpp2py_api.hpp"