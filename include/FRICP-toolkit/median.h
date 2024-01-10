#ifndef MEDIAN_H
#define MEDIAN_H
// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2013 Alec Jacobson <alecjacobson@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
#include <Eigen/Dense>
#include <vector>
namespace igl
{
    template <typename DerivedM>
void matrix_to_list(const Eigen::DenseBase<DerivedM>& M,
                    std::vector<typename DerivedM::Scalar> &V)
{
    using namespace std;
    V.resize(M.size());
    // loop over cols then rows
    for(int j =0; j<M.cols();j++)
    {
        for(int i = 0; i < M.rows();i++)
        {
            V[i+j*M.rows()] = M(i,j);
        }
    }
}
  // Compute the median of an eigen vector
  //
  // Inputs:
  //   V  #V list of unsorted values
  // Outputs:
  //   m  median of those values
  // Returns true on success, false on failure
  template <typename DerivedV, typename mType>
  bool median(
    const Eigen::MatrixBase<DerivedV> & V, mType & m)
  {
    using namespace std;
    if(V.size() == 0)
    {
      return false;
    }
    vector<typename DerivedV::Scalar> vV;
    matrix_to_list(V,vV);
    // http://stackoverflow.com/a/1719155/148668
    size_t n = vV.size()/2;
    nth_element(vV.begin(),vV.begin()+n,vV.end());
    if(vV.size()%2==0)
    {
      nth_element(vV.begin(),vV.begin()+n-1,vV.end());
      m = 0.5*(vV[n]+vV[n-1]);
    }else
    {
      m = vV[n];
    }
    return true;
  }
}
#endif
