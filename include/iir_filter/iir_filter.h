/*
 *  iir_filter.h
 *
 *
 *  Created on: Jul 30, 2014
 *  Authors:   Francisco Viña
 *            fevb <at> kth.se
 */

/* Copyright (c) 2014, Francisco Viña, CVAP, KTH
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of KTH nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL KTH BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <eigen3/Eigen/Core>


// can be used to implement e.g. Butterworth filters
class IIRFilter
{
public:
    // initialize with B, A coefficients
    // it is assumed that these are row vectors
    // (follows Matlab model for filter function)
    IIRFilter(Eigen::MatrixXd B, Eigen::MatrixXd A);
    IIRFilter();

    virtual ~IIRFilter();

    void setCoefficients(Eigen::MatrixXd B, Eigen::MatrixXd A);

    double update(double x);

    void reset(double x0=0, double y0=0);

private:

    Eigen::MatrixXd B_;
    Eigen::MatrixXd A_;

    Eigen::MatrixXd x_;
    Eigen::MatrixXd y_;

    // helper variables
    Eigen::MatrixXd A2_;
    Eigen::MatrixXd x2_;
    Eigen::MatrixXd y2_;
    Eigen::Matrix<double, 1, 1> y_output_;
};
