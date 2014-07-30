/*
 *  iir_filter.cpp
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


#include <iir_filter/iir_filter.h>

IIRFilter::IIRFilter(Eigen::MatrixXd B, Eigen::MatrixXd A)
{
    setCoefficients(B, A);
}

IIRFilter::IIRFilter()
{

}

IIRFilter::~IIRFilter()
{

}

void IIRFilter::setCoefficients(Eigen::MatrixXd B, Eigen::MatrixXd A)
{
    B_ = B;
    A_ = A;

    x_ = Eigen::MatrixXd::Zero(B.rows(), 1);
    y_ = Eigen::MatrixXd::Zero(A.rows() - 1, 1);
}

double IIRFilter::filter(double x)
{
    Eigen::MatrixXd x2 = x_;
    x2(0, 0) = x;
    x2.bottomRows(x2.rows() - 1) = x_.topRows(x_.rows() - 1);
    x_ = x2;

    double y = (B_.transpose()*x_ - (A_.bottomRows(A_.rows()-1)).transpose()*y_)/A_(0, 0);

    Eigen::MatrixXd y2 = y_;
    y2(0, 0) = y;
    y2.bottomRows(y2.rows() - 1) = y_.topRows(y_.rows() - 1);

    return y;
}

void IIRFilter::reset()
{
    x_ = Eigen::MatrixXd::Zero(B_.rows(), 1);
    y_ = Eigen::MatrixXd::Zero(A_.rows() - 1, 1);
}
