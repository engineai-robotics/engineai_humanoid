//
// Created by billchent on 2020/12/16.
//
#include "../include/Utilities/CKalmanFilter.h"

CKalmanFilter::CKalmanFilter()
{
    R_signal = 0.5;
    R_noise = 5;
    s = 0;
    p = 0;
}

CKalmanFilter::~CKalmanFilter()
{
}
float CKalmanFilter::Filter(float data)
{
    p1 = p + R_signal;
    b = p1 / (p1 + R_noise);
    s = s + b * (data - s);
    p = p1 - b * p1;
    return s;
}
float CKalmanFilter::Filter(float data, float r_signal, float r_noise)
{
    p1 = p + r_signal;
    b = p1 / (p1 + r_noise);
    s = s + b * (data - s);
    p = p1 - b * p1;
    return s;
}
