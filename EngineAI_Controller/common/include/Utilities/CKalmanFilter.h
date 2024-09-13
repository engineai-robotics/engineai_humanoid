//
// Created by billchent on 2020/12/16.
//

#ifndef ZQ_HUMANOID_CKALMANFILTER_H
#define ZQ_HUMANOID_CKALMANFILTER_H
class CKalmanFilter
{
public:
    CKalmanFilter();
    virtual ~CKalmanFilter();
    float R_signal;
    float R_noise;
    float Filter(float data);
    float Filter(float data, float R_signal, float R_noise);
    float p1;
    float p;
    float s;
    float b;
};
#endif // ZQ_HUMANOID_CKALMANFILTER_H
