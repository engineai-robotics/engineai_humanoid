#include "Controllers/PositionVelocityEstimator.h"

/*!
 * Initialize the state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::setup()
{
    T dt = this->_stateEstimatorData.parameters->controller_dt;
    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity();
    _A.block(6, 6, 6, 6) = Eigen::Matrix<T, 6, 6>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<T, 3, 3>::Identity(), Eigen::Matrix<T, 3, 3>::Zero();
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<T, 3, 3>::Zero(), Eigen::Matrix<T, 3, 3>::Identity();
    _C.setZero();

    _C.block(0, 0, 3, 6) = C1;
    _C.block(3, 0, 3, 6) = C1;
    _C.block(6, 0, 3, 6) = C2;
    _C.block(9, 0, 3, 6) = C2;
    _C.block(0, 6, 6, 6) = T(-1) * Eigen::Matrix<T, 6, 6>::Identity();
    _C(13, 11) = T(1);
    _C(12, 8) = T(1);
    _P.setIdentity();
    _P = T(100) * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<T, 3, 3>::Identity();
    _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<T, 6, 6>::Identity();
    _R0.setIdentity();
}

template <typename T>
LinearKFPositionVelocityEstimator<T>::LinearKFPositionVelocityEstimator() {}

/*!
 * Run state estimator
 */
template <typename T>
void LinearKFPositionVelocityEstimator<T>::run()
{

    static int sleep_time = 0;
    sleep_time++;
    if (sleep_time > 5)
    {
        T process_noise_pimu = 0.02;
        T process_noise_vimu = 0.02;
        T process_noise_pfoot = 0.002;
        T sensor_noise_pimu_rel_foot = 0.001;
        T sensor_noise_vimu_rel_foot = 0.1;
        T sensor_noise_zfoot = 0.001;

        Eigen::Matrix<T, 12, 12> Q = Eigen::Matrix<T, 12, 12>::Identity(); // process noise
        Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
        Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
        Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 6, 6) * process_noise_pfoot;

        Eigen::Matrix<T, 14, 14> R = Eigen::Matrix<T, 14, 14>::Identity(); // measurement noise

        R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
        R.block(6, 6, 6, 6) = _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
        R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;

        int qindex = 0;
        int rindex1 = 0;
        int rindex2 = 0;
        int rindex3 = 0;

        Vec3<T> g(0, 0, T(-9.81));
        Mat3<T> Rbod = this->_stateEstimatorData.result->rBody.transpose();

        Vec3<T> a = this->_stateEstimatorData.result->aWorld + g;

        T acc_filter_ratio = this->_stateEstimatorData.parameters->IMU_acc_filter_ratio;

        if (acc_filter_ratio - 1.0 > 1e6)
            acc_filter_ratio = 1.0;
        else if (acc_filter_ratio < 0.0)
            acc_filter_ratio = 0.0;
        else
            acc_filter_ratio = acc_filter_ratio;

        a = (1 - acc_filter_ratio) * acc_body_pre + acc_filter_ratio * a;

        Vec2<T> pzs = Vec2<T>::Zero();
        Vec2<T> trusts = Vec2<T>::Zero();
        Vec3<T> p0, v0;
        p0 << _xhat[0], _xhat[1], _xhat[2];
        v0 << _xhat[3], _xhat[4], _xhat[5];

        Vec3<T> omegaBody = this->_stateEstimatorData.result->omegaBody;
        T omega_filter_ratio = this->_stateEstimatorData.parameters->IMU_omega_filter_ratio;

        if (omega_filter_ratio - 1.0 > 1e6)
            omega_filter_ratio = 1.0;
        else if (omega_filter_ratio < 0.0)
            omega_filter_ratio = 0.0;
        else
            omega_filter_ratio = omega_filter_ratio;

        omegaBody = (1 - omega_filter_ratio) * omega_body_pre + omega_filter_ratio * omegaBody;

        omega_body_pre = omegaBody;

        for (int i = 0; i < 2; i++)
        {
            int i1 = 3 * i;
            RobotConstructor<T> &humanoid_biped = *(this->_stateEstimatorData.legControllerData->humanoid_biped);
            Vec3<T> ph = humanoid_biped.getHipLocation(i); // hip positions relative to CoM
            Vec3<T> p_rel = ph + this->_stateEstimatorData.legControllerData[i].p;
            Vec3<T> dp_rel = this->_stateEstimatorData.legControllerData[i].v;

            T ratio = this->_stateEstimatorData.parameters->foot_vel_filter_ratio;

            if (ratio - 1.0 > 1e6)
                ratio = 1.0;
            else if (ratio < 0.0)
                ratio = 0.0;
            else
                ratio = ratio;

            dp_rel = (1 - ratio) * dp_f_pre + ratio * dp_rel;

            dp_f_pre = dp_rel;

            Vec3<T> p_f = Rbod * p_rel;

            Vec3<T> dp_f = Rbod * (omegaBody.cross(p_rel) + dp_rel);

            qindex = 6 + i1;
            rindex1 = i1;

            rindex2 = 6 + i1;
            rindex3 = 12 + i;

            T trust = T(1);
            T phase = fmin(this->_stateEstimatorData.result->contactEstimate(i), T(1));

            T trust_window = T(0.2);

            if (phase < trust_window)
            {
                trust = phase / trust_window;
            }
            else if (phase > (T(1) - trust_window))
            {
                trust = (T(1) - phase) / trust_window;
            }
            T high_suspect_number(999999);

            Q.block(qindex, qindex, 3, 3) =
                (T(1) + (T(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
            R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
            R.block(rindex2, rindex2, 3, 3) =
                (T(1) + (T(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
            R(rindex3, rindex3) =
                (T(1) + (T(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

            trusts(i) = trust;

            _ps.segment(i1, 3) = -p_f;
            _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
            pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
        }

        Eigen::Matrix<T, 14, 1> y;
        y << _ps, _vs, pzs;

        _xhat = _A * _xhat + _B * a;

        Eigen::Matrix<T, 12, 12> At = _A.transpose();
        Eigen::Matrix<T, 12, 12> Pm = _A * _P * At + Q; // covariance of predicition error
        Eigen::Matrix<T, 12, 14> Ct = _C.transpose();
        Eigen::Matrix<T, 14, 1> yModel = _C * _xhat;
        Eigen::Matrix<T, 14, 1> ey = y - yModel;
        Eigen::Matrix<T, 14, 14> S = _C * Pm * Ct + R; // covariance of measurement error

        Eigen::Matrix<T, 14, 1> S_ey = S.lu().solve(ey);
        _xhat += Pm * Ct * S_ey;

        Eigen::Matrix<T, 14, 12> S_C = S.lu().solve(_C);
        _P = (Eigen::Matrix<T, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

        //        Eigen::Matrix<T, 18, 18> Pt = _P.transpose();
        Eigen::Matrix<T, 12, 12> Pt = _P.transpose();
        _P = (_P + Pt) / T(2);

        if (_P.block(0, 0, 2, 2).determinant() > T(0.000001))
        { //? 表示xy位置的增益协方差大于0
            _P.block(0, 2, 2, 10).setZero();
            _P.block(2, 0, 10, 2).setZero();
            _P.block(0, 0, 2, 2) /= T(10);
        }

        this->_stateEstimatorData.result->position = _xhat.block(0, 0, 3, 1);
        this->_stateEstimatorData.result->vWorld = _xhat.block(3, 0, 3, 1);
        this->_stateEstimatorData.result->vBody =
            this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;

        if (first_run)
        {
            init_pos = this->_stateEstimatorData.result->position;
            first_run = false;
        }
        else
        {
            Vec3<T> pos_drift = this->_stateEstimatorData.result->position - init_pos;
            T current_time = timer_obs.getMs();
        }

        this->_stateEstimatorData.result->vBody =
            this->_stateEstimatorData.result->rBody * this->_stateEstimatorData.result->vWorld;
    }
}

template class LinearKFPositionVelocityEstimator<float>;
template class LinearKFPositionVelocityEstimator<double>;

/*!
 * Run cheater estimator to copy cheater state into state estimate
 */
template <typename T>
void CheaterPositionVelocityEstimator<T>::run()
{
    this->_stateEstimatorData.result->position = this->_stateEstimatorData.cheaterState->position.template cast<T>();
    this->_stateEstimatorData.result->vWorld =
        this->_stateEstimatorData.result->rBody.transpose().template cast<T>() * this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
    this->_stateEstimatorData.result->vBody = this->_stateEstimatorData.cheaterState->vBody.template cast<T>();
}

template class CheaterPositionVelocityEstimator<float>;
template class CheaterPositionVelocityEstimator<double>;
