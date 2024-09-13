//
// Created by engineai on 2024/07/03.
//

#ifndef ZQ_HUMANOID_MATHUTILITIES_H
#define ZQ_HUMANOID_MATHUTILITIES_H

#include <eigen3/Eigen/Dense>

/*!
 * Square a number
 */
template <typename T>
T square(T a)
{
    return a * a;
}

/*!
 * Are two eigen matrices almost equal?
 */
template <typename T, typename T2>
bool almostEqual(const Eigen::MatrixBase<T> &a, const Eigen::MatrixBase<T> &b,
                 T2 tol)
{
    long x = T::RowsAtCompileTime;
    long y = T::ColsAtCompileTime;

    if (T::RowsAtCompileTime == Eigen::Dynamic ||
        T::ColsAtCompileTime == Eigen::Dynamic)
    {
        assert(a.rows() == b.rows());
        assert(a.cols() == b.cols());
        x = a.rows();
        y = a.cols();
    }

    for (long i = 0; i < x; i++)
    {
        for (long j = 0; j < y; j++)
        {
            T2 error = std::abs(a(i, j) - b(i, j));
            if (error >= tol)
                return false;
        }
    }
    return true;
}
template <typename T>
Eigen::Matrix<T, 3, 3> VectorCrossMatrix(Eigen::Matrix<T, 3, 1> v)
{
    Eigen::Matrix<T, 3, 3> rs;
    rs << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return rs;
}
#endif // ZQ_HUMANOID_MATHUTILITIES_H
