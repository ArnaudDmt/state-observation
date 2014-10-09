/**
 * \file      imu-attitude-trajectory-reconstruction.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Gives an implementation of attitude estimation for IMU reconstruction
 *             with or without given input. The source is in a file
 *             imu-attitude-trajectory-reconstruction.hxx
 *
 * \details
 *
 *
 */

#ifndef IMUATTITUDETRAJECTORYRECONTRUCTIONHPP
#define IMUATTITUDETRAJECTORYRECONTRUCTIONHPP


#include <state-observation/dynamical-system/imu-dynamical-system.hpp>
#include <state-observation/dynamical-system/dynamical-system-simulator.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace stateObservation
{
    namespace examples
    {

        /*! \fn IndexedMatrixArray imuAttitudeTrajectoryReconstruction(
         *   const IndexedMatrixArray & y,
         *   const IndexedMatrixArray & u,
         *   const Vector & xh0,
         *   const Matrix & p,
         *   const Matrix & q,
         *   const Matrix & r,
         *   double dt);
         *
         *  \brief Provides the estimation of the state (mostly attitude) of an
         *         IMU, given the measurements of the IMU and the input
         *         (which provides the acceleration/jerk). This method uses
         *         extended Kalman filtering, we need to provide it with an
         *         initial guess, a covariance matrix of this initial guess
         *         and covariance matrices of the state perturbations and
         *         measurement noises.
         *
         *
         *  \param y IMU measurements
         *  \param u the inputs of the dynamical system
         *  \param xh0 an initial guess of the state
         *  \param p the covariance matrix of the error of the initial guess
         *  \param q the covariance matrix of the process noise (state perturbation)
         *  \param r the covariance matrix of the measurement noise
         *  \param dt the time discretization period
         */

        IndexedMatrixArray imuAttitudeTrajectoryReconstruction(
            const IndexedMatrixArray & y,
            const IndexedMatrixArray & u,
            const Vector & xh0,
            const Matrix & p,
            const Matrix & q,
            const Matrix & r,
            double dt);


        /*! \fn IndexedMatrixArray imuAttitudeTrajectoryReconstruction(
         *   const IndexedMatrixArray & y,
         *   const Vector & xh0,
         *   const Matrix & p,
         *   const Matrix & q,
         *   const Matrix & r,
         *   double dt);
         *
         *  \brief Provides the estimation of the state (mostly attitude) of an
         *         IMU, given the measurements of the IMU without knowing the input.
         *         The input is assumed to be zero over the observation. This method uses
         *         extended Kalman filtering, we need to provide it with an
         *         initial guess, a covariance matrix of this initial guess
         *         and covariance matrices of the state perturbations and
         *         measurement noises.
         *
         *
         *  \param y IMU measurements
         *  \param xh0 an initial guess of the state
         *  \param p the covariance matrix of the error of the initial guess
         *  \param q the covariance matrix of the process noise (state perturbation)
         *  \param r the covariance matrix of the measurement noise
         *  \param dt the time discretization period
         */
        IndexedMatrixArray imuAttitudeTrajectoryReconstruction(
            const IndexedMatrixArray & y,
            const Vector & xh0,
            const Matrix & p,
            const Matrix & q,
            const Matrix & r,
            double dt);




#include <state-observation/examples/imu-attitude-trajectory-reconstruction.hxx>

    }

}

#endif //IMUATTITUDETRAJECTORYRECONTRUCTIONHPP
