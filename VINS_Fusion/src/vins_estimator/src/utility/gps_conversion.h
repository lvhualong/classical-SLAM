#include <ros/ros.h>
#include <Eigen/Dense>

#define DEG2RAD (M_PI / 180.0)

class GPSConversion
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GPSConversion() : ecef_ref_orientation_(),
                      ecef_ref_point_(Eigen::Vector3d::Zero())
    {
        ecef_ref_orientation_.setIdentity();
    }

    ~GPSConversion() {}

    void InitReference(const double latitude,
                       const double longitude,
                       const double altitude)
    {
        Eigen::Matrix3d R;
        double s_long, s_lat, c_long, c_lat;
        sincos(latitude * DEG2RAD, &s_lat, &c_lat);
        sincos(longitude * DEG2RAD, &s_long, &c_long);

        R(0, 0) = -s_long;
        R(0, 1) = c_long;
        R(0, 2) = 0;

        R(1, 0) = -s_lat * c_long;
        R(1, 1) = -s_lat * s_long;
        R(1, 2) = c_lat;

        R(2, 0) = c_lat * c_long;
        R(2, 1) = c_lat * s_long;
        R(2, 2) = s_lat;

        ecef_ref_orientation_ = Eigen::Quaterniond(R);

        ecef_ref_point_ = WGS84ToECEF(latitude, longitude, altitude);
        ROS_WARN_STREAM("reference: "<< ecef_ref_point_[0]<<" "<<ecef_ref_point_[1]<<" "<<ecef_ref_point_[2]);
    }

    Eigen::Vector3d WGS84ToECEF(const double latitude,
                                const double longitude,
                                const double altitude) const
    {
        const double a = 6378137.0;           // semi-major axis
        const double e_sq = 6.69437999014e-3; // first eccentricity squared

        double s_long, s_lat, c_long, c_lat;
        sincos(latitude * DEG2RAD, &s_lat, &c_lat);
        sincos(longitude * DEG2RAD, &s_long, &c_long);

        const double N = a / sqrt(1 - e_sq * s_lat * s_lat);

        Eigen::Vector3d ecef;

        ecef[0] = (N + altitude) * c_lat * c_long;
        ecef[1] = (N + altitude) * c_lat * s_long;
        ecef[2] = (N * (1 - e_sq) + altitude) * s_lat;

        return ecef;
    }

    Eigen::Vector3d ECEFToENU(const Eigen::Vector3d &ecef) const
    {

        if (ecef_ref_point_.norm() == 0)
        {
            ROS_ERROR_STREAM_ONCE(
                "The gps reference is not initialized. Returning global coordinates. This warning will only show once.");
        }
        return ecef_ref_orientation_ * (ecef - ecef_ref_point_);
    }

    // Simply calls WGS84ToECEF(...) followed by ECEFToENU(...)
    Eigen::Vector3d WGS84ToENU(const double latitude,
                               const double longitude,
                               const double altitude) const
    {
        const Eigen::Vector3d ecef = this->WGS84ToECEF(latitude, longitude, altitude);
        return this->ECEFToENU(ecef);
    }

    void AdjustReference(const double z_correction)
    {
        ROS_WARN_STREAM("z-ref old: " << ecef_ref_point_(2));
        ecef_ref_point_(2) += z_correction;
        ROS_WARN_STREAM("z-ref new: " << ecef_ref_point_(2));
    }

private:
    Eigen::Quaterniond ecef_ref_orientation_;
    Eigen::Vector3d ecef_ref_point_;
};