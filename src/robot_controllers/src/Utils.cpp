#include "Utils.h"


namespace controllers{

Vector3d utils::quat_to_euler(Eigen::Quaterniond quat) {
    Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Vector4d coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
    }

    //zyx顺序
Matrix3d utils::euler_to_rot(Vector3d euler)
{
    Matrix3d result;
    double ts0,tc0, ts1,tc1, ts2,tc2;
    ts0 = std::sin(euler[0]); tc0 = std::cos(euler[0]);
    ts1 = std::sin(euler[1]); tc1 = std::cos(euler[1]);
    ts2 = std::sin(euler[2]); tc2 = std::cos(euler[2]);
    Matrix3d rot_yaw;
    rot_yaw <<  tc2, -ts2, 0,
                ts2, tc2, 0,
                0,   0,   1;
    Matrix3d rot_pitch;
    rot_pitch <<    tc1, 0, ts1,
                    0,  1,  0,
                    -ts1,  0, tc1;
    Matrix3d rot_roll;
    rot_roll <<    1, 0, 0,
                    0,  tc0,  -ts0,
                    0,  ts0, tc0;
    result = rot_yaw * rot_pitch * rot_roll;
    return result;
}

Vector3d utils::bezier_pos(Vector3d start,Vector3d end,double swing_high,double devel)
{
    double bezier_A;
    double bezier_B;
    Vector3d result;
    //pos
    Vector3d bezier_delta;
    bezier_delta = end - start;

    bezier_A = std::pow(devel,3)+3*std::pow(devel,2)*(1-devel);
    result[0] = start[0] + bezier_A * bezier_delta[0];
    result[1] = start[1] + bezier_A * bezier_delta[1];

    static double Z_start,Z_mid,Z_end;
    Z_start = start[2];
    Z_mid = end[2] + swing_high;
    Z_end =  end[2];
    float t1 = ( devel<=0.5 ?  ( devel*2 ) : ( devel*2-1 ));
    bezier_B = std::pow(t1,3)+3*std::pow(t1,2)*(1-t1);

    if( devel<=0.5 ) { result[2] = Z_start+ bezier_B*(Z_mid - Z_start);    } 
    else { result[2] = Z_mid + bezier_B*(Z_end - Z_mid); }

    return result;
}

Vector3d utils::bezier_vel(Vector3d start,Vector3d end,double swing_high,double devel)
{
    double bezier_C;
    double bezier_D;
    Vector3d result;
    //vel
    Vector3d bezier_delta;
    bezier_delta = end - start;

    bezier_C = 6*devel*(1-devel);

    result[0] = bezier_C*bezier_delta[0];
    result[1] = bezier_C*bezier_delta[1];
    
    static double Z_start,Z_mid,Z_end;
    Z_start = start[2];
    Z_mid = end[2] + swing_high;
    Z_end = end[2];
    float t2 = ( devel<=0.5 ?  (devel*2) : (devel*2-1));
    bezier_D = 6*t2*(1-t2);
    if(devel<=0.5) { result[2] = bezier_D*(Z_mid - Z_start);  }
    else {  result[2] = bezier_D*( Z_end - Z_mid ); }

    return result;
}

Matrix3d utils::skew(Vector3d vec)
{
    Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

}