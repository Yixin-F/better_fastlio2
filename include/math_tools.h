// math tool

#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

// hat
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0;
    return skew_sym_mat;
}

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

/*
 * @brief Normalize the given quaternion to unit quaternion.
 */
inline void quaternionNormalize(Eigen::Vector4d& q) {
  double norm = q.norm();
  q = q / norm;
  return;
}

/*
 * @brief Perform q1 * q2.
 *  
 *    Format of q1 and q2 is as [x,y,z,w]
 */
inline Eigen::Vector4d quaternionMultiplication(
    const Eigen::Vector4d& q1,
    const Eigen::Vector4d& q2) {
  Eigen::Matrix4d L;

  // QXC: Hamilton
  L(0, 0) =  q1(3); L(0, 1) = -q1(2); L(0, 2) =  q1(1); L(0, 3) =  q1(0);
  L(1, 0) =  q1(2); L(1, 1) =  q1(3); L(1, 2) = -q1(0); L(1, 3) =  q1(1);
  L(2, 0) = -q1(1); L(2, 1) =  q1(0); L(2, 2) =  q1(3); L(2, 3) =  q1(2);
  L(3, 0) = -q1(0); L(3, 1) = -q1(1); L(3, 2) = -q1(2); L(3, 3) =  q1(3);

  Eigen::Vector4d q = L * q2;
  quaternionNormalize(q);
  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
inline Eigen::Vector4d smallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {

  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Vector4d q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.head<3>() = dq;
    q(3) = std::sqrt(1-dq_square_norm);
  } else {
    q.head<3>() = dq;
    q(3) = 1;
    q = q / std::sqrt(1+dq_square_norm);
  }

  return q;
}

/*
 * @brief Convert the vector part of a quaternion to a
 *    full quaternion.
 * @note This function is useful to convert delta quaternion
 *    which is usually a 3x1 vector to a full quaternion.
 *    For more details, check Section 3.2 "Kalman Filter Update" in
 *    "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for quaternion Algebra".
 */
inline Eigen::Quaterniond getSmallAngleQuaternion(
    const Eigen::Vector3d& dtheta) {

  Eigen::Vector3d dq = dtheta / 2.0;
  Eigen::Quaterniond q;
  double dq_square_norm = dq.squaredNorm();

  if (dq_square_norm <= 1) {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = std::sqrt(1-dq_square_norm);
  } else {
    q.x() = dq(0);
    q.y() = dq(1);
    q.z() = dq(2);
    q.w() = 1;
    q.normalize();
  }

  return q;
}

/*
 * @brief Convert a quaternion to the corresponding rotation matrix
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
inline Eigen::Matrix3d quaternionToRotation(
    const Eigen::Vector4d& q) {
  // QXC: Hamilton
  const double& qw = q(3);
  const double& qx = q(0);
  const double& qy = q(1);
  const double& qz = q(2);
  Eigen::Matrix3d R;
  R(0, 0) = 1-2*(qy*qy+qz*qz);  R(0, 1) =   2*(qx*qy-qw*qz);  R(0, 2) =   2*(qx*qz+qw*qy);
  R(1, 0) =   2*(qx*qy+qw*qz);  R(1, 1) = 1-2*(qx*qx+qz*qz);  R(1, 2) =   2*(qy*qz-qw*qx);
  R(2, 0) =   2*(qx*qz-qw*qy);  R(2, 1) =   2*(qy*qz+qw*qx);  R(2, 2) = 1-2*(qx*qx+qy*qy);

  return R;
}

/*
 * @brief Convert a rotation matrix to a quaternion.
 * @note Pay attention to the convention used. The function follows the
 *    conversion in "Indirect Kalman Filter for 3D Attitude Estimation:
 *    A Tutorial for Quaternion Algebra", Equation (78).
 *
 *    The input quaternion should be in the form
 *      [q1, q2, q3, q4(scalar)]^T
 */
inline Eigen::Vector4d rotationToQuaternion(
    const Eigen::Matrix3d& R) {
  Eigen::Vector4d score;
  score(0) = R(0, 0);
  score(1) = R(1, 1);
  score(2) = R(2, 2);
  score(3) = R.trace();

  int max_row = 0, max_col = 0;
  score.maxCoeff(&max_row, &max_col);

  Eigen::Vector4d q = Eigen::Vector4d::Zero();

  // QXC: Hamilton
  if (max_row == 0) {
    q(0) = std::sqrt(1+2*R(0, 0)-R.trace()) / 2.0;
    q(1) = (R(0, 1)+R(1, 0)) / (4*q(0));
    q(2) = (R(0, 2)+R(2, 0)) / (4*q(0));
    q(3) = (R(2, 1)-R(1, 2)) / (4*q(0));
  } else if (max_row == 1) {
    q(1) = std::sqrt(1+2*R(1, 1)-R.trace()) / 2.0;
    q(0) = (R(0, 1)+R(1, 0)) / (4*q(1));
    q(2) = (R(1, 2)+R(2, 1)) / (4*q(1));
    q(3) = (R(0, 2)-R(2, 0)) / (4*q(1));
  } else if (max_row == 2) {
    q(2) = std::sqrt(1+2*R(2, 2)-R.trace()) / 2.0;
    q(0) = (R(0, 2)+R(2, 0)) / (4*q(2));
    q(1) = (R(1, 2)+R(2, 1)) / (4*q(2));
    q(3) = (R(1, 0)-R(0, 1)) / (4*q(2));
  } else {
    q(3) = std::sqrt(1+R.trace()) / 2.0;
    q(0) = (R(2, 1)-R(1, 2)) / (4*q(3));
    q(1) = (R(0, 2)-R(2, 0)) / (4*q(3));
    q(2) = (R(1, 0)-R(0, 1)) / (4*q(3));
  }

  if (q(3) < 0) q = -q;
  quaternionNormalize(q);
  return q;
}

// Sign function
template <typename T>
T sgnFunc(T val)
{
    return (T(0) < val) - (val < T(0));
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = q.w(), ans.template block<1, 3>(0, 1) = -q.vec().transpose();
    ans.template block<3, 1>(1, 0) = q.vec(), ans.template block<3, 3>(1, 1) = q.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(q.vec());
    return ans;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
{
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = p.w(), ans.template block<1, 3>(0, 1) = -p.vec().transpose();
    ans.template block<3, 1>(1, 0) = p.vec(), ans.template block<3, 3>(1, 1) = p.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(p.vec());
    return ans;
}

// Convert from quaternion to rotation vector
template <typename T>
inline Eigen::Matrix<T, 3, 1> quaternionToRotationVector(const Eigen::Quaternion<T> &qua)
{
    Eigen::Matrix<T, 3, 3> mat = qua.toRotationMatrix();
    Eigen::Matrix<T, 3, 1> rotation_vec;
    Eigen::AngleAxis<T> angle_axis;
    angle_axis.fromRotationMatrix(mat);
    rotation_vec = angle_axis.angle() * angle_axis.axis();
    return rotation_vec;
}

// Right Jacobian matrix
template <typename T>
inline Eigen::Matrix3d Jright(const Eigen::Quaternion<T> &qua)
{
    Eigen::Matrix<T, 3, 3> mat;
    Eigen::Matrix<T, 3, 1> rotation_vec = quaternionToRotationVector(qua);
    double theta_norm = rotation_vec.norm();
    mat = Eigen::Matrix<T, 3, 3>::Identity()
            - (1 - cos(theta_norm)) / (theta_norm * theta_norm + 1e-10) * hat(rotation_vec)
            + (theta_norm - sin(theta_norm)) / (theta_norm * theta_norm * theta_norm + 1e-10) * hat(rotation_vec) * hat(rotation_vec);
    return mat;
}

// Calculate the Jacobian with respect to the quaternion
template <typename T>
inline Eigen::Matrix<T, 3, 4> quaternionJacobian(const Eigen::Quaternion<T> &qua, const Eigen::Matrix<T, 3, 1> &vec)
{
    Eigen::Matrix<T, 3, 4> mat;
    Eigen::Matrix<T, 3, 1> quaternion_imaginary(qua.x(), qua.y(), qua.z());

    mat.template block<3, 1>(0, 0) = qua.w() * vec + quaternion_imaginary.cross(vec);
    mat.template block<3, 3>(0, 1) = quaternion_imaginary.dot(vec) * Eigen::Matrix<T, 3, 3>::Identity()
            + quaternion_imaginary * vec.transpose()
            - vec * quaternion_imaginary.transpose()
            - qua.w() * hat(vec);
    return T(2) * mat;
}

// Calculate the Jacobian with respect to the inverse quaternion
template <typename T>
inline Eigen::Matrix<T, 3, 4> quaternionInvJacobian(const Eigen::Quaternion<T> &qua, const Eigen::Matrix<T, 3, 1> &vec)
{
    Eigen::Matrix<T, 3, 4> mat;
    Eigen::Matrix<T, 3, 1> quaternion_imaginary(qua.x(), qua.y(), qua.z());

    mat.template block<3, 1>(0, 0) = qua.w() * vec + vec.cross(quaternion_imaginary);
    mat.template block<3, 3>(0, 1) = quaternion_imaginary.dot(vec) * Eigen::Matrix<T, 3, 3>::Identity()
            + quaternion_imaginary * vec.transpose()
            - vec * quaternion_imaginary.transpose()
            + qua.w() * hat(vec);
    return T(2) * mat;
}

// Calculate the Jacobian rotation vector to quaternion
template <typename T>
inline Eigen::Matrix<T, 3, 4> JacobianV2Q(const Eigen::Quaternion<T> &qua)
{
    Eigen::Matrix<T, 3, 4> mat;

    T c = 1 / (1 - qua.w() * qua.w());
    T d = acos(qua.w()) / sqrt(1 - qua.w() * qua.w());

    mat.template block<3, 1>(0, 0) = Eigen::Matrix<T, 3, 1>(c * qua.x() * (d * qua.x() - 1),
                                                            c * qua.y() * (d * qua.x() - 1),
                                                            c * qua.z() * (d * qua.x() - 1));
    mat.template block<3, 3>(0, 1) = d * Eigen::Matrix<T, 3, 4>::Identity();
    return T(2) * mat;
}

//get quaternion from rotation vector
template <typename Derived>
Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived> &theta)
{
    typedef typename Derived::Scalar Scalar_t;

    Eigen::Quaternion<Scalar_t> dq;
    Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
    half_theta /= static_cast<Scalar_t>(2.0);
    dq.w() = static_cast<Scalar_t>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q) {
    Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
    Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
    typename Derived::Scalar q4 = q.w();
    m.block(0, 0, 3, 3) << q4 * Eigen::Matrix3d::Identity() + skewSymmetric(vq);
    m.block(3, 0, 1, 3) << -vq.transpose();
    m.block(0, 3, 3, 1) << vq;
    m(3, 3) = q4;
    return m;
}

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p) {
    Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
    Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
    typename Derived::Scalar p4 = p.w();
    m.block(0, 0, 3, 3) << p4 * Eigen::Matrix3d::Identity() - skewSymmetric(vp);
    m.block(3, 0, 1, 3) << -vp.transpose();
    m.block(0, 3, 3, 1) << vp;
    m(3, 3) = p4;
    return m;
}


template <typename T>
Eigen::Quaternion<T> unifyQuaternion(const Eigen::Quaternion<T> &q)
{
    if(q.w() >= 0) return q;
    else {
        Eigen::Quaternion<T> resultQ(-q.w(), -q.x(), -q.y(), -q.z());
        return resultQ;
    }
}



#endif
