#ifndef TYPES_H
#define TYPES_H

#include <eigen3/Eigen/Dense>

#ifdef USE_FLOAT_SCALAR
typedef float Scalar
#else
typedef double Scalar;
#endif

#ifdef EIGEN_DONT_ALIGN
#define EIGEN_ALIGNMENT Eigen::DontAlign
#else
#define EIGEN_ALIGNMENT Eigen::AutoAlign
#endif


template < int Rows, int Cols, int Options = (Eigen::ColMajor | EIGEN_ALIGNMENT) >
using MatrixT = Eigen::Matrix<Scalar, Rows, Cols, Options>; ///< A typedef of the dense matrix of Eigen.
typedef MatrixT<2, 1> Vector2;								///< A 2d column vector.
typedef MatrixT<2, 2> Matrix22;								///< A 2 by 2 matrix.
typedef MatrixT<2, 3> Matrix23;								///< A 2 by 3 matrix.
typedef MatrixT<3, 1> Vector3;								///< A 3d column vector.
typedef MatrixT<3, 2> Matrix32;								///< A 3 by 2 matrix.
typedef MatrixT<3, 3> Matrix33;								///< A 3 by 3 matrix.
typedef MatrixT<3, 4> Matrix34;								///< A 3 by 4 matrix.
typedef MatrixT<4, 1> Vector4;								///< A 4d column vector.
typedef MatrixT<4, 4> Matrix44;								///< A 4 by 4 matrix.
typedef MatrixT<4, Eigen::Dynamic> Matrix4X;				///< A 4 by n matrix.
typedef MatrixT<3, Eigen::Dynamic> Matrix3X;				///< A 3 by n matrix.
typedef MatrixT<Eigen::Dynamic, 3> MatrixX3;				///< A n by 3 matrix.
typedef MatrixT<2, Eigen::Dynamic> Matrix2X;				///< A 2 by n matrix.
typedef MatrixT<Eigen::Dynamic, 2> MatrixX2;				///< A n by 2 matrix.
typedef MatrixT<Eigen::Dynamic, 1> VectorX;					///< A nd column vector.
typedef MatrixT<Eigen::Dynamic, Eigen::Dynamic> MatrixXX;	///< A n by m matrix.
typedef Eigen::Matrix<Scalar, 12, 12, 0, 12, 12> EigenMatrix12;

// eigen quaternions
typedef Eigen::AngleAxis<Scalar> EigenAngleAxis;
typedef Eigen::Quaternion<Scalar, Eigen::DontAlign> EigenQuaternion;

// Conversion between a 3d vector type to Eigen::Vector3d
template<typename Vec_T>
inline Vector3 to_eigen_vec3(const Vec_T &vec)
{
    return Vector3(vec[0], vec[1], vec[2]);
}


template<typename Vec_T>
inline Vec_T from_eigen_vec3(const Vector3 &vec)
{
    Vec_T v;
    v[0] = vec(0);
    v[1] = vec(1);
    v[2] = vec(2);

    return v;
}


class Matrix3333 // 3x3 matrix: each element is a 3x3 matrix
{
public:
    Matrix3333();
    Matrix3333(const Matrix3333& other);
    ~Matrix3333() {}

    void SetZero(); // [0 0 0; 0 0 0; 0 0 0]; 0 = 3x3 zeros
    void SetIdentity(); //[I 0 0; 0 I 0; 0 0 I]; 0 = 3x3 zeros, I = 3x3 identity

    // operators
    Matrix33& operator() (int row, int col);
    Matrix3333 operator+ (const Matrix3333& plus);
    Matrix3333 operator- (const Matrix3333& minus);
    Matrix3333 operator* (const Matrix33& multi);
    friend Matrix3333 operator* (const Matrix33& multi1, Matrix3333& multi2);
    Matrix3333 operator* (Scalar multi);
    friend Matrix3333 operator* (Scalar multi1, Matrix3333& multi2);
    Matrix3333 transpose();
    Matrix33 Contract(const Matrix33& multi); // this operator is commutative
    Matrix3333 Contract(Matrix3333& multi);

//protected:

    Matrix33 mat[3][3];
};

class Matrix2222 // 2x2 matrix: each element is a 2x2 matrix
{
public:
    Matrix2222();
    Matrix2222(const Matrix2222& other);
    ~Matrix2222() {}

    void SetZero(); // [0 0; 0 0]; 0 = 2x2 zeros
    void SetIdentity(); //[I 0; 0 I;]; 0 = 2x2 zeros, I = 2x2 identity

    // operators and basic functions
    Matrix22& operator() (int row, int col);
    Matrix2222 operator+ (const Matrix2222& plus);
    Matrix2222 operator- (const Matrix2222& minus);
    Matrix2222 operator* (const Matrix22& multi);
    friend Matrix2222 operator* (const Matrix22& multi1, Matrix2222& multi2);
    Matrix2222 operator* (Scalar multi);
    friend Matrix2222 operator* (Scalar multi1, Matrix2222& multi2);
    Matrix2222 transpose();
    Matrix22 Contract(const Matrix22& multi); // this operator is commutative
    Matrix2222 Contract(Matrix2222& multi);

protected:

    Matrix22 mat[2][2];
};

// dst = src1 \kron src2
void directProduct(Matrix3333& dst, const Matrix33& src1, const Matrix33& src2);
void directProduct(Matrix2222& dst, const Matrix22& src1, const Matrix22& src2);
#endif // TYPES_H
///////////////////////////////////////////////////////////////////////////////
