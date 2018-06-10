using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic class for common operations.
/// </summary>
public class Utils
{
    /// <summary>
    /// Convert a Vector3 to VectorXD
    /// </summary>
    public static VectorXD ToVectorXD(Vector3 vin)
    {
        VectorXD vout = new DenseVectorXD(3);
        vout[0] = vin[0];
        vout[1] = vin[1];
        vout[2] = vin[2];

        return vout;
    }

    /// <summary>
    /// Convert a Quaternion to VectorXD
    /// </summary>
    public static VectorXD ToVectorXD(Quaternion vin)
    {
        VectorXD vout = new DenseVectorXD(4);
        vout[0] = vin.x;
        vout[1] = vin.y;
        vout[2] = vin.z;
        vout[3] = vin.w;

        return vout;
    }

    /// <summary>
    /// Convert a VectorXD to Vector3
    /// </summary>
    public static Vector3 ToVector3(VectorXD vin)
    {
        Vector3 vout = new Vector3();
        vout.Set((float) vin[0], 
                 (float) vin[1], 
                 (float) vin[2]);
        return vout;
    }

    /// <summary>
    /// Convert a VectorXD to Quaternion
    /// </summary>
    public static Quaternion ToQuaternion(VectorXD vin)
    {
        Quaternion qout = new Quaternion();
        qout.Set((float)vin[0],
                 (float)vin[1],
                 (float)vin[2],
                 (float)vin[3]);
        return qout;
    }

    /// <summary>
    /// Return a normalized quaternion
    /// </summary>
    public static Quaternion NormalizeQuaternion(Quaternion qin)
    {
        Quaternion qout = new Quaternion();
        float norm = Mathf.Sqrt(qin.x * qin.x + qin.y * qin.y + qin.z * qin.z + qin.w * qin.w);
        qout.Set((float)qin[0] / norm,
                 (float)qin[1] / norm,
                 (float)qin[2] / norm,
                 (float)qin[3] / norm);
        return qout;
    }

    /// <summary>
    /// Add the specified 3D vector to the concatenaded vector.
    /// </summary>
    public static void Add3D(int idx, VectorXD v3, VectorXD v)
	{
		v.SetSubVector (3 * idx, 3, v3 + v.SubVector(3 * idx, 3));
	}

	/// <summary>
	/// Add the specified 3x3 matrix to the specified matrix.
	/// </summary>
	public static void Add3D(int idx0, int idx1, MatrixXD m3, MatrixXD m)
	{
		m.SetSubMatrix (3 * idx0, 3, 3 * idx1, 3, m3 + m.SubMatrix (3 * idx0, 3, 3 * idx1, 3));
	}

    /// <summary>
    /// Return the skew-symmetric matrix corresponding to the cross product
    /// </summary>
    public static MatrixXD Skew(Vector3 v)
    {
        MatrixXD mout = new DenseMatrixXD(3, 3);
        mout[0, 1] = -v.z;
        mout[0, 2] = v.y;
        mout[1, 0] = v.z;
        mout[1, 2] = -v.x;
        mout[2, 0] = -v.y;
        mout[2, 1] = v.x;
        return mout;
    }

    /// <summary>
    /// Warp a matrix M as R * M * R^T
    /// </summary>
    public static MatrixXD WarpMatrix(Quaternion R, MatrixXD M)
    {
        MatrixXD mout = new DenseMatrixXD(3, 3);
        mout.SetRow(0, ToVectorXD(R * ToVector3(M.Row(0))));
        mout.SetRow(1, ToVectorXD(R * ToVector3(M.Row(1))));
        mout.SetRow(2, ToVectorXD(R * ToVector3(M.Row(2))));
        mout.SetColumn(0, ToVectorXD(R * ToVector3(mout.Column(0))));
        mout.SetColumn(1, ToVectorXD(R * ToVector3(mout.Column(1))));
        mout.SetColumn(2, ToVectorXD(R * ToVector3(mout.Column(2))));
        return mout;
    }

}
