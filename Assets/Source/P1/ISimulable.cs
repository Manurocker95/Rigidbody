using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic interface for any simulation model.
/// </summary>
public interface ISimulable
{
	/// <summary>
	/// Initialize the simulable. It receives the start index into the simulation dof vector
	/// </summary>
	void initialize(int index, PhysicsManager manager);

    /// <summary>
    /// Returns the numbef of dofs (i.e., size of the velocity vector)
    /// </summary>
    int getNumDof();

    /// <summary>
    /// Returns the start simulation index
    /// </summary>
    int getSimIndex();

    /// <summary>
    /// Write the force subvector to vfout.
    /// </summary>
    void getForceVector(VectorXD vfout);

    /// <summary>
	/// Writes the velocity subvector to vvout.
    /// </summary>
    void getVelocityVector(VectorXD vvout);

	/// <summary>
	/// Reads the velocity vector from vvin.
	/// </summary>
	void setVelocityVector(VectorXD vvin);
	
	/// <summary>
    /// Advance position using the internal velocity.
	/// </summary>
	void advancePosition();

    /// <summary>
    /// Write the mass submatrix to mmout.
    /// </summary>
    void getMassMatrix(MatrixXD mmout);

	/// <summary>
	/// Updates model forces/Jacobian.
	/// </summary>
	void addForcesAndMatrices();

	/// <summary>
	/// Clears model forces/Jacobian.
	/// </summary>
	void clearForcesAndMatrices();

	/// <summary>
	/// Updates the scene.
	/// </summary>
	void updateScene();


	void getRotationVector (VectorXD vvout);


}
