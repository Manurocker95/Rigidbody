/*=================================================================================== *
 *                  Práctica 2 - Manuel Rodriguez Matesanz                            *
 *          URJC - Máster Informática Gráfica, Juegos y Realidad Virtual              *
 * ================================================================================== */

using UnityEngine;
using System.Collections;
using System.Collections.Generic;

using VectorXD = MathNet.Numerics.LinearAlgebra.Vector<double>;
using MatrixXD = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using DenseVectorXD = MathNet.Numerics.LinearAlgebra.Double.DenseVector;
using DenseMatrixXD = MathNet.Numerics.LinearAlgebra.Double.DenseMatrix;

/// <summary>
/// Basic rigid-body model component which can be dropped onto
/// a game object.
/// </summary>
public class RigidBody : MonoBehaviour, ISimulable
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public RigidBody()
    {
        this.m_manager = null;
    }

    #region EditorVariables

    public float mass;
    #endregion

    #region OtherVariables

    protected PhysicsManager m_manager;
    protected int m_index;

    protected Vector3 m_pos;
    protected Quaternion m_rot;
    protected Vector3 m_vel;
    protected Vector3 m_omega;

    protected Vector3 m_force;
    protected Vector3 m_torque;

    protected MatrixXD m_inertia0;
    protected MatrixXD m_inertia;

    /// <summary>
    /// Properties varias. Como las variables son protected no se pueden acceder desde otras clases. 
    /// </summary>
    public PhysicsManager PManager { get { return m_manager; } }
    public Vector3 Position { get { return m_pos; } set { m_pos = value; } }
    public Quaternion Rotation { get { return m_rot; } set { m_rot = value; } }
    public Vector3 Velocity { get { return m_vel; } set { m_vel = value; } }
    public Vector3 Omega { get { return m_omega; } set { m_omega = value; } }
    public Vector3 Force { get { return m_force; } set { m_force = value; } }
    public Vector3 Torque { get { return m_torque; } set { m_torque = value; } }

    #endregion

    #region MonoBehaviour

    // Nothing to do here

    #endregion

    #region ISimulable

    public void initialize(int index, PhysicsManager manager)
    {
        m_manager = manager;

        // Initialize indices
        m_index = index;

        // Initialize inertia. We assume that the object is connected to a Cube mesh.
        Transform xform = this.GetComponent<Transform>();
        if (xform == null)
        {
            System.Console.WriteLine("[ERROR] Couldn't find any transform connected to the rigid body");
        }
        else
        {
            System.Console.WriteLine("[TRACE] Succesfully found transform connected to the rigid body");
        }

        if (xform != null)
        {
            this.m_inertia0 = DenseMatrixXD.CreateIdentity(3);
            double[] vals;
            vals = new double[3];
            vals[0] = 1.0f / 12.0f * mass * (xform.localScale.y * xform.localScale.y + xform.localScale.z * xform.localScale.z);
            vals[1] = 1.0f / 12.0f * mass * (xform.localScale.x * xform.localScale.x + xform.localScale.z * xform.localScale.z);
            vals[2] = 1.0f / 12.0f * mass * (xform.localScale.x * xform.localScale.x + xform.localScale.y * xform.localScale.y);
            this.m_inertia0.SetDiagonal(vals);
            this.m_inertia = m_inertia0;
        }

        // Initialize kinematics
        this.m_pos = xform.position;
        this.m_rot = xform.rotation;
        this.m_vel = Vector3.zero;
        this.m_omega = Vector3.zero;

    }

    public int getNumDof()
    {
        return 6;
    }

    public int getSimIndex()
    {
        return this.m_index;
    }

    public void AddToForce(Vector3 _force)
    {
        this.m_force += _force;
    }

    public void AddToTorque(Vector3 _torque)
    {
        this.m_torque += _torque;
    }

    public void getForceVector(VectorXD vfout)
    {
        vfout.SetSubVector(m_index, 3, Utils.ToVectorXD(this.m_force));
        vfout.SetSubVector(m_index + 3, 3, Utils.ToVectorXD(this.m_torque));
    }

    public void getVelocityVector(VectorXD vvout)
    {
        vvout.SetSubVector(m_index, 3, Utils.ToVectorXD(this.m_vel));
        vvout.SetSubVector(m_index + 3, 3, Utils.ToVectorXD(this.m_omega));
    }

    public void setVelocityVector(VectorXD vvin)
    {
        this.m_vel = Utils.ToVector3(vvin.SubVector(this.m_index, 3));
        this.m_omega = Utils.ToVector3(vvin.SubVector(this.m_index + 3, 3));
    }

	public void getRotationVector(VectorXD vvout)
	{
		vvout.SetSubVector(m_index, 3, Utils.ToVectorXD(this.m_rot));
	}

    public void advancePosition()
    {
        // Compute position change
        this.m_pos += m_manager.TimeStep * this.m_vel;

        // Computer linearized rotation change
        Quaternion qomega = new Quaternion(this.m_omega.x, this.m_omega.y, this.m_omega.z, 0.0f);
        qomega *= this.m_rot;
        this.m_rot = Utils.ToQuaternion(Utils.ToVectorXD(this.m_rot) + m_manager.TimeStep * 0.5f * Utils.ToVectorXD(qomega));

        // Project rotation
        this.m_rot = Utils.NormalizeQuaternion(this.m_rot);
    }

    public void getMassMatrix(MatrixXD mmout)
    {
        mmout.SetSubMatrix(m_index, m_index, this.mass * DenseMatrixXD.CreateIdentity(3));
        mmout.SetSubMatrix(m_index + 3, m_index + 3, this.m_inertia);

    }

    public void clearForcesAndMatrices()
    {
        this.m_force = Vector3.zero;
        this.m_torque = Vector3.zero;
    }

    public void updateScene()
    {
        // Apply the position and rotation to the mesh
        Transform xform = this.GetComponent<Transform>();
        xform.position = this.m_pos;
        xform.rotation = this.m_rot;
    }

    public void addForcesAndMatrices()
    {
        // Apuntes día 5:

        // Lo primero que debemos añadir es la fuerza de la gravedad (peso) y así cae directamente
        // Es decir: peso = gravedad * masa
        // Después añadimos Añadimos la fuerza de amortiguamiento como -Constante de Damping * velocidad


        // Compute inertia
        this.m_inertia = Utils.WarpMatrix(m_rot, m_inertia0);

        // TO BE COMPLETED: ADD FORCE AND TORQUE LOCAL TO THE RIGID BODY

        // añadimos la fuerza de peso
        Vector3 weight = m_manager.Gravity * mass;
        // añadimos la fuerza de amortiguamiento
        Vector3 damping = -1f*m_manager.Damping*m_vel;
        // la sumamos para tener toda la fuerza
		m_force += weight + damping;
	
    }

    #endregion

    #region OtherMethods

    public Vector3 PointGlobalToLocal(Vector3 p)
    {
        return Quaternion.Inverse(this.m_rot) * (p - this.m_pos);
    }

    public Vector3 PointLocalToGlobal(Vector3 p)
    {
        return this.m_pos + this.m_rot * p;
    }

    public Vector3 VecLocalToGlobal(Vector3 v)
    {
        return this.m_rot * v;
    }

    #endregion

}
