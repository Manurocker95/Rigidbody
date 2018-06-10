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
/// Basic constraint which needs to be dropped on a sphere.
/// We only implement spherical joints. Other constraints would require a generalization of the constraint interface.
/// </summary>
public class Constraint : MonoBehaviour
{
    /// <summary>
    /// Default constructor. All zero. 
    /// </summary>
    public Constraint()
    {
        this.m_manager = null;
    }

    #region EditorVariables

    public RigidBody BodyA;
    public RigidBody BodyB;

    #endregion

    #region OtherVariables

    private PhysicsManager m_manager;
    private int m_index; //Index of the constraint into the global vector of constraints
    public Vector3 m_pA; //Constraint point in the local reference frame of bodyA
    public Vector3 m_pB; //Constraint point in the local reference frame of bodyB
    // If BodyA or BodyB is not defined, m_pA or m_pB stores the global coordinates of the constraint point
	public float L0A;
	public float L0B;

    #endregion

    #region MonoBehaviour

    // Nothing to do here

    #endregion

    #region OtherMethods

    public void initialize(int index, PhysicsManager manager)
    {
        m_manager = manager;

        m_index = index;

        // Get the center of the sphere as the constraint point and transform it to the local frames of the bodies
        Transform xform = this.GetComponent<Transform>();

        if (xform != null)
        {
            Vector3 p = xform.localPosition;

            if (BodyA != null)
            {
                m_pA = BodyA.PointGlobalToLocal(p);

            }
            else
            {
                m_pA = p;
            }

            if (BodyB != null)
            {
                m_pB = BodyB.PointGlobalToLocal(p);

            }
            else
            {
                m_pB = p;
            }
        }
    }

    public int getSize()
    {
        return 3;
    }

    public void updateScene()
    {
        // Apply the average position to the mesh
        this.GetComponent<Transform>().position =
            0.5f * ((BodyA ? BodyA.PointLocalToGlobal(m_pA) : m_pA) + (BodyB ? BodyB.PointLocalToGlobal(m_pB) : m_pB));
    }

    public void addForces()
    {
        // TO BE COMPLETED: ADD SOFT CONSTRAINT FORCES TO THE RIGID BODIES

        // RESTRICCIONES DEBILES:

        // por sólidos articulados (3 Restricciones por ser en 3D):
        // C = Ra*ra + Xa - Rb*rb - Xb = 0

        // Los rigidBodies pueden ser null si la constraint no está asociada a uno
        // Por lo que de forma genérica deberían ser cero y así, si son null,
        // no se tienen en cuenta.

        // referencia en local de la posición del rigidbody
        Vector3 ra = m_pA;
        Vector3 rb = m_pB;

        // Posición del rigidbody
        Vector3 Xa = Vector3.zero;
        Vector3 Xb = Vector3.zero;

        //Rotaciones de los rigidbody
        Quaternion Ra = Quaternion.identity;
        Quaternion Rb = Quaternion.identity;

        // Si tenemos el cuerpo 1 asociado - Cogemos su posición y rotación para calcular C
        if (BodyA != null)
        {
            Xa = BodyA.Position;
            Ra = BodyA.Rotation;
        }

        // Si tenemos el cuerpo 2 asociado - Cogemos su posición y rotación para calcular C
        if (BodyB != null)
        {
            Xb = BodyB.Position;
            Rb = BodyB.Rotation;
        }

        // Sin cuerpos, Xb/Xa y Rb/Ra son 0
        Vector3 C = (Ra * ra + Xa - Rb * rb - Xb);

        // Restricciones débiles: 
        // Fuerza = -Gradiente de la energía

        if (BodyA != null)
        {
            // Grad de ||Xa-xb|| -> 2 * (xa-xb / | xa-xb |)
            float XaXb = (Xa - Xb).magnitude;

            // Gradiente
            float C_Grad_Xa = 2f * (XaXb / Mathf.Abs(XaXb));

            // Fa = - k * C Grad Xa * C
            Vector3 Fa = -m_manager.K * C_Grad_Xa * C;
            BodyA.AddToForce(Fa);
            BodyA.AddToTorque (Vector3.Cross(BodyA.Rotation * m_pA, Fa));
        }

        if (BodyB != null)
        {
            // Grad de ||Xb-xa|| -> 2 * (xb-xa / | xb-xa |)
            float XbXa = (Xb - Xa).magnitude;

            // Gradiente
            float C_Grad_Xb = 2f*(-XbXa / Mathf.Abs(XbXa));

            // Fb = - k * C Grad Xb * C
            Vector3 Fb = -m_manager.K * C_Grad_Xb * C;
            BodyB.AddToForce(Fb);
            BodyB.AddToTorque(Vector3.Cross(BodyB.Rotation * m_pB, Fb));
        }

    }

    public void getConstraintVector(VectorXD C0)
    {
        // TO BE COMPLETED: WRITE CONSTRAINT VALUES TO C0

        // Apuntes del día 5 :

        // C0 = SoftConstraint C

        // referencia en local de la posición del rigidbody
        Vector3 ra = m_pA;
        Vector3 rb = m_pB;

        // Posición del rigidbody
        Vector3 Xa = Vector3.zero;
        Vector3 Xb = Vector3.zero;

        //Rotaciones de los rigidbody
        Quaternion Ra = Quaternion.identity;
        Quaternion Rb = Quaternion.identity;

        // Si tenemos el cuerpo 1 asociado - Cogemos su posición y rotación para calcular C
        if (BodyA != null)
        {
            Xa = BodyA.Position;
            Ra = BodyA.Rotation;
        }

        // Si tenemos el cuerpo 2 asociado - Cogemos su posición y rotación para calcular C
        if (BodyB != null)
        {
            Xb = BodyB.Position;
            Rb = BodyB.Rotation;
        }

        // Sin cuerpos, Xb/Xa y Rb/Ra son 0
        Vector3 C = (Ra * ra + Xa - Rb * rb - Xb);

        // Creamos C0
        C0.SetSubVector(m_index, 3, Utils.ToVectorXD(C));
    }

    public void getConstraintJacobian(MatrixXD J)
    {
        // TO BE COMPLETED: WRITE CONSTRAINT JACOBIANS TO J

        // Apuntes del día 5 :
        // La jacobiana es un conjunto de V y W formada como
        // V,W,V,W... siendo (V,W) un rigidbody
 
        // Wa y Wb son matrices 3X3 formadas por (-Ra * ra) * <- Skewed
        // Va y Vb son matrices identidad 3x3
       
        // Metemos (si hay cuerpo) -> Identidad y después Wa/Wb 

        // un truco es meter la identidad en B como negativa y así ya es opuesto a Va
        // Va = -Vb

        MatrixXD IdentityA;
        MatrixXD IdentityB;
        MatrixXD Wa, Wb;
        

        if (BodyA != null)
        {
            Wa = Utils.Skew(-(BodyA.Rotation * m_pA));
            IdentityA = DenseMatrixXD.CreateIdentity(3);
            J.SetSubMatrix(m_index, BodyA.getSimIndex(), IdentityA);
            J.SetSubMatrix(m_index, BodyA.getSimIndex() + 3, Wa);
        }

        if (BodyB != null)
        {
            Wb = Utils.Skew((BodyB.Rotation * m_pB));
            IdentityB = -DenseMatrixXD.CreateIdentity(3);
            J.SetSubMatrix(m_index, BodyB.getSimIndex(), IdentityB);
            J.SetSubMatrix(m_index, BodyB.getSimIndex() + 3, Wb);
        }
    }

    #endregion

}
