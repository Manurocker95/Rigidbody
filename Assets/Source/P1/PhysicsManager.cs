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
/// Basic physics manager capable of simulating a given ISimulable
/// implementation using diverse integration methods: explicit,
/// implicit, Verlet and semi-implicit.
/// </summary>
public class PhysicsManager : MonoBehaviour 
{
	/// <summary>
	/// Default constructor. Zero all. 
	/// </summary>
	public PhysicsManager()
	{
		this.Paused = true;
		this.OneStep = false;
		this.TimeStep = 0.01f;
		this.Gravity = new Vector3 (0.0f, -9.81f, 0.0f);
        this.MethodConstraints = ConstraintMethod.Hard;
	}

	/// <summary>
	/// Constraint method.
	/// </summary>
	public enum ConstraintMethod
	{
		Soft = 0,
		Hard = 1
	};

    #region InEditorVariables

	public bool Paused;
	public bool OneStep;
	public float TimeStep;
	public Vector3 Gravity;
    public float K;
    public float Damping;
    public List<GameObject> SimulableObjects;
    public List<GameObject> Constraints;
    public ConstraintMethod MethodConstraints;

	#endregion

	#region OtherVariables

	private List<ISimulable> m_objs;
    private List<Constraint> m_constraints;
    private int m_numdofs;
    private int m_numcs;

    #endregion

    #region MonoBehaviour

    public void Start()
    {
        int index = 0;

        m_objs = new List<ISimulable>(SimulableObjects.Capacity);

        foreach (GameObject obj in SimulableObjects)
        {
            ISimulable simobj = obj.GetComponent<ISimulable>();

            if (simobj != null)
            {
                m_objs.Add(simobj);

                // Initialize simulable model
                simobj.initialize(index, this);

                // Retrieve pos and vel size
                index += simobj.getNumDof();
            }
        }

        m_numdofs = index;

        index = 0;

        m_constraints = new List<Constraint>(Constraints.Capacity);

        foreach (GameObject obj in Constraints)
        {
            Constraint constraint = obj.GetComponent<Constraint>();

            if (constraint != null)
            {
                m_constraints.Add(constraint);

                // Initialize constraint
                constraint.initialize(index, this);

                // Retrieve constraint size
                index += constraint.getSize();
            }
        }

        m_numcs = index;
    }

    public void Update()
	{
		if (Input.GetKeyUp (KeyCode.O))
			this.OneStep = !this.OneStep;

		if (Input.GetKeyUp (KeyCode.P))
			this.Paused = !this.Paused;
	}

	public void FixedUpdate () 
	{
		if (this.Paused && !this.OneStep)
			return; // Not simulating

		if (this.OneStep) // One!
			this.OneStep = false;

        // Integration method
        switch (MethodConstraints)
        {
            case ConstraintMethod.Hard:
                HardConstraintsStep();
                break;
            case ConstraintMethod.Soft:
            default:
                SoftConstraintsStep();
                break;
        }

        // Update visual elements
        foreach (ISimulable obj in m_objs)
        {
            obj.updateScene();
        }

        foreach (Constraint constraint in m_constraints)
        {
            constraint.updateScene();
        }
    }

    #endregion

    #region OtherMethods

    private void HardConstraintsStep()
    {

        // Apuntes miki + apuntes clase

        // Step de Fuertes:
        // Tenemos que conseguir una matriz enorme formada por
        //
        //  | A    J^t | 
        //  | J    0   |

        // A es la matriz de masas-> 6 * cada componente como en soft
        // J es la matriz jacobiana que rellenamos en Constraint.getConstraintJacobian
        // J se forma 3 * numero de constraints, 6* num rigidbodies
        // J^t  se forma 6* num rigidbodies,  3 * numero de constraints

        // Recordemos que cada rigidbody tiene 6 numdof
        // C0 = 3 * numero de constraints<- Es similar a C en soft

        MatrixXD massMatrix = DenseMatrixXD.CreateIdentity(m_objs.Count * 6);
        MatrixXD jacobian = new DenseMatrixXD(3 * Constraints.Count, 6 * m_objs.Count);
        VectorXD C0 = new DenseVectorXD(3 * m_constraints.Count);
        

        // paso 1: necesitamos settear la jacobiana y C0 para cada restricción
        foreach (Constraint c in m_constraints)
        {
            c.getConstraintVector(C0);
            c.getConstraintJacobian(jacobian);
        }

        // las fuerzas son 1 x 6 * m_objs.Count, igual que las velocidades.
        // Parece que se mete el torque y demás. 3 * objetos no funciona
        VectorXD total_force = new DenseVectorXD(6 * m_objs.Count);
        VectorXD total_velocities = new DenseVectorXD(6 * m_objs.Count);


        // PARA CADA RIGIDBODY

        // Paso 2: Limpiamos las fuerzas y matrices anteriores para evitar sumas de error
        // Paso 3: Establecemos la matriz de masas (Pondremos la inercia como inercia 0 porque si no es null->RigidBody.cs
        // Paso 4: Metemos valores en las velocidades anteriores, ¿Por qué? Porque Al final haremos un A * v = b
        // como en las prácticas anteriores y b es un vector formado por b y  -1 * C0 / TimeStep 
        // siendo b la fórmula (matriz_de_masas * velocidades) + (TimeStep * fuerzas);
        // Paso 5: Añadimos fuerzas y matrices (que las hemos limpiado antes)
        // Paso 6: Metemos las nuevas fuerzas (getForceVector) en el vector de fuerzas

        foreach (RigidBody obj in m_objs)
        {
            obj.clearForcesAndMatrices();
            obj.getMassMatrix(massMatrix);
            obj.getVelocityVector(total_velocities);
            obj.addForcesAndMatrices();
            obj.getForceVector(total_force);
        }


        // Creamos la super matriz que hemos dicho con la de masas, jacobianas y ceros
        // Al crear una MtrixXD se crea con ceros.
        MatrixXD jacobianT = jacobian.Transpose();
        MatrixXD ceroMatrix = new DenseMatrixXD(3 * m_constraints.Count, 3 * m_constraints.Count);
   
        DenseMatrixXD megaMatrix = new DenseMatrixXD(jacobian.RowCount + massMatrix.RowCount, jacobianT.ColumnCount + massMatrix.ColumnCount);

        // la matriz de masas es 0,0 a (m_objs.Count * 6)
        // Una vez acaba esa, va la jacobiana traspuesta
        // Debajo de la misma manera va la jacobiana
        // y en la esquina abajo derecha va la de ceros
        megaMatrix.SetSubMatrix(0, 0, massMatrix);
        megaMatrix.SetSubMatrix(0, 0 + massMatrix.ColumnCount, jacobianT);
        megaMatrix.SetSubMatrix(0 + massMatrix.RowCount, 0, jacobian);
        megaMatrix.SetSubMatrix(0 + massMatrix.RowCount, 0 + massMatrix.ColumnCount, ceroMatrix);

        // M * v = M * v0 + timestep * fuerzas
        // V0 es la velocidad del step anterior
        // en A * v = b, b lo dividiremos como b y b2
        // y los concatenamos
        VectorXD b = (massMatrix * total_velocities) + (TimeStep * total_force);
        VectorXD b2 = -1 * C0 / TimeStep;

        // b total - ambos bs
        VectorXD realB = new DenseVectorXD(b.Count + b2.Count);
        realB.SetSubVector(0, b.Count, b);
        realB.SetSubVector(b.Count, b2.Count, b2);

        // V se forma por velocidades y un vector de lamdas, que debe ser del tamaño de C0 ya que 
        // b2 es escalar * c0
        VectorXD lamdas = new DenseVectorXD(C0.Count);
        // conjunto de velocidades formada por las velocidades y las lamdas
        VectorXD megaV = new DenseVectorXD(total_velocities.Count + lamdas.Count);
        megaV.SetSubVector(0, total_velocities.Count, total_velocities);
        megaV.SetSubVector(total_velocities.Count, lamdas.Count, lamdas);

        // Resolvemos el sistema
        megaV = megaMatrix.Solve(realB);

        // nueva velocidad
        VectorXD newVelocities = megaV.SubVector(0, total_velocities.Count);

        // Establecemos las nuevas posiciones y velocidades
        foreach (RigidBody obj in m_objs)
        {
            obj.setVelocityVector(newVelocities);
            obj.advancePosition();
        }
    }

    private void SoftConstraintsStep()
	{

        // Apuntes Miki + apuntes Marta + apuntes clase

        // Step de Débiles:


        // Podemos integrar cada rigidbody por separado o tener un solo sistema V siendo un vector V,W
        // Teniendo M * v' = F

        // Siendo M:

        //  | m*Identidad   0          | 
        //  | 0             MInercia   |

        // y F = | F |
        //       | T |  


        // A * v = b
        // M * V = M*V0 + timestep * F
        // b = M*V0 + timestep * F


        // La matriz de masas es numero de objetos * 6

        MatrixXD massMatrix = DenseMatrixXD.CreateIdentity (m_objs.Count * 6); 

		VectorXD total_force = new DenseVectorXD (m_objs.Count * 6);
		VectorXD total_velocities = new DenseVectorXD (m_objs.Count * 6);

        // Paso 0: Limpiamos las fuerzas y matrices anteriores para evitar sumas de error - Podríamos meterlo después, but 
        foreach (RigidBody obj in m_objs)
        {
            obj.clearForcesAndMatrices();
        }

        // Paso 1: añadimos las fuerzas del constraint
        // En ellas hacemos Fa y Fb (Si tienen los cuerpos)
        // Y añadimos el torque a los rigidbodies
        foreach (Constraint c in m_constraints)
        {
            c.addForces();
        }

        // Paso 2: Establecemos la parte del objeto en la matriz de masas (Pondremos la inercia como inercia 0 porque si no es null->RigidBody.cs
        // Paso 3: Metemos las velocidades y las fuerzas (y rotaciones)
        // Fuertes deberían ser practicamente iguales pero añadiendo las jacobianas
        foreach (RigidBody obj in m_objs)
        {
            obj.getMassMatrix(massMatrix);
            obj.getVelocityVector(total_velocities);
            obj.addForcesAndMatrices();
            obj.getForceVector(total_force);
        }

        // Resolvemos el sistema 
        VectorXD b = (massMatrix * total_velocities) + (TimeStep * total_force);
        VectorXD newVelocities = massMatrix.Solve(b);
   
        // Y modificamos posiciones
        foreach (RigidBody obj in m_objs)
        {
            obj.setVelocityVector(newVelocities);
            obj.advancePosition();
        }  
    }

    #endregion

}
