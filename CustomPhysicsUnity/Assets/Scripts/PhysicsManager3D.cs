using UnityEngine;

public class PhysicsManager3D : MonoBehaviour
{
    [Header("Max Bodies")]
    public int _maxBodies = 100;

    [Header("Gravity")]
    public Vector3 _gravity = new Vector3(0, -9.81f, 0);

    [Header("PhysicsGlobals")]
    // let’s define them as public fields so you can tweak in Inspector
    public float _sleepLinVel = 0.05f;
    public float _sleepAngVel = 0.05f;
    public float _sleepTime   = 1.0f;
    public float _wakeThreshold = 0.01f;
    public float _overlapEpsilon = 1e-4f;
    public float _maxPenClamp    = 0.1f;
    public float _correctionFactor= 0.01f;
    public float _verticalSeparation = 0.01f;
    public float _baumgarteBeta = 0.2f;
    public int _solverIterations = 30;
    

    void Awake()
    {
        // 1) init
        Physics3DWrapper.Init(_maxBodies);

        // 2) create a C# struct
        PhysicsGlobalsC g;
        g.sleepLinVel       = _sleepLinVel;
        g.sleepAngVel       = _sleepAngVel;
        g.sleepTime         = _sleepTime;
        g.wakeThreshold     = _wakeThreshold;
        g.overlapEpsilon    = _overlapEpsilon;
        g.maxPenClamp       = _maxPenClamp;
        g.correctionFactor  = _correctionFactor;   // minimal direct fix
        g.verticalSeparation= _verticalSeparation;
        g.baumgarteBeta     = _baumgarteBeta;   // velocity-based approach
        g.solverIterations  = _solverIterations;
        Physics3DWrapper.SetGlobals(g);
        // 3) pass to native
        Physics3DWrapper.SetGlobals(g);
    }

    void FixedUpdate()
    {
        Physics3DWrapper.StepPhysics(Time.fixedDeltaTime, _gravity);
    }

    void OnDestroy()
    {
        Physics3DWrapper.Shutdown();
    }
}