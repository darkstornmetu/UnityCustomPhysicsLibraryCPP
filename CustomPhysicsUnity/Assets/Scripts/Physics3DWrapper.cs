using System;
using System.Runtime.InteropServices;
using UnityEngine;

[StructLayout(LayoutKind.Sequential)]
public struct Vector3C
{
    public float x, y, z;
}

[StructLayout(LayoutKind.Sequential)]
public struct QuaternionC
{
    public float w, x, y, z;
}

[StructLayout(LayoutKind.Sequential)]
public struct RigidBodyParams
{
    public Vector3C position;
    public QuaternionC rotation;
    public Vector3C halfSize;
    public float mass;
    public float friction;
    public float restitution;
    public float linearDamping;
    public float angularDamping; 
    public float maxAngularSpeed;
}

[StructLayout(LayoutKind.Sequential)]
public struct TransformData
{
    public Vector3C position;
    public QuaternionC rotation;
}

[StructLayout(LayoutKind.Sequential)]
public struct PhysicsGlobalsC
{
    public float sleepLinVel;
    public float sleepAngVel;
    public float sleepTime;
    public float wakeThreshold;

    public float overlapEpsilon;
    public float maxPenClamp;
    public float correctionFactor;
    public float verticalSeparation;
    public float baumgarteBeta;

    public int solverIterations;
}

public static class Physics3DWrapper
{
    private const string _DLL_NAME = "CustomPhysicsLibrary"; 
    // change to your actual DLL name minus .dll

    [DllImport(_DLL_NAME)]
    private static extern void InitializePhysics(int maxBodies);
    
    [DllImport(_DLL_NAME)]
    private static extern void SetPhysicsGlobals(ref PhysicsGlobalsC globals);

    [DllImport(_DLL_NAME)]
    private static extern void ShutdownPhysics();

    [DllImport(_DLL_NAME)]
    private static extern int CreateBody(ref RigidBodyParams param);

    [DllImport(_DLL_NAME)]
    private static extern void GetBodyTransform(int bodyID, out TransformData outT);

    [DllImport(_DLL_NAME)]
    private static extern void UpdatePhysics(float dt, Vector3C gravity);

    // Public Methods for Unity scripts

    public static void Init(int maxBodies)
    {
        InitializePhysics(maxBodies);
    }
    
    public static void SetGlobals(PhysicsGlobalsC g)
    {
        SetPhysicsGlobals(ref g);
    }

    public static void Shutdown()
    {
        ShutdownPhysics();
    }

    public static int AddBody(Vector3 pos, Quaternion rot, Vector3 halfSize, float mass, float friction, float rest,
        float linearDamping, float angularDamping, float maxAngularSpeed)
    {
        RigidBodyParams rp = new RigidBodyParams();
        rp.position.x = pos.x; rp.position.y = pos.y; rp.position.z = pos.z;
        rp.rotation.w = rot.w; rp.rotation.x = rot.x; rp.rotation.y = rot.y; rp.rotation.z = rot.z;
        rp.halfSize.x = halfSize.x; rp.halfSize.y = halfSize.y; rp.halfSize.z = halfSize.z;
        rp.mass = mass;
        rp.friction = friction;
        rp.linearDamping = linearDamping;
        rp.angularDamping = angularDamping;
        rp.maxAngularSpeed = maxAngularSpeed;
        rp.restitution = rest;
        return CreateBody(ref rp);
    }

    public static void GetTransform(int bodyID, out Vector3 pos, out Quaternion rot)
    {
        TransformData td;
        GetBodyTransform(bodyID, out td);
        pos = new Vector3(td.position.x, td.position.y, td.position.z);
        rot = new Quaternion(td.rotation.x, td.rotation.y, td.rotation.z, td.rotation.w);
    }

    public static void StepPhysics(float dt, Vector3 gravity)
    {
        Vector3C g;
        g.x = gravity.x; g.y = gravity.y; g.z = gravity.z;
        UpdatePhysics(dt, g);
    }
}
