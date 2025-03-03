#pragma once

#ifdef CUSTOMPHYSICSLIBRARY_EXPORTS
#define PHYSICS3D_API __declspec(dllexport)
#else
#define PHYSICS3D_API __declspec(dllimport)
#endif
#include "Math3D.h"

extern "C"
{
    struct RigidBodyParams
    {
	    Math3D::Vector3 position;
	    Math3D::Quaternion rotation;
	    Math3D::Vector3 halfSize;
        float mass;
        float friction;
        float restitution;
        float linearDamping;
        float angularDamping;
        float maxAngularSpeed;
    };

    struct PhysicsGlobals
    {
        float sleepLinVel;
        float sleepAngVel;
        float sleepTime;
        float wakeThreshold;
        float overlapEpsilon;
        float maxPenClamp;
        float correctionFactor; // minimal direct position fix
        float baumgarteBeta;    // velocity-based penetration correction
        float verticalSeparation;
        int solverIterations;
    };

    struct TransformData
    {
	    Math3D::Vector3 position;
	    Math3D::Quaternion rotation;
    };

    // extern "C" API
    PHYSICS3D_API void InitializePhysics(int maxBodies);
    PHYSICS3D_API void SetPhysicsGlobals(const PhysicsGlobals& globals);
    PHYSICS3D_API void ShutdownPhysics();
    PHYSICS3D_API int  CreateBody(const RigidBodyParams& params);
    PHYSICS3D_API void GetBodyTransform(int bodyID, TransformData* outTransform);
    PHYSICS3D_API void UpdatePhysics(float deltaTime, Math3D::Vector3 gravity);
}
