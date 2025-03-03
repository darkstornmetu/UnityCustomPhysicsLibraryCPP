#pragma once
#include "Math3D.h"
#include "RigidBody.h"
#include "Physics3D.h"

struct ContactPoint
{
    Math3D::Vector3 position;
    float penetration;
};

struct ContactManifold
{
    Math3D::Vector3 normal;
    ContactPoint points[4];
    int contactCount;
};


namespace PhysicsSolver
{
    Math3D::Vector3 ComputeBoxInertia(float m, const Math3D::Vector3& hs);
    void ComputeInvInertiaWorld(RigidBody& rb);

    bool SAT_Boxes(const RigidBody& A, const RigidBody& B, float& overlap, Math3D::Vector3& normal,
        const PhysicsGlobals& globals);

    void BuildContactManifold(const RigidBody& A, const RigidBody& B, float overlap, const Math3D::Vector3& n,
        ContactManifold& mf, const PhysicsGlobals& globals);

    void ResolveManifold(RigidBody& A, RigidBody& B, const ContactManifold& mf, const PhysicsGlobals& globals);
}
