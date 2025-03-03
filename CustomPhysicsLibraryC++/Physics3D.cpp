#include "Physics3D.h"
#include "RigidBody.h"
#include "PhysicsSolver.h"
#include "Math3D.h"
#include <vector>

static bool g_initialized = false;
static std::vector<RigidBody> g_bodies;

static PhysicsGlobals g_physicsGlobals =
{
    0.05f,
    0.05f,
    1.0f,
    0.01f,
    1e-4f,
    0.1f,
    0.0f,
    0.2f,
    0.001f,
    30
};

extern "C"
{

    PHYSICS3D_API void InitializePhysics(int maxBodies)
    {
        if (!g_initialized) {
            g_bodies.reserve(maxBodies);
            g_initialized = true;
        }
    }

    PHYSICS3D_API void SetPhysicsGlobals(const PhysicsGlobals& globals)
    {
        g_physicsGlobals = globals;
    }

    PHYSICS3D_API void ShutdownPhysics()
    {
        g_bodies.clear();
        g_initialized = false;
    }

    PHYSICS3D_API int CreateBody(const RigidBodyParams& p)
    {
        if (!g_initialized) return -1;
        RigidBody rb;
        rb.position = { p.position.x,p.position.y,p.position.z };
        rb.rotation = { p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z };
        rb.halfSize = { p.halfSize.x, p.halfSize.y, p.halfSize.z };

        rb.mass = p.mass;
        rb.friction = p.friction;
        rb.restitution = p.restitution;
        rb.linearDamping = p.linearDamping;
        rb.angularDamping = p.angularDamping;
        rb.maxAngularSpeed = p.maxAngularSpeed;

        if (rb.mass > 0.f) {
            rb.invMass = 1.f / rb.mass;
            rb.inertiaLocal = PhysicsSolver::ComputeBoxInertia(rb.mass, rb.halfSize);
            rb.invInertiaLocal.x = 1.f / rb.inertiaLocal.x;
            rb.invInertiaLocal.y = 1.f / rb.inertiaLocal.y;
            rb.invInertiaLocal.z = 1.f / rb.inertiaLocal.z;
        }
        else {
            rb.invMass = 0.f;
            rb.inertiaLocal = { 0,0,0 };
            rb.invInertiaLocal = { 0,0,0 };
        }

        rb.linearVelocity = { 0,0,0 };
        rb.angularVelocity = { 0,0,0 };
        rb.isSleeping = false;
        rb.sleepTimer = 0.f;

        g_bodies.push_back(rb);
        return (int)g_bodies.size() - 1;
    }

    PHYSICS3D_API void GetBodyTransform(int bodyID, TransformData* outT)
    {
        if (!g_initialized) return;
        if (bodyID < 0 || bodyID >= (int)g_bodies.size()) return;
        RigidBody& rb = g_bodies[bodyID];
        outT->position = { rb.position.x,rb.position.y,rb.position.z };
        outT->rotation = { rb.rotation.w,rb.rotation.x,rb.rotation.y,rb.rotation.z };
    }

    PHYSICS3D_API void UpdatePhysics(float dt, Math3D::Vector3 gravityP)
    {
        if (!g_initialized) return;

        Math3D::Vector3 gravity = { gravityP.x,gravityP.y,gravityP.z };

        // 1) compute inertia
        for (auto& rb : g_bodies) {
            if (rb.isStatic() || rb.isSleeping) continue;
            PhysicsSolver::ComputeInvInertiaWorld(rb);
        }

        // 2) integrate
        for (auto& rb : g_bodies) {
            if (rb.isStatic() || rb.isSleeping) continue;

            rb.linearVelocity.x += gravity.x * dt;
            rb.linearVelocity.y += gravity.y * dt;
            rb.linearVelocity.z += gravity.z * dt;

            rb.position.x += rb.linearVelocity.x * dt;
            rb.position.y += rb.linearVelocity.y * dt;
            rb.position.z += rb.linearVelocity.z * dt;

            Math3D::Quaternion avQ = { 0, rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z };
            auto dQ = Math3D::MulQ(avQ, rb.rotation);
            dQ.w *= 0.5f * dt; dQ.x *= 0.5f * dt; dQ.y *= 0.5f * dt; dQ.z *= 0.5f * dt;
            rb.rotation.w += dQ.w; rb.rotation.x += dQ.x;
            rb.rotation.y += dQ.y; rb.rotation.z += dQ.z;
            rb.rotation = Math3D::NormalizeQ(rb.rotation);

            float ld = (1.f - rb.linearDamping * dt);
            if (ld < 0.f) ld = 0.f;
            rb.linearVelocity.x *= ld;
            rb.linearVelocity.y *= ld;
            rb.linearVelocity.z *= ld;

            float ad = (1.f - rb.angularDamping * dt);
            if (ad < 0.f) ad = 0.f;
            rb.angularVelocity.x *= ad;
            rb.angularVelocity.y *= ad;
            rb.angularVelocity.z *= ad;

            float asq = Math3D::Dot(rb.angularVelocity, rb.angularVelocity);
            float msq = rb.maxAngularSpeed * rb.maxAngularSpeed;
            if (asq > msq) {
                float sc = rb.maxAngularSpeed / std::sqrt(asq);
                rb.angularVelocity.x *= sc;
                rb.angularVelocity.y *= sc;
                rb.angularVelocity.z *= sc;
            }
        }

        // 3) collisions
        for (int iter = 0; iter < g_physicsGlobals.solverIterations; iter++) {
            for (int i = 0; i < (int)g_bodies.size(); i++) {
                for (int j = i + 1; j < (int)g_bodies.size(); j++) {
                    if (g_bodies[i].isStatic() && g_bodies[j].isStatic()) continue;
                    if (g_bodies[i].isSleeping && g_bodies[j].isSleeping) continue;

                    float overlap = 0.f;
                    Math3D::Vector3 n;
                    if (!PhysicsSolver::SAT_Boxes(g_bodies[i], g_bodies[j], overlap, n, g_physicsGlobals)) continue;
                    if (overlap > 0.f) {
                        auto oldA = g_bodies[i].linearVelocity;
                        auto oldB = g_bodies[j].linearVelocity;

                        ContactManifold mf; mf.contactCount = 0;
                        PhysicsSolver::BuildContactManifold(g_bodies[i], g_bodies[j], overlap, n, mf, g_physicsGlobals);
                        if (mf.contactCount < 1) continue;

                        PhysicsSolver::ResolveManifold(g_bodies[i], g_bodies[j], mf, g_physicsGlobals);

                        float dvA = Math3D::Length(Math3D::Sub(g_bodies[i].linearVelocity, oldA));
                        float dvB = Math3D::Length(Math3D::Sub(g_bodies[j].linearVelocity, oldB));
                        float dv = std::max(dvA, dvB);
                        if (dv > g_physicsGlobals.wakeThreshold) {
                            if (!g_bodies[i].isStatic()) {
                                g_bodies[i].isSleeping = false;
                                g_bodies[i].sleepTimer = 0.f;
                            }
                            if (!g_bodies[j].isStatic()) {
                                g_bodies[j].isSleeping = false;
                                g_bodies[j].sleepTimer = 0.f;
                            }
                        }
                    }
                }
            }
        }

        // 4) sleep check
        for (auto& rb : g_bodies) {
            if (rb.isStatic()) continue;
            if (rb.isSleeping) continue;
            float lin = Math3D::Length(rb.linearVelocity);
            float ang = Math3D::Length(rb.angularVelocity);
            if (lin < g_physicsGlobals.sleepLinVel && ang < g_physicsGlobals.sleepAngVel) {
                rb.sleepTimer += dt;
                if (rb.sleepTimer >= g_physicsGlobals.sleepTime) {
                    rb.isSleeping = true;
                    rb.linearVelocity = { 0,0,0 };
                    rb.angularVelocity = { 0,0,0 };
                }
            }
            else {
                rb.sleepTimer = 0.f;
            }
        }
    }

} // extern "C"
