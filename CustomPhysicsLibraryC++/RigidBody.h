#pragma once
#include "Math3D.h"

struct RigidBody
{
    Math3D::Vector3 position;
    Math3D::Quaternion rotation;
    Math3D::Vector3 halfSize;

    float mass;
    float invMass;
    float friction;
    float restitution;
    float linearDamping;
    float angularDamping;
    float maxAngularSpeed;

    Math3D::Vector3 inertiaLocal;
    Math3D::Vector3 invInertiaLocal;
    Math3D::Matrix3 invInertiaWorld;

    Math3D::Vector3 linearVelocity;
    Math3D::Vector3 angularVelocity;

    bool isSleeping;
    float sleepTimer;

    bool isStatic() const { return (mass <= 0.f); }
};
