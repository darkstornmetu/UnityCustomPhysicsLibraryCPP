#include "PhysicsSolver.h"
#include <algorithm>
#include <cmath>

using namespace Math3D;

namespace
{
    void ApplyImpulse(RigidBody& rb, const Vector3& imp, const Vector3& r)
    {
        if (rb.isStatic()) return;
        rb.linearVelocity = Sub(rb.linearVelocity, Scale(imp, rb.invMass));
        auto c = Cross(r, imp);
        auto w = MulMatVec(rb.invInertiaWorld, c);
        rb.angularVelocity = Sub(rb.angularVelocity, w);
    }

    float InvInertiaWorld(const RigidBody& rb, const Vector3& r, const Vector3& axis)
    {
        if (rb.isStatic()) return 0.f;
        auto c = Cross(r, axis);
        auto iW = MulMatVec(rb.invInertiaWorld, c);
        auto cross2 = Cross(iW, r);
        return Dot(axis, cross2);
    }

    bool TestAxis(const Vector3& axis, const RigidBody& A, const RigidBody& B,
        float& minOverlap, Vector3& bestNormal, const PhysicsGlobals& g)
    {
        float len = Length(axis);
        if (len < 1e-6f) return true;
        Vector3 n = Scale(axis, 1.f / len);

        float dist = Dot(Sub(B.position, A.position), n);

        // vertical separation check
        if (std::fabs(n.x) < 0.1f && std::fabs(n.z) < 0.1f && n.y > 0.9f) {
            if (dist > g.verticalSeparation) {
                return true;
            }
        }

        auto Project = [&](const RigidBody& rb, const Vector3& norm) {
            Vector3 Ax[3] = { GetAxisX(rb.rotation), GetAxisY(rb.rotation), GetAxisZ(rb.rotation) };
            float r = 0.f;
            for (int i = 0; i < 3; i++) {
                float val = std::fabs(Dot(Ax[i], norm));
                float e = (i == 0 ? rb.halfSize.x : (i == 1 ? rb.halfSize.y : rb.halfSize.z));
                r += val * e;
            }
            return r;
            };
        float rA = Project(A, n), rB = Project(B, n);
        float pen = rA + rB - std::fabs(dist);

        if (pen < g.overlapEpsilon) pen = 0.f;
        if (pen < 0.f) return false;

        if (pen < minOverlap) {
            minOverlap = pen;
            float sign = (dist < 0.f) ? -1.f : 1.f;
            bestNormal = Scale(n, sign);
        }
        return true;
    }

    void ClipPolygon(const std::vector<Vector3>& inVerts, const Vector3& normal,
        float planeDistVal, std::vector<Vector3>& outVerts)
    {
        outVerts.clear();
        int c = (int)inVerts.size();
        for (int i = 0; i < c; i++) {
            int j = (i + 1) % c;
            auto& v1 = inVerts[i];
            auto& v2 = inVerts[j];
            float d1 = Dot(v1, normal) - planeDistVal;
            float d2 = Dot(v2, normal) - planeDistVal;
            bool in1 = (d1 >= 0.f), in2 = (d2 >= 0.f);
            if (in1) outVerts.push_back(v1);
            if (in1 != in2) {
                float alpha = d1 / (d1 - d2);
                Vector3 vv = {
                    v1.x + alpha * (v2.x - v1.x),
                    v1.y + alpha * (v2.y - v1.y),
                    v1.z + alpha * (v2.z - v1.z)
                };
                outVerts.push_back(vv);
            }
        }
    }

    int FindBestFace(const RigidBody& rb, const Vector3& n)
    {
        Vector3 norms[6] = {
            GetAxisX(rb.rotation),
            Scale(GetAxisX(rb.rotation),-1.f),
            GetAxisY(rb.rotation),
            Scale(GetAxisY(rb.rotation),-1.f),
            GetAxisZ(rb.rotation),
            Scale(GetAxisZ(rb.rotation),-1.f)
        };
        int best = 0;
        float bestDot = -1e9f;
        for (int i = 0; i < 6; i++) {
            float d = Dot(norms[i], n);
            if (d > bestDot) {
                bestDot = d;
                best = i;
            }
        }
        return best;
    }

    void GetBoxFaceVerts(const RigidBody& rb, int faceIdx, std::vector<Vector3>& out)
    {
        out.clear();
        auto c = rb.position;
        auto x = GetAxisX(rb.rotation);
        auto y = GetAxisY(rb.rotation);
        auto z = GetAxisZ(rb.rotation);
        float hx = rb.halfSize.x, hy = rb.halfSize.y, hz = rb.halfSize.z;

        Vector3 faceCenter, faceNorm;
        switch (faceIdx) {
        case 0: faceCenter = Add(c, Scale(x, hx)); faceNorm = x; break;
        case 1: faceCenter = Sub(c, Scale(x, hx)); faceNorm = Scale(x, -1.f); break;
        case 2: faceCenter = Add(c, Scale(y, hy)); faceNorm = y; break;
        case 3: faceCenter = Sub(c, Scale(y, hy)); faceNorm = Scale(y, -1.f); break;
        case 4: faceCenter = Add(c, Scale(z, hz)); faceNorm = z; break;
        default: faceCenter = Sub(c, Scale(z, hz)); faceNorm = Scale(z, -1.f); break;
        }

        Vector3 up, rt;
        if (std::fabs(Dot(faceNorm, y)) < 0.99f) up = y; else up = x;
        rt = Cross(faceNorm, up);
        up = Cross(rt, faceNorm);
        rt = Normalize(rt); up = Normalize(up);

        float ex, ey;
        if (faceIdx < 2) { ex = hy; ey = hz; }
        else if (faceIdx < 4) { ex = hx; ey = hz; }
        else { ex = hx; ey = hy; }

        Vector3 v0 = Add(Add(faceCenter, Scale(rt, ex)), Scale(up, ey));
        Vector3 v1 = Add(Sub(faceCenter, Scale(rt, ex)), Scale(up, ey));
        Vector3 v2 = Sub(Sub(faceCenter, Scale(rt, ex)), Scale(up, ey));
        Vector3 v3 = Sub(Add(faceCenter, Scale(rt, ex)), Scale(up, ey));
        out.push_back(v0); out.push_back(v1); out.push_back(v2); out.push_back(v3);
    }
}

namespace PhysicsSolver
{

    Math3D::Vector3 ComputeBoxInertia(float m, const Math3D::Vector3& hs)
    {
        float x = hs.x * 2.f, y = hs.y * 2.f, z = hs.z * 2.f;
        Math3D::Vector3 I;
        I.x = (1.f / 12.f) * m * (y * y + z * z);
        I.y = (1.f / 12.f) * m * (x * x + z * z);
        I.z = (1.f / 12.f) * m * (x * x + y * y);
        return I;
    }

    void ComputeInvInertiaWorld(RigidBody& rb)
    {
        if (rb.isStatic()) {
            Math3D::Matrix3 I;
            for (int i = 0; i < 9; i++) I.m[i] = (i % 4 == 0) ? 1.f : 0.f;
            rb.invInertiaWorld = I;
            return;
        }
        auto x = GetAxisX(rb.rotation);
        auto y = GetAxisY(rb.rotation);
        auto z = GetAxisZ(rb.rotation);

        float ixx = rb.invInertiaLocal.x, iyy = rb.invInertiaLocal.y, izz = rb.invInertiaLocal.z;

        float xx = x.x * x.x * ixx + x.y * x.y * iyy + x.z * x.z * izz;
        float xy = x.x * y.x * ixx + x.y * y.y * iyy + x.z * y.z * izz;
        float xz = x.x * z.x * ixx + x.y * z.y * iyy + x.z * z.z * izz;
        float yx = y.x * x.x * ixx + y.y * x.y * iyy + y.z * x.z * izz;
        float yy = y.x * y.x * ixx + y.y * y.y * iyy + y.z * y.z * izz;
        float yz = y.x * z.x * ixx + y.y * z.y * iyy + y.z * z.z * izz;
        float zx = z.x * x.x * ixx + z.y * x.y * iyy + z.z * x.z * izz;
        float zy = z.x * y.x * ixx + z.y * y.y * iyy + z.z * y.z * izz;
        float zz = z.x * z.x * ixx + z.y * z.y * iyy + z.z * z.z * izz;

        Math3D::Matrix3 M;
        M.m[0] = xx; M.m[1] = xy; M.m[2] = xz;
        M.m[3] = yx; M.m[4] = yy; M.m[5] = yz;
        M.m[6] = zx; M.m[7] = zy; M.m[8] = zz;
        Math3D::InvertMatrix3(M);
        rb.invInertiaWorld = M;
    }

    bool SAT_Boxes(const RigidBody& A, const RigidBody& B, float& overlap, Math3D::Vector3& normal, const PhysicsGlobals& g)
    {
        overlap = 1e9f;
        normal = { 0,0,0 };

        Vector3 Ax[3] = { GetAxisX(A.rotation), GetAxisY(A.rotation), GetAxisZ(A.rotation) };
        Vector3 Bx[3] = { GetAxisX(B.rotation), GetAxisY(B.rotation), GetAxisZ(B.rotation) };

        // for each axis
        for (int i = 0; i < 3; i++) {
            if (!TestAxis(Ax[i], A, B, overlap, normal, g)) return false;
            if (!TestAxis(Bx[i], A, B, overlap, normal, g)) return false;
        }
        // cross
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                auto c = Cross(Ax[i], Bx[j]);
                if (!TestAxis(c, A, B, overlap, normal, g)) return false;
            }
        }
        auto d = Sub(B.position, A.position);
        if (Dot(d, normal) < 0.f) normal = Scale(normal, -1.f);
        return true;
    }

    void BuildContactManifold(const RigidBody& A, const RigidBody& B, float overlap, const Vector3& n,
        ContactManifold& mf, const PhysicsGlobals& g)
    {
        mf.normal = n;
        mf.contactCount = 0;
        if (overlap > g.maxPenClamp) {
            overlap = g.maxPenClamp;
        }
        // pick reference face
        float dotAx = std::fabs(Dot(GetAxisX(A.rotation), n));
        float dotAy = std::fabs(Dot(GetAxisY(A.rotation), n));
        float dotAz = std::fabs(Dot(GetAxisZ(A.rotation), n));
        float maxA = std::max(dotAx, std::max(dotAy, dotAz));

        float dotBx = std::fabs(Dot(GetAxisX(B.rotation), n));
        float dotBy = std::fabs(Dot(GetAxisY(B.rotation), n));
        float dotBz = std::fabs(Dot(GetAxisZ(B.rotation), n));
        float maxB = std::max(dotBx, std::max(dotBy, dotBz));

        bool Aref = (maxA >= maxB);
        const RigidBody& Ref = Aref ? A : B;
        const RigidBody& Inc = Aref ? B : A;
        float sign = Aref ? 1.f : -1.f;

        int refFace = FindBestFace(Ref, Scale(n, sign));
        std::vector<Vector3> refVerts;
        GetBoxFaceVerts(Ref, refFace, refVerts);

        auto refNorm = (Aref ? n : Scale(n, -1.f));

        std::vector<Vector3> planesN;
        std::vector<float> planesD;
        planesN.reserve(4); planesD.reserve(4);

        Vector3 faceCenter = { 0,0,0 };
        for (auto& v : refVerts) { faceCenter = Add(faceCenter, v); }
        faceCenter = Scale(faceCenter, 1.f / refVerts.size());

        for (int i = 0; i < (int)refVerts.size(); i++) {
            int j = (i + 1) % refVerts.size();
            auto edge = Sub(refVerts[j], refVerts[i]);
            auto faceN = Normalize(Cross(edge, refNorm));
            float d = Dot(refVerts[i], faceN);
            planesN.push_back(faceN);
            planesD.push_back(d);
        }

        int incFace = FindBestFace(Inc, Scale(n, -sign));
        std::vector<Vector3> incVerts;
        GetBoxFaceVerts(Inc, incFace, incVerts);

        // clip
        std::vector<Vector3> clipPoly = incVerts;
        for (auto i = 0; i < (int)planesN.size(); i++) {
            std::vector<Vector3> outclip;
            ::ClipPolygon(clipPoly, planesN[i], planesD[i], outclip);
            clipPoly = outclip;
            if (clipPoly.empty()) break;
        }

        if (clipPoly.empty()) {
            mf.contactCount = 2;
            float half = overlap * 0.5f;
            Vector3 mid = {
                (A.position.x + B.position.x) * 0.5f,
                (A.position.y + B.position.y) * 0.5f,
                (A.position.z + B.position.z) * 0.5f
            };
            mf.points[0].position = Sub(mid, Scale(n, 0.25f * overlap));
            mf.points[0].penetration = half;
            mf.points[1].position = Add(mid, Scale(n, 0.25f * overlap));
            mf.points[1].penetration = half;
            return;
        }

        int ccount = 0;
        for (auto& v : clipPoly) {
            if (overlap <= 0.f) continue;
            mf.points[ccount].position = v;
            mf.points[ccount].penetration = overlap;
            ccount++;
            if (ccount >= 4) break;
        }
        mf.contactCount = ccount;
        if (ccount == 0) {
            mf.contactCount = 2;
            float half = overlap * 0.5f;
            Vector3 mid = {
                (A.position.x + B.position.x) * 0.5f,
                (A.position.y + B.position.y) * 0.5f,
                (A.position.z + B.position.z) * 0.5f
            };
            mf.points[0].position = Sub(mid, Scale(n, 0.25f * overlap));
            mf.points[0].penetration = half;
            mf.points[1].position = Add(mid, Scale(n, 0.25f * overlap));
            mf.points[1].penetration = half;
        }
    }

    void ResolveManifold(RigidBody& A, RigidBody& B, const ContactManifold& mf, const PhysicsGlobals& g)
    {
        if (A.isStatic() && B.isStatic()) return;
        float baumBeta = g.baumgarteBeta; // e.g. 0.2

        for (int i = 0; i < mf.contactCount; i++) {
            auto& cp = mf.points[i];
            auto n = mf.normal;
            float pen = cp.penetration;
            float totalInv = A.invMass + B.invMass;
            if (totalInv < 1e-12f) continue;

            // minimal direct fix
            float penFix = pen - 0.001f;
            if (penFix < 0.f) penFix = 0.f;
            float corrFactor = g.correctionFactor;
            auto corr = Scale(n, (penFix * corrFactor) / totalInv);

            if (!A.isStatic()) {
                A.position = Sub(A.position, Scale(corr, A.invMass));
            }
            if (!B.isStatic()) {
                B.position = Add(B.position, Scale(corr, B.invMass));
            }

            auto rA = Sub(cp.position, A.position);
            auto rB = Sub(cp.position, B.position);

            // velocities
            auto vA_lin = A.linearVelocity;
            auto vA_ang = Cross(A.angularVelocity, rA);
            auto vA_tot = Add(vA_lin, vA_ang);

            auto vB_lin = B.linearVelocity;
            auto vB_ang = Cross(B.angularVelocity, rB);
            auto vB_tot = Add(vB_lin, vB_ang);

            auto rv = Sub(vB_tot, vA_tot);
            float vn = Dot(rv, n);

            float penTerm = std::max(pen - 0.001f, 0.f);
            float baumTerm = baumBeta * penTerm / (1.f / 60.f);

            float e = std::min(A.restitution, B.restitution);

            auto InvInertiaWorld = [&](const RigidBody& rb, const Vector3& r, const Vector3& axis) {
                if (rb.isStatic()) return 0.f;
                auto c = Cross(r, axis);
                auto iW = MulMatVec(rb.invInertiaWorld, c);
                auto cross2 = Cross(iW, r);
                return Dot(axis, cross2);
                };

            float invIa = InvInertiaWorld(A, rA, n);
            float invIb = InvInertiaWorld(B, rB, n);

            float j = -(vn + baumTerm * (1.f + e));
            float denom = totalInv + invIa + invIb;
            if (denom < 1e-12f) continue;
            j /= denom;
            auto impulse = Scale(n, j);

            // apply impulse
            auto applyImp = [&](RigidBody& rb, const Vector3& imp, const Vector3& r) {
                if (rb.isStatic()) return;
                rb.linearVelocity = Sub(rb.linearVelocity, Scale(imp, rb.invMass));
                auto c = Cross(r, imp);
                auto w = MulMatVec(rb.invInertiaWorld, c);
                rb.angularVelocity = Sub(rb.angularVelocity, w);
                };

            applyImp(A, impulse, rA);
            applyImp(B, Scale(impulse, -1.f), rB);

            // friction
            auto t = Sub(rv, Scale(n, Dot(rv, n)));
            float tl = Length(t);
            if (tl > 1e-6f) {
                auto T = Scale(t, 1.f / tl);
                float mu = (A.friction + B.friction) * 0.5f;
                float jt = -Dot(rv, T);
                float invIaT = InvInertiaWorld(A, rA, T);
                float invIbT = InvInertiaWorld(B, rB, T);
                float denom2 = totalInv + invIaT + invIbT;
                if (denom2 < 1e-12f) continue;
                jt /= denom2;
                float maxF = std::fabs(j) * mu;
                if (jt > maxF) jt = maxF;
                if (jt < -maxF) jt = -maxF;
                auto fImp = Scale(T, jt);

                applyImp(A, fImp, rA);
                applyImp(B, Scale(fImp, -1.f), rB);
            }
        }
    }

}
