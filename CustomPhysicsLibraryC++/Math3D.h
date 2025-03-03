#pragma once
#include <cmath> // for std::sqrt, etc.

// Basic math structs + inline functions

namespace Math3D
{
    // ------------ Vector3 ------------
    struct Vector3
    {
        float x, y, z;
    };

    // ------------ Quaternion ------------
    struct Quaternion
    {
        float w, x, y, z;
    };

    // ------------ Matrix3 ------------
    struct Matrix3
    {
        float m[9];
    };

    // Vector functions
    inline float Dot(const Vector3& a, const Vector3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline Vector3 Cross(const Vector3& a, const Vector3& b)
    {
        Vector3 r;
        r.x = a.y * b.z - a.z * b.y;
        r.y = a.z * b.x - a.x * b.z;
        r.z = a.x * b.y - a.y * b.x;
        return r;
    }

    inline Vector3 Add(const Vector3& a, const Vector3& b) { return { a.x + b.x,a.y + b.y,a.z + b.z }; }
    inline Vector3 Sub(const Vector3& a, const Vector3& b) { return { a.x - b.x,a.y - b.y,a.z - b.z }; }
    inline Vector3 Scale(const Vector3& v, float s) { return { v.x * s,v.y * s,v.z * s }; }

    inline float LengthSq(const Vector3& v) { return Dot(v, v); }
    inline float Length(const Vector3& v) { return std::sqrt(LengthSq(v)); }

    inline Vector3 Normalize(const Vector3& v)
    {
        float ln = Length(v);
        if (ln < 1e-12f) return { 0,0,0 };
        return { v.x / ln,v.y / ln,v.z / ln };
    }

    // Quaternion
    inline float QLen(const Quaternion& q)
    {
        return std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    }

    inline Quaternion NormalizeQ(const Quaternion& q)
    {
        float ln = QLen(q);
        if (ln < 1e-12f) return { 1,0,0,0 };
        float inv = 1.f / ln;
        return { q.w * inv,q.x * inv,q.y * inv,q.z * inv };
    }

    inline Quaternion MulQ(const Quaternion& A, const Quaternion& B)
    {
        Quaternion R;
        R.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z;
        R.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y;
        R.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x;
        R.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w;
        return R;
    }

    inline Vector3 QRotate(const Quaternion& q, const Vector3& v)
    {
        Quaternion vq = { 0,v.x,v.y,v.z };
        Quaternion conj = { q.w,-q.x,-q.y,-q.z };
        Quaternion r = MulQ(MulQ(q, vq), conj);
        return { r.x,r.y,r.z };
    }

    inline Vector3 GetAxisX(const Quaternion& q) { return QRotate(q, { 1,0,0 }); }
    inline Vector3 GetAxisY(const Quaternion& q) { return QRotate(q, { 0,1,0 }); }
    inline Vector3 GetAxisZ(const Quaternion& q) { return QRotate(q, { 0,0,1 }); }

    // Matrix3
    bool InvertMatrix3(Matrix3& M);
    Vector3 MulMatVec(const Matrix3& M, const Vector3& v);
}
