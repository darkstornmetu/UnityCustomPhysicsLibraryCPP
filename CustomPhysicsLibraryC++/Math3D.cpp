#include "Math3D.h"

namespace Math3D
{
    bool InvertMatrix3(Matrix3& M)
    {
        float d = M.m[0] * (M.m[4] * M.m[8] - M.m[5] * M.m[7])
            - M.m[1] * (M.m[3] * M.m[8] - M.m[5] * M.m[6])
            + M.m[2] * (M.m[3] * M.m[7] - M.m[4] * M.m[6]);
        if (std::fabs(d) < 1e-12f) return false;
        float inv = 1.f / d;
        Matrix3 R;
        R.m[0] = (M.m[4] * M.m[8] - M.m[5] * M.m[7]) * inv;
        R.m[1] = (M.m[2] * M.m[7] - M.m[1] * M.m[8]) * inv;
        R.m[2] = (M.m[1] * M.m[5] - M.m[2] * M.m[4]) * inv;
        R.m[3] = (M.m[5] * M.m[6] - M.m[3] * M.m[8]) * inv;
        R.m[4] = (M.m[0] * M.m[8] - M.m[2] * M.m[6]) * inv;
        R.m[5] = (M.m[2] * M.m[3] - M.m[0] * M.m[5]) * inv;
        R.m[6] = (M.m[3] * M.m[7] - M.m[4] * M.m[6]) * inv;
        R.m[7] = (M.m[1] * M.m[6] - M.m[0] * M.m[7]) * inv;
        R.m[8] = (M.m[0] * M.m[4] - M.m[1] * M.m[3]) * inv;
        M = R;
        return true;
    }

    Vector3 MulMatVec(const Matrix3& M, const Vector3& v)
    {
        return {
          M.m[0] * v.x + M.m[1] * v.y + M.m[2] * v.z,
          M.m[3] * v.x + M.m[4] * v.y + M.m[5] * v.z,
          M.m[6] * v.x + M.m[7] * v.y + M.m[8] * v.z
        };
    }
}
