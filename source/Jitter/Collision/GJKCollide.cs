using Jitter.LinearMath;

namespace Jitter.Collision
{
    public sealed class GJKCollide
    {
        private const int MaxIterations = 15;

        private static readonly ResourcePool<VoronoiSimplexSolver> simplexSolverPool = new ResourcePool<VoronoiSimplexSolver>();

        private static void SupportMapTransformed(
            ISupportMappable support,
            in JMatrix orientation,
            in JVector position,
            in JVector direction,
            out JVector result)
        {
            result = new JVector(
                (direction.X * orientation.M11) + (direction.Y * orientation.M12) + (direction.Z * orientation.M13),
                (direction.X * orientation.M21) + (direction.Y * orientation.M22) + (direction.Z * orientation.M23),
                (direction.X * orientation.M31) + (direction.Y * orientation.M32) + (direction.Z * orientation.M33));

            support.SupportMapping(result, out result);

            float x = (result.X * orientation.M11) + (result.Y * orientation.M21) + (result.Z * orientation.M31);
            float y = (result.X * orientation.M12) + (result.Y * orientation.M22) + (result.Z * orientation.M32);
            float z = (result.X * orientation.M13) + (result.Y * orientation.M23) + (result.Z * orientation.M33);

            result = new JVector(
                position.X + x,
                position.Y + y,
                position.Z + z);
        }

        public static bool Pointcast(
            ISupportMappable support,
            in JMatrix orientation,
            in JVector position,
            in JVector point)
        {
            SupportMapTransformed(support, orientation, position, point, out var arbitraryPoint);
            JVector.Subtract(point, arbitraryPoint, out arbitraryPoint);

            support.SupportCenter(out var r);
            JVector.Transform(r, orientation, out r);
            JVector.Add(position, r, out r);
            JVector.Subtract(point, r, out r);

            var x = point;
            float VdotR;

            JVector.Subtract(x, arbitraryPoint, out var v);
            float dist = v.LengthSquared();
            const float epsilon = 0.0001f;

            int maxIter = MaxIterations;

            var simplexSolver = simplexSolverPool.GetNew();

            simplexSolver.Reset();

            while ((dist > epsilon) && (maxIter-- != 0))
            {
                SupportMapTransformed(support, orientation, position, v, out var p);
                JVector.Subtract(x, p, out var w);

                float VdotW = JVector.Dot(v, w);

                if (VdotW > 0.0f)
                {
                    VdotR = JVector.Dot(v, r);

                    if (VdotR >= -(JMath.Epsilon * JMath.Epsilon))
                    {
                        simplexSolverPool.GiveBack(simplexSolver);
                        return false;
                    }
                    else
                    {
                        simplexSolver.Reset();
                    }
                }
                if (!simplexSolver.InSimplex(w))
                {
                    simplexSolver.AddVertex(w, x, p);
                }

                if (simplexSolver.Closest(out v))
                {
                    dist = v.LengthSquared();
                }
                else
                {
                    dist = 0.0f;
                }
            }

            simplexSolverPool.GiveBack(simplexSolver);
            return true;
        }

        public static bool ClosestPoints(
            ISupportMappable support1,
            ISupportMappable support2,
            in JMatrix orientation1,
            in JMatrix orientation2,
            in JVector position1,
            in JVector position2,
            out JVector p1,
            out JVector p2,
            out JVector normal)
        {
            var simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            var r = position1 - position2;
            var rn = JVector.Negate(r);

            SupportMapTransformed(support1, orientation1, position1, rn, out var supVertexA);
            SupportMapTransformed(support2, orientation2, position2, r, out var supVertexB);

            var v = supVertexA - supVertexB;

            normal = JVector.Zero;

            int maxIter = 15;

            float distSq = v.LengthSquared();
            const float epsilon = 0.00001f;

            while ((distSq > epsilon) && (maxIter-- != 0))
            {
                var vn = JVector.Negate(v);
                SupportMapTransformed(support1, orientation1, position1, vn, out supVertexA);
                SupportMapTransformed(support2, orientation2, position2, v, out supVertexB);
                var w = supVertexA - supVertexB;

                if (!simplexSolver.InSimplex(w))
                {
                    simplexSolver.AddVertex(w, supVertexA, supVertexB);
                }

                if (simplexSolver.Closest(out v))
                {
                    distSq = v.LengthSquared();
                    normal = v;
                }
                else
                {
                    distSq = 0.0f;
                }
            }

            simplexSolver.ComputePoints(out p1, out p2);

            if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
            {
                normal = JVector.Normalize(normal);
            }

            simplexSolverPool.GiveBack(simplexSolver);

            return true;
        }

        public static bool Raycast(
            ISupportMappable support,
            in JMatrix orientation,
            in JVector position,
            in JVector origin,
            in JVector direction,
            out float fraction,
            out JVector normal)
        {
            var simplexSolver = simplexSolverPool.GetNew();
            simplexSolver.Reset();

            normal = JVector.Zero;
            fraction = float.MaxValue;

            float lambda = 0.0f;

            var r = direction;
            var x = origin;

            SupportMapTransformed(support, orientation, position, r, out var arbitraryPoint);
            JVector.Subtract(x, arbitraryPoint, out var v);

            int maxIter = MaxIterations;

            float distSq = v.LengthSquared();
            const float epsilon = 0.000001f;

            float VdotR;

            while ((distSq > epsilon) && (maxIter-- != 0))
            {
                SupportMapTransformed(support, orientation, position, v, out var p);
                JVector.Subtract(x, p, out var w);

                float VdotW = JVector.Dot(v, w);

                if (VdotW > 0.0f)
                {
                    VdotR = JVector.Dot(v, r);

                    if (VdotR >= -JMath.Epsilon)
                    {
                        simplexSolverPool.GiveBack(simplexSolver);
                        return false;
                    }
                    else
                    {
                        lambda -= VdotW / VdotR;
                        JVector.Multiply(r, lambda, out x);
                        JVector.Add(origin, x, out x);
                        JVector.Subtract(x, p, out w);
                        normal = v;
                    }
                }
                if (!simplexSolver.InSimplex(w))
                {
                    simplexSolver.AddVertex(w, x, p);
                }

                if (simplexSolver.Closest(out v)) { distSq = v.LengthSquared(); }
                else
                {
                    distSq = 0.0f;
                }
            }

            simplexSolver.ComputePoints(out _, out var p2);

            p2 -= origin;
            fraction = p2.Length() / direction.Length();

            if (normal.LengthSquared() > JMath.Epsilon * JMath.Epsilon)
            {
                normal = JVector.Normalize(normal);
            }

            simplexSolverPool.GiveBack(simplexSolver);

            return true;
        }

        private class UsageBitfield
        {
            public bool UsedVertexA { get; set; }
            public bool UsedVertexB { get; set; }
            public bool UsedVertexC { get; set; }
            public bool UsedVertexD { get; set; }

            public void Reset()
            {
                UsedVertexA = UsedVertexB = UsedVertexC = UsedVertexD = false;
            }
        }

        private class SubSimplexClosestResult
        {
            public JVector ClosestPointOnSimplex { get; set; }
            public UsageBitfield UsedVertices { get; set; } = new UsageBitfield();
            public float[] BarycentricCoords { get; set; } = new float[4];
            public bool Degenerate { get; set; }

            public void Reset()
            {
                Degenerate = false;
                SetBarycentricCoordinates();
                UsedVertices.Reset();
            }

            public bool IsValid => (BarycentricCoords[0] >= 0f)
                            && (BarycentricCoords[1] >= 0f)
                            && (BarycentricCoords[2] >= 0f)
                            && (BarycentricCoords[3] >= 0f);

            public void SetBarycentricCoordinates()
            {
                SetBarycentricCoordinates(0f, 0f, 0f, 0f);
            }

            public void SetBarycentricCoordinates(float a, float b, float c, float d)
            {
                BarycentricCoords[0] = a;
                BarycentricCoords[1] = b;
                BarycentricCoords[2] = c;
                BarycentricCoords[3] = d;
            }
        }

        private class VoronoiSimplexSolver
        {
            private const int VertexA = 0, VertexB = 1, VertexC = 2;

            private const int VoronoiSimplexMaxVerts = 5;
            private readonly JVector[] _simplexVectorW = new JVector[VoronoiSimplexMaxVerts];
            private readonly JVector[] _simplexPointsP = new JVector[VoronoiSimplexMaxVerts];
            private readonly JVector[] _simplexPointsQ = new JVector[VoronoiSimplexMaxVerts];

            private JVector _cachedPA;
            private JVector _cachedPB;
            private JVector _cachedV;
            private JVector _lastW;
            private bool _cachedValidClosest;

            private SubSimplexClosestResult _cachedBC = new SubSimplexClosestResult();

            private SubSimplexClosestResult tempResult = new SubSimplexClosestResult();

            private bool _needsUpdate;

            public bool FullSimplex => NumVertices == 4;

            public int NumVertices { get; private set; }

            public void Reset()
            {
                _cachedValidClosest = false;
                NumVertices = 0;
                _needsUpdate = true;
                _lastW = new JVector(1e30f, 1e30f, 1e30f);
                _cachedBC.Reset();
            }

            public void AddVertex(JVector w, JVector p, JVector q)
            {
                _lastW = w;
                _needsUpdate = true;

                _simplexVectorW[NumVertices] = w;
                _simplexPointsP[NumVertices] = p;
                _simplexPointsQ[NumVertices] = q;

                NumVertices++;
            }

            public bool Closest(out JVector v)
            {
                bool succes = UpdateClosestVectorAndPoints();
                v = _cachedV;
                return succes;
            }

            public float MaxVertex
            {
                get
                {
                    int numverts = NumVertices;
                    float maxV = 0f, curLen2;
                    for (int i = 0; i < numverts; i++)
                    {
                        curLen2 = _simplexVectorW[i].LengthSquared();
                        if (maxV < curLen2)
                        {
                            maxV = curLen2;
                        }
                    }
                    return maxV;
                }
            }

            public int GetSimplex(out JVector[] pBuf, out JVector[] qBuf, out JVector[] yBuf)
            {
                int numverts = NumVertices;
                pBuf = new JVector[numverts];
                qBuf = new JVector[numverts];
                yBuf = new JVector[numverts];
                for (int i = 0; i < numverts; i++)
                {
                    yBuf[i] = _simplexVectorW[i];
                    pBuf[i] = _simplexPointsP[i];
                    qBuf[i] = _simplexPointsQ[i];
                }
                return numverts;
            }

            public bool InSimplex(JVector w)
            {
                if (w == _lastW)
                {
                    return true;
                }

                int numverts = NumVertices;
                for (int i = 0; i < numverts; i++)
                {
                    if (_simplexVectorW[i] == w)
                    {
                        return true;
                    }
                }

                return false;
            }

            public void BackupClosest(out JVector v)
            {
                v = _cachedV;
            }

            public bool EmptySimplex => NumVertices == 0;

            public void ComputePoints(out JVector p1, out JVector p2)
            {
                UpdateClosestVectorAndPoints();
                p1 = _cachedPA;
                p2 = _cachedPB;
            }

            public void RemoveVertex(int index)
            {
                NumVertices--;
                _simplexVectorW[index] = _simplexVectorW[NumVertices];
                _simplexPointsP[index] = _simplexPointsP[NumVertices];
                _simplexPointsQ[index] = _simplexPointsQ[NumVertices];
            }

            public void ReduceVertices(UsageBitfield usedVerts)
            {
                if ((NumVertices >= 4) && (!usedVerts.UsedVertexD))
                {
                    RemoveVertex(3);
                }

                if ((NumVertices >= 3) && (!usedVerts.UsedVertexC))
                {
                    RemoveVertex(2);
                }

                if ((NumVertices >= 2) && (!usedVerts.UsedVertexB))
                {
                    RemoveVertex(1);
                }

                if ((NumVertices >= 1) && (!usedVerts.UsedVertexA))
                {
                    RemoveVertex(0);
                }
            }

            public bool UpdateClosestVectorAndPoints()
            {
                if (_needsUpdate)
                {
                    _cachedBC.Reset();
                    _needsUpdate = false;

                    JVector p, a, b, c, d;
                    switch (NumVertices)
                    {
                        case 0:
                            _cachedValidClosest = false;
                            break;
                        case 1:
                            _cachedPA = _simplexPointsP[0];
                            _cachedPB = _simplexPointsQ[0];
                            _cachedV = _cachedPA - _cachedPB;
                            _cachedBC.Reset();
                            _cachedBC.SetBarycentricCoordinates(1f, 0f, 0f, 0f);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 2:
                            var from = _simplexVectorW[0];
                            var to = _simplexVectorW[1];
                            var diff = from * (-1);
                            var v = to - from;
                            float t = JVector.Dot(v, diff);

                            if (t > 0)
                            {
                                float dotVV = v.LengthSquared();
                                if (t < dotVV)
                                {
                                    t /= dotVV;
                                    _cachedBC.UsedVertices.UsedVertexA = true;
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                                else
                                {
                                    t = 1;
                                    _cachedBC.UsedVertices.UsedVertexB = true;
                                }
                            }
                            else
                            {
                                t = 0;
                                _cachedBC.UsedVertices.UsedVertexA = true;
                            }

                            _cachedBC.SetBarycentricCoordinates(1 - t, t, 0, 0);
                            _ = from + (t * v);

                            _cachedPA = _simplexPointsP[0] + (t * (_simplexPointsP[1] - _simplexPointsP[0]));
                            _cachedPB = _simplexPointsQ[0] + (t * (_simplexPointsQ[1] - _simplexPointsQ[0]));
                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);

                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 3:
                            p = new JVector();
                            a = _simplexVectorW[0];
                            b = _simplexVectorW[1];
                            c = _simplexVectorW[2];

                            ClosestPtPointTriangle(p, a, b, c, _cachedBC);
                            _cachedPA = (_simplexPointsP[0] * _cachedBC.BarycentricCoords[0])
                                            + (_simplexPointsP[1] * _cachedBC.BarycentricCoords[1])
                                            + (_simplexPointsP[2] * _cachedBC.BarycentricCoords[2])
                                            + (_simplexPointsP[3] * _cachedBC.BarycentricCoords[3]);

                            _cachedPB = (_simplexPointsQ[0] * _cachedBC.BarycentricCoords[0])
                                            + (_simplexPointsQ[1] * _cachedBC.BarycentricCoords[1])
                                            + (_simplexPointsQ[2] * _cachedBC.BarycentricCoords[2])
                                            + (_simplexPointsQ[3] * _cachedBC.BarycentricCoords[3]);

                            _cachedV = _cachedPA - _cachedPB;

                            ReduceVertices(_cachedBC.UsedVertices);
                            _cachedValidClosest = _cachedBC.IsValid;
                            break;
                        case 4:
                            p = new JVector();
                            a = _simplexVectorW[0];
                            b = _simplexVectorW[1];
                            c = _simplexVectorW[2];
                            d = _simplexVectorW[3];

                            bool hasSeperation = ClosestPtPointTetrahedron(p, a, b, c, d, _cachedBC);

                            if (hasSeperation)
                            {
                                _cachedPA = (_simplexPointsP[0] * _cachedBC.BarycentricCoords[0])
                                                + (_simplexPointsP[1] * _cachedBC.BarycentricCoords[1])
                                                + (_simplexPointsP[2] * _cachedBC.BarycentricCoords[2])
                                                + (_simplexPointsP[3] * _cachedBC.BarycentricCoords[3]);

                                _cachedPB = (_simplexPointsQ[0] * _cachedBC.BarycentricCoords[0])
                                                + (_simplexPointsQ[1] * _cachedBC.BarycentricCoords[1])
                                                + (_simplexPointsQ[2] * _cachedBC.BarycentricCoords[2])
                                                + (_simplexPointsQ[3] * _cachedBC.BarycentricCoords[3]);

                                _cachedV = _cachedPA - _cachedPB;
                                ReduceVertices(_cachedBC.UsedVertices);
                            }
                            else
                            {
                                if (_cachedBC.Degenerate)
                                {
                                    _cachedValidClosest = false;
                                }
                                else
                                {
                                    _cachedValidClosest = true;
                                    _cachedV = new JVector();

                                }
                                break;
                            }

                            _cachedValidClosest = _cachedBC.IsValid;

                            break;
                        default:
                            _cachedValidClosest = false;
                            break;
                    }
                }

                return _cachedValidClosest;
            }

            public bool ClosestPtPointTriangle(
                JVector p,
                JVector a,
                JVector b,
                JVector c,
                SubSimplexClosestResult result)
            {
                result.UsedVertices.Reset();

                float v, w;

                var ab = b - a;
                var ac = c - a;
                var ap = p - a;
                float d1 = JVector.Dot(ab, ap);
                float d2 = JVector.Dot(ac, ap);
                if (d1 <= 0f && d2 <= 0f)
                {
                    result.ClosestPointOnSimplex = a;
                    result.UsedVertices.UsedVertexA = true;
                    result.SetBarycentricCoordinates(1, 0, 0, 0);
                    return true;
                }

                var bp = p - b;
                float d3 = JVector.Dot(ab, bp);
                float d4 = JVector.Dot(ac, bp);
                if (d3 >= 0f && d4 <= d3)
                {
                    result.ClosestPointOnSimplex = b;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(0, 1, 0, 0);

                    return true;
                }
                float vc = (d1 * d4) - (d3 * d2);
                if (vc <= 0f && d1 >= 0f && d3 <= 0f)
                {
                    v = d1 / (d1 - d3);
                    result.ClosestPointOnSimplex = a + (v * ab);
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexB = true;
                    result.SetBarycentricCoordinates(1 - v, v, 0, 0);
                    return true;
                }

                var cp = p - c;
                float d5 = JVector.Dot(ab, cp);
                float d6 = JVector.Dot(ac, cp);
                if (d6 >= 0f && d5 <= d6)
                {
                    result.ClosestPointOnSimplex = c;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 0, 1, 0);
                    return true;
                }

                float vb = (d5 * d2) - (d1 * d6);
                if (vb <= 0f && d2 >= 0f && d6 <= 0f)
                {
                    w = d2 / (d2 - d6);
                    result.ClosestPointOnSimplex = a + (w * ac);
                    result.UsedVertices.UsedVertexA = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(1 - w, 0, w, 0);
                    return true;
                }

                float va = (d3 * d6) - (d5 * d4);
                if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
                {
                    w = (d4 - d3) / (d4 - d3 + (d5 - d6));

                    result.ClosestPointOnSimplex = b + (w * (c - b));
                    result.UsedVertices.UsedVertexB = true;
                    result.UsedVertices.UsedVertexC = true;
                    result.SetBarycentricCoordinates(0, 1 - w, w, 0);
                    return true;
                }

                float denom = 1.0f / (va + vb + vc);
                v = vb * denom;
                w = vc * denom;

                result.ClosestPointOnSimplex = a + (ab * v) + (ac * w);
                result.UsedVertices.UsedVertexA = true;
                result.UsedVertices.UsedVertexB = true;
                result.UsedVertices.UsedVertexC = true;
                result.SetBarycentricCoordinates(1 - v - w, v, w, 0);

                return true;
            }

            public int PointOutsideOfPlane(JVector p, JVector a, JVector b, JVector c, JVector d)
            {
                var normal = JVector.Cross(b - a, c - a);

                float signp = JVector.Dot(p - a, normal);
                float signd = JVector.Dot(d - a, normal);

                if (signd * signd < (1e-4f * 1e-4f))
                {
                    return -1;
                }

                return signp * signd < 0f ? 1 : 0;
            }

            public bool ClosestPtPointTetrahedron(
                JVector p,
                JVector a, 
                JVector b, 
                JVector c, 
                JVector d,
                SubSimplexClosestResult finalResult)
            {
                tempResult.Reset();

                finalResult.ClosestPointOnSimplex = p;
                finalResult.UsedVertices.Reset();
                finalResult.UsedVertices.UsedVertexA = true;
                finalResult.UsedVertices.UsedVertexB = true;
                finalResult.UsedVertices.UsedVertexC = true;
                finalResult.UsedVertices.UsedVertexD = true;

                int pointOutsideABC = PointOutsideOfPlane(p, a, b, c, d);
                int pointOutsideACD = PointOutsideOfPlane(p, a, c, d, b);
                int pointOutsideADB = PointOutsideOfPlane(p, a, d, b, c);
                int pointOutsideBDC = PointOutsideOfPlane(p, b, d, c, a);

                if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
                {
                    finalResult.Degenerate = true;
                    return false;
                }

                if (pointOutsideABC == 0 && pointOutsideACD == 0 && pointOutsideADB == 0 && pointOutsideBDC == 0)
                {
                    return false;
                }

                float bestSqDist = float.MaxValue;
                if (pointOutsideABC != 0)
                {
                    ClosestPtPointTriangle(p, a, b, c, tempResult);
                    var q = tempResult.ClosestPointOnSimplex;

                    float sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexB],
                                tempResult.BarycentricCoords[VertexC],
                                0);
                    }
                }

                if (pointOutsideACD != 0)
                {
                    ClosestPtPointTriangle(p, a, c, d, tempResult);
                    var q = tempResult.ClosestPointOnSimplex;

                    float sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                0,
                                tempResult.BarycentricCoords[VertexB],
                                tempResult.BarycentricCoords[VertexC]);
                    }
                }

                if (pointOutsideADB != 0)
                {
                    ClosestPtPointTriangle(p, a, d, b, tempResult);
                    var q = tempResult.ClosestPointOnSimplex;

                    float sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        bestSqDist = sqDist;
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexA = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexC;
                        finalResult.SetBarycentricCoordinates(
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexC],
                                0,
                                tempResult.BarycentricCoords[VertexB]);
                    }
                }

                if (pointOutsideBDC != 0)
                {
                    ClosestPtPointTriangle(p, b, d, c, tempResult);
                    var q = tempResult.ClosestPointOnSimplex;
                    float sqDist = (q - p).LengthSquared();
                    if (sqDist < bestSqDist)
                    {
                        finalResult.ClosestPointOnSimplex = q;
                        finalResult.UsedVertices.Reset();
                        finalResult.UsedVertices.UsedVertexB = tempResult.UsedVertices.UsedVertexA;
                        finalResult.UsedVertices.UsedVertexD = tempResult.UsedVertices.UsedVertexB;
                        finalResult.UsedVertices.UsedVertexC = tempResult.UsedVertices.UsedVertexC;

                        finalResult.SetBarycentricCoordinates(
                                0,
                                tempResult.BarycentricCoords[VertexA],
                                tempResult.BarycentricCoords[VertexC],
                                tempResult.BarycentricCoords[VertexB]);
                    }
                }

                return true;
            }
        }
    }
}
