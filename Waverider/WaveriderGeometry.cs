//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - osculating-cone waverider geometry construction.
//
// Implements the Sobieczky osculating-cone inverse-design method:
//
//   * A shock-trace curve is prescribed in the base plane (x = L).
//   * At each spanwise station the local radius of curvature of that curve
//     defines a local cone of constant shock angle beta (one Taylor-Maccoll
//     field is shared by every station because beta is constant).
//   * The lower (compression) surface is obtained by tracing streamlines of
//     that conical flow from the base plane upstream to the shock, where they
//     define the leading edge.
//   * The upper surface is a freestream surface (streamwise rulings from the
//     leading edge to the base plane).
//
// All lengths in this file are in metres. The conversion to PicoGK's millimetre
// world happens only when a mesh is emitted (see WaveriderBuilder).
//

namespace WaveriderForge
{
    /// <summary>A minimal double-precision 3D vector for geometry math.</summary>
    public readonly struct D3
    {
        public readonly double X, Y, Z;
        public D3(double x, double y, double z) { X = x; Y = y; Z = z; }

        public static D3 operator +(D3 a, D3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static D3 operator -(D3 a, D3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static D3 operator *(D3 a, double s) => new(a.X * s, a.Y * s, a.Z * s);

        public static double Dot(D3 a, D3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        public static D3 Cross(D3 a, D3 b) =>
            new(a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X);
        public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
    }

    /// <summary>Immutable description of a waverider design (SI, metres / radians).</summary>
    public sealed class WaveriderDesign
    {
        public double LengthM            { get; init; } = 20.0;  // nose-to-base length L
        public double WidthM             { get; init; } = 18.0;  // full span (2b)
        public double ShockAngleRad      { get; init; }          // beta
        public double CurveDepthRatio    { get; init; } = 0.15;  // Hc / b (shock-curve depth)
        public double CompressionFraction{ get; init; } = 0.55;  // captured streamtube fraction
        public double TipTaper           { get; init; } = 0.85;  // taper of compression toward tips
        public double CurveExponent      { get; init; } = 2.0;   // shock-curve power law (>=2)
        public int    Nspan              { get; init; } = 101;   // spanwise stations (odd)
        public int    Nchord             { get; init; } = 41;    // chordwise samples

        public double HalfSpan => 0.5 * WidthM;

        public WaveriderDesign With(double? beta = null,
                                    double? width = null,
                                    double? depth = null,
                                    double? compression = null)
            => new WaveriderDesign
            {
                LengthM            = LengthM,
                WidthM             = width       ?? WidthM,
                ShockAngleRad      = beta        ?? ShockAngleRad,
                CurveDepthRatio    = depth       ?? CurveDepthRatio,
                CompressionFraction= compression ?? CompressionFraction,
                TipTaper           = TipTaper,
                CurveExponent      = CurveExponent,
                Nspan              = Nspan,
                Nchord             = Nchord,
            };
    }

    /// <summary>
    /// Generated waverider surfaces (point grids) plus geometric metrics. This
    /// class is pure math and has no dependency on PicoGK, so the optimizer can
    /// evaluate thousands of candidates cheaply.
    /// </summary>
    public sealed class WaveriderSurfaces
    {
        public WaveriderDesign      Design { get; }
        public ConicalFlowField     Field  { get; }

        // [s][t] grids: s = 0..Ns-1 (span, -b..+b), t = 0..Nc (chord, LE..base).
        public D3[][] Lower { get; }
        public D3[][] Upper { get; }
        // Local polar angle theta for each lower-surface node (for Cp lookup).
        public double[][] Theta { get; }

        public bool Valid { get; }

        readonly int Ns, Nc;

        public WaveriderSurfaces(WaveriderDesign design, ConicalFlowField field)
        {
            Design = design;
            Field  = field;
            Ns     = design.Nspan;
            Nc     = design.Nchord;

            Lower = new D3[Ns][];
            Upper = new D3[Ns][];
            Theta = new double[Ns][];

            Valid = field.Valid && Build();
        }

        // Shock-trace curve z_s(y) and its first two derivatives (metres).
        void ShockCurve(double y, out double z, out double dz, out double ddz)
        {
            double b  = Design.HalfSpan;
            double Hc = Design.CurveDepthRatio * b;
            double n  = Design.CurveExponent;
            double ay = Math.Abs(y);
            double u  = ay / b;

            z = Hc * Math.Pow(u, n);

            if (ay < 1e-9)
            {
                dz  = 0.0;
                ddz = (Math.Abs(n - 2.0) < 1e-9) ? 2.0 * Hc / (b * b) : 0.0;
                return;
            }

            double sign = Math.Sign(y);
            dz  = sign * Hc * n * Math.Pow(u, n - 1.0) / b;
            ddz = Hc * n * (n - 1.0) * Math.Pow(u, n - 2.0) / (b * b);
        }

        bool Build()
        {
            double L     = Design.LengthM;
            double b     = Design.HalfSpan;
            double beta  = Design.ShockAngleRad;
            double tanB  = Math.Tan(beta);
            double tanC  = Math.Tan(Field.ConeAngle);
            double Rmax  = 200.0 * L;

            for (int s = 0; s < Ns; s++)
            {
                double y = -b + 2.0 * b * s / (Ns - 1);
                ShockCurve(y, out double z, out double dz, out double ddz);

                // Local radius of curvature and unit normal toward the centre of
                // curvature (the concave / compression side).
                double curv = Math.Abs(ddz) / Math.Pow(1.0 + dz * dz, 1.5);
                double R    = curv > 1.0 / Rmax ? 1.0 / curv : Rmax;

                double nlen = Math.Sqrt(1.0 + dz * dz);
                double ny   = -dz / nlen;          // ddz > 0 here, so normal points +z-ish
                double nz   =  1.0 / nlen;

                // Local cone: axis through centre of curvature, apex upstream.
                double Cy     = y + R * ny;
                double Cz     = z + R * nz;
                double apexX  = L - R / tanB;
                double rhatY  = -ny;               // outward radial unit (axis -> shock)
                double rhatZ  = -nz;

                double rhoShock = R;
                double rhoCone  = R * tanC / tanB;

                // Captured streamtube: lower-surface trailing edge radius, tapered
                // toward the tips so the section closes to a fine edge.
                double taper = 1.0 - Design.TipTaper * Math.Pow(Math.Abs(y) / b, 2.0);
                double fc    = Design.CompressionFraction * Math.Max(taper, 0.04);
                double rhoTe = rhoShock - fc * (rhoShock - rhoCone);

                double axialBase = R / tanB;                       // x_base - apexX
                double thetaTe   = Math.Atan2(rhoTe, axialBase);
                double rTe       = Math.Sqrt(axialBase * axialBase + rhoTe * rhoTe);
                double rsTe      = Field.StreamlineRadiusFactor(thetaTe);
                double rCross    = rTe / rsTe;                     // shock-crossing radius

                Lower[s] = new D3[Nc + 1];
                Upper[s] = new D3[Nc + 1];
                Theta[s] = new double[Nc + 1];

                // Leading-edge point (theta = beta).
                double xLE = apexX + rCross * Math.Cos(beta);
                double yLE = Cy + rCross * Math.Sin(beta) * rhatY;
                double zLE = Cz + rCross * Math.Sin(beta) * rhatZ;

                for (int t = 0; t <= Nc; t++)
                {
                    double f  = (double)t / Nc;
                    double th = beta + (thetaTe - beta) * f;       // beta -> thetaTe
                    double rr = rCross * Field.StreamlineRadiusFactor(th);
                    double xx = apexX + rr * Math.Cos(th);
                    double rho = rr * Math.Sin(th);

                    Lower[s][t] = new D3(xx, Cy + rho * rhatY, Cz + rho * rhatZ);
                    Theta[s][t] = th;

                    // Freestream upper surface: streamwise ruling from LE to base.
                    double xu = xLE + (L - xLE) * f;
                    Upper[s][t] = new D3(xu, yLE, zLE);
                }

                if (double.IsNaN(xLE) || double.IsNaN(rCross) || rCross <= 0)
                    return false;
            }
            return true;
        }

        // ---- Triangle enumeration (used for volume and PicoGK mesh) -----------

        public delegate void TriSink(D3 a, D3 b, D3 c);

        /// <summary>Emit every triangle of the closed body, outward-wound.</summary>
        public void ForEachTriangle(TriSink sink)
        {
            void Quad(D3 a, D3 b, D3 c, D3 d) { sink(a, b, c); sink(a, c, d); }

            // Lower (compression) surface - outward normal points downward.
            for (int s = 0; s < Ns - 1; s++)
                for (int t = 0; t < Nc; t++)
                    Quad(Lower[s][t], Lower[s + 1][t], Lower[s + 1][t + 1], Lower[s][t + 1]);

            // Upper (freestream) surface - outward normal points upward.
            for (int s = 0; s < Ns - 1; s++)
                for (int t = 0; t < Nc; t++)
                    Quad(Upper[s][t], Upper[s][t + 1], Upper[s + 1][t + 1], Upper[s + 1][t]);

            // Base cap at x = L - outward normal points downstream (+x).
            for (int s = 0; s < Ns - 1; s++)
                Quad(Lower[s][Nc], Lower[s + 1][Nc], Upper[s + 1][Nc], Upper[s][Nc]);

            // Tip caps (close the first and last cross-section loops).
            EmitTipCap(sink, 0,      outwardSign: -1.0);
            EmitTipCap(sink, Ns - 1, outwardSign: +1.0);
        }

        void EmitTipCap(TriSink sink, int s, double outwardSign)
        {
            // Cross-section loop: lower LE->TE, then upper TE->LE.
            var loop = new List<D3>(2 * Nc + 1);
            for (int t = 0; t <= Nc; t++) loop.Add(Lower[s][t]);
            for (int t = Nc; t >= 1; t--) loop.Add(Upper[s][t]);

            D3 c = new(0, 0, 0);
            foreach (var p in loop) c += p;
            c = c * (1.0 / loop.Count);

            for (int i = 0; i < loop.Count; i++)
            {
                D3 p0 = loop[i];
                D3 p1 = loop[(i + 1) % loop.Count];
                D3 nrm = D3.Cross(p1 - c, p0 - c);
                // Orient so the triangle normal has the desired spanwise sign.
                if (nrm.Y * outwardSign >= 0) sink(c, p0, p1);
                else                          sink(c, p1, p0);
            }
        }

        // ---- Geometric metrics -------------------------------------------------

        /// <summary>Enclosed volume (m^3) via the divergence theorem.</summary>
        public double Volume()
        {
            double v6 = 0.0;
            ForEachTriangle((a, b, c) => v6 += D3.Dot(a, D3.Cross(b, c)));
            return Math.Abs(v6) / 6.0;
        }

        /// <summary>Total wetted (surface) area, m^2.</summary>
        public double WettedArea()
        {
            double area = 0.0;
            ForEachTriangle((a, b, c) => area += 0.5 * D3.Cross(b - a, c - a).Length);
            return area;
        }

        /// <summary>Planform (top-view projected) area, m^2.</summary>
        public double PlanformArea()
        {
            double area = 0.0;
            // Project the lower surface onto the x-y plane.
            for (int s = 0; s < Ns - 1; s++)
                for (int t = 0; t < Nc; t++)
                {
                    D3 a = Lower[s][t], b = Lower[s + 1][t],
                       c = Lower[s + 1][t + 1], d = Lower[s][t + 1];
                    area += 0.5 * Math.Abs(
                        (b.X - a.X) * (d.Y - a.Y) - (b.Y - a.Y) * (d.X - a.X));
                    area += 0.5 * Math.Abs(
                        (c.X - b.X) * (d.Y - b.Y) - (c.Y - b.Y) * (d.X - b.X));
                }
            return area;
        }

        /// <summary>Volumetric efficiency tau = V^(2/3) / S_planform.</summary>
        public double VolumetricEfficiency()
        {
            double sp = PlanformArea();
            return sp > 0 ? Math.Pow(Volume(), 2.0 / 3.0) / sp : 0.0;
        }

        public D3[] LeadingEdge()
        {
            var le = new D3[Ns];
            for (int s = 0; s < Ns; s++) le[s] = Lower[s][0];
            return le;
        }
    }
}
