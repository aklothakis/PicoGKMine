//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - parametric fins with diamond (double-wedge) airfoils.
//
// Fins are built as watertight lofted solids in absolute vehicle coordinates
// (metres) and sunk into the fuselage so the voxel union is a single body.
// Each fin cross-section is a classic supersonic diamond airfoil; drag is
// estimated with linearized 2-D theory (c_d,wave = 4 tau^2 / sqrt(M^2-1))
// plus reference-temperature skin friction on both faces.
//

namespace WaveriderForge
{
    /// <summary>Parametric description of the fin arrangement (fractions of vehicle size).</summary>
    public sealed class FinSet
    {
        public bool   Pair           = true;   // mirrored outboard pair
        public bool   Center         = false;  // single centerline fin (cant forced 0)
        public double RootChordFrac  = 0.25;   // root chord / vehicle length
        public double TaperRatio     = 0.45;   // tip chord / root chord
        public double HeightFrac     = 0.12;   // fin height / vehicle length
        public double SweepDeg       = 55.0;   // leading-edge sweep
        public double CantDeg        = 15.0;   // outward lean from vertical (pair only)
        public double SpanPosFrac    = 0.60;   // fin root spanwise position / half-span
        public double TEInsetFrac    = 0.0;    // trailing edge inset from the base / length
        public double ThicknessRatio = 0.06;   // diamond t/c
        public double MaxThickPos    = 0.5;    // chordwise position of max thickness
        public double LERadiusMM     = 0.0;    // fin leading-edge bluntness (0 = sharp)
    }

    /// <summary>One generated fin: watertight triangles + data for blunting and drag.</summary>
    public sealed class Fin
    {
        public required List<(D3 a, D3 b, D3 c)> Tris { get; init; }
        public required D3     LERoot         { get; init; }
        public required D3     LETip          { get; init; }
        public required double ExposedAreaM2  { get; init; }
        public required double MeanChordM     { get; init; }
        public required double ThicknessRatio { get; init; }
        public required double LERadiusMM     { get; init; }
    }

    public static class FinGeometry
    {
        /// <summary>Build the configured fins on the given body. Sides: pair and/or center.</summary>
        public static List<Fin> Build(WaveriderSurfaces s, FinSet set)
        {
            var fins = new List<Fin>();
            if (set.Pair)
            {
                fins.Add(BuildOne(s, set, +1));
                fins.Add(BuildOne(s, set, -1));
            }
            if (set.Center)
                fins.Add(BuildOne(s, set, 0));
            return fins;
        }

        static Fin BuildOne(WaveriderSurfaces s, FinSet set, int side)
        {
            double L     = s.Design.LengthM;
            double b     = s.Design.HalfSpan;
            double cRoot = Math.Max(set.RootChordFrac, 0.02) * L;
            double cTip  = Math.Clamp(set.TaperRatio, 0.05, 1.0) * cRoot;
            double h     = Math.Max(set.HeightFrac, 0.01) * L;
            double sweep = set.SweepDeg * Math.PI / 180.0;
            double cant  = side == 0 ? 0.0 : set.CantDeg * Math.PI / 180.0;
            double tau   = Math.Clamp(set.ThicknessRatio, 0.02, 0.20);
            double fm    = Math.Clamp(set.MaxThickPos, 0.2, 0.8);

            double xTeRoot = L * (1.0 - Math.Clamp(set.TEInsetFrac, 0.0, 0.5));
            double x0      = xTeRoot - cRoot;                    // root LE x
            double yPos    = side == 0 ? 0.0 : set.SpanPosFrac * b;

            // Mount height: sample the body at mid root chord and sink the root
            // 75% of the local body thickness below the upper surface, without
            // ever dropping below the lower surface.
            s.SampleZ(x0 + 0.5 * cRoot, side >= 0 ? yPos : -yPos,
                      out double zLo, out double zHi);
            double z0 = zLo + 0.25 * (zHi - zLo);

            double cf = Math.Cos(cant), sf = Math.Sin(cant);

            // Local fin frame: xl chordwise from root LE, yt thickness, zl along
            // the (pre-cant) fin axis. The fin is built leaning outward on the
            // +y side, then the whole thing is mirrored (y -> -y) for side = -1.
            D3 P(double xl, double yt, double zl)
            {
                double y  = yt * cf + zl * sf;                 // outward lean
                double z  = -yt * sf + zl * cf;
                double py = yPos + y;
                if (side < 0) py = -py;
                return new D3(x0 + xl, py, z0 + z);
            }

            // Section polygon at height zl: [LE, midTop, TE, midBot].
            D3[] Section(double zl)
            {
                double f = zl / h;
                double c = cRoot + (cTip - cRoot) * f;
                double xle = zl * Math.Tan(sweep);
                double t2 = 0.5 * tau * c;
                return new[]
                {
                    P(xle,            0.0, zl),
                    P(xle + fm * c,  +t2,  zl),
                    P(xle + c,        0.0, zl),
                    P(xle + fm * c,  -t2,  zl),
                };
            }

            D3[] root = Section(0.0);
            D3[] tip  = Section(h);

            D3 C0 = Centroid(root);
            D3 C1 = Centroid(tip);
            D3 axisMid = (C0 + C1) * 0.5;

            var tris = new List<(D3, D3, D3)>();

            // Four side faces with outward winding.
            for (int i = 0; i < 4; i++)
            {
                int j = (i + 1) % 4;
                D3 fc = (root[i] + root[j] + tip[j] + tip[i]) * 0.25;
                AddQuad(tris, root[i], root[j], tip[j], tip[i], fc - axisMid);
            }

            // Root and tip caps.
            AddCap(tris, root, C0 - C1);
            AddCap(tris, tip,  C1 - C0);

            // Exposed geometry estimate for drag (part above the upper surface).
            double buried  = Math.Max(0.0, (zHi - z0)) / Math.Max(cf, 0.2);
            double hExp    = Math.Max(h - buried, 0.05 * h);
            double cAtSurf = cRoot + (cTip - cRoot) * Math.Clamp(buried / h, 0, 1);
            double area    = 0.5 * (cAtSurf + cTip) * hExp;

            return new Fin
            {
                Tris = tris,
                LERoot = root[0],
                LETip  = tip[0],
                ExposedAreaM2  = area,
                MeanChordM     = 0.5 * (cAtSurf + cTip),
                ThicknessRatio = tau,
                LERadiusMM     = set.LERadiusMM,
            };
        }

        static D3 Centroid(D3[] p)
        {
            D3 c = new(0, 0, 0);
            foreach (var q in p) c += q;
            return c * (1.0 / p.Length);
        }

        static void AddQuad(List<(D3, D3, D3)> tris, D3 a, D3 b, D3 c, D3 d, D3 outwardRef)
        {
            D3 n = D3.Cross(b - a, c - a);
            if (D3.Dot(n, outwardRef) >= 0) { tris.Add((a, b, c)); tris.Add((a, c, d)); }
            else                            { tris.Add((a, c, b)); tris.Add((a, d, c)); }
        }

        static void AddCap(List<(D3, D3, D3)> tris, D3[] p, D3 outwardRef)
        {
            D3 n = D3.Cross(p[1] - p[0], p[2] - p[0]);
            if (D3.Dot(n, outwardRef) >= 0) { tris.Add((p[0], p[1], p[2])); tris.Add((p[0], p[2], p[3])); }
            else                            { tris.Add((p[0], p[2], p[1])); tris.Add((p[0], p[3], p[2])); }
        }

        /// <summary>
        /// Estimated zero-lift drag of one fin (N): linearized double-wedge wave
        /// drag plus reference-temperature skin friction on both faces.
        /// </summary>
        public static double DragNewtons(FlightState f, Fin fin)
        {
            double m = f.Mach;
            if (m <= 1.05) return 0;
            double beta = Math.Sqrt(m * m - 1.0);
            double cdw  = 4.0 * fin.ThicknessRatio * fin.ThicknessRatio / beta;
            double dW   = cdw * f.DynamicPressure * fin.ExposedAreaM2;
            double dF   = AeroPerformance.FrictionDrag(
                m, f.Temperature, f.Pressure,
                Math.Max(fin.MeanChordM, 1e-3), 2.0 * fin.ExposedAreaM2, f.Gamma);
            return dW + dF;
        }
    }
}
