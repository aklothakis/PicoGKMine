//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - static stability analysis and fin sizing recommendation.
//
// Assumptions (engineering / concept level, stated in the report):
//   * CG at the volume centroid of the body (uniform density).
//   * Pitch: neutral point from Modified-Newtonian panel force/moment
//     derivatives (central finite difference in angle of attack). Static
//     margin is (x_np - x_cg)/L, positive = stable (x runs nose -> base).
//   * Yaw: the body's lateral centre of pressure from the same panel method in
//     sideslip; fin effectiveness from linearized supersonic theory,
//     CL_alpha = (4/beta)(1 - 1/(2 AR beta)), reduced by cos^2(cant) and a
//     blanketing/interference factor. Directional stability is expressed as
//     Cn_beta per radian (positive = weathercock stable).
//   * Fin sizing: the exposed fin area that achieves a target Cn_beta
//     (default 0.05 /rad, a typical concept-level target for high-speed
//     vehicles) with the current fin arrangement, converted to a fin height
//     using the current chord and taper settings.
//

namespace WaveriderForge
{
    public sealed class StabilityInfo
    {
        public double CgX, CgZ;                 // volume centroid (m)
        public double XnpPitch;                 // pitch neutral point (m from nose)
        public double StaticMarginPctL;         // (Xnp - Xcg)/L * 100, + = stable
        public double CnBetaBody;               // /rad (+ = stabilizing)
        public double CnBetaFins;               // /rad
        public double CnBetaTotal;              // /rad
        public double TargetCnBeta;             // /rad
        public double ReqAreaPerFinM2;          // exposed area per fin to hit target
        public double ReqHeightFrac;            // fin height / length to hit target
        public bool   ReqValid;                 // false if fins cannot stabilize (e.g. ahead of CG)
        public bool   FinsHypothetical;         // recommendation assumed a mirrored pair

        public string Describe(double lengthM)
        {
            var sb = new System.Text.StringBuilder();
            sb.AppendLine("Stability (CG assumed at volume centroid)");
            sb.AppendLine($"  CG position          : x = {CgX:F2} m ({CgX / lengthM * 100:F1} % L)");
            sb.AppendLine($"  Pitch neutral point  : x = {XnpPitch:F2} m ({XnpPitch / lengthM * 100:F1} % L)");
            sb.AppendLine($"  Static margin        : {StaticMarginPctL:+0.0;-0.0} % L " +
                          (StaticMarginPctL >= 0 ? "(stable)" : "(UNSTABLE - move payload forward or add pitch trim)"));
            sb.AppendLine($"  Cn_beta body / fins  : {CnBetaBody:+0.0000;-0.0000} / {CnBetaFins:+0.0000;-0.0000} per rad");
            sb.AppendLine($"  Cn_beta total        : {CnBetaTotal:+0.0000;-0.0000} per rad  (target {TargetCnBeta:F3}, " +
                          (CnBetaTotal >= TargetCnBeta ? "MET)" : "below target)"));
            if (ReqValid)
            {
                sb.AppendLine($"  Recommended fin size : {ReqAreaPerFinM2:F2} m^2 exposed per fin " +
                              $"-> height ~ {ReqHeightFrac * 100:F1} % of length" +
                              (FinsHypothetical ? "  (assuming a mirrored pair with the current fin settings)" : ""));
            }
            else
            {
                sb.AppendLine("  Recommended fin size : n/a (fin centre of pressure is not aft of the CG;" +
                              " move fins aft or reduce TE inset)");
            }
            return sb.ToString();
        }
    }

    public static class Stability
    {
        public const double DefaultTargetCnBeta = 0.05;   // per rad
        const double FinEfficiency = 0.85;                // blanketing / interference
        const double DeltaRad = 1.5 * Math.PI / 180.0;    // finite-difference step

        /// <summary>
        /// Analyze static stability of the body (+ fins if present) and produce
        /// a fin-area recommendation for the target Cn_beta. If no fins are
        /// configured, the recommendation assumes a mirrored pair using
        /// <paramref name="finSet"/> (or defaults).
        /// </summary>
        public static StabilityInfo Analyze(WaveriderSurfaces s,
                                            FlightState flow,
                                            AeroResult aero,
                                            IReadOnlyList<Fin>? fins,
                                            FinSet? finSet,
                                            double targetCnBeta = DefaultTargetCnBeta)
        {
            double L    = s.Design.LengthM;
            double Sref = Math.Max(aero.PlanformArea, 1e-6);
            double bref = Math.Max(s.Design.WidthM, 1e-6);
            double q    = flow.DynamicPressure;

            // ---- CG: volume centroid via divergence theorem --------------------
            double v6 = 0; D3 cSum = new(0, 0, 0);
            s.ForEachTriangle((a, b, c) =>
            {
                double vt = D3.Dot(a, D3.Cross(b, c));      // 6 * signed tet volume
                v6 += vt;
                cSum += (a + b + c) * (vt * 0.25);          // tet centroid * 6V
            });
            D3 cg = Math.Abs(v6) > 1e-12 ? cSum * (1.0 / v6) : new D3(0.6 * L, 0, 0);

            // ---- Panel force/moment derivatives (Newtonian, about the nose) ----
            (D3 F, D3 M) LoadsAt(double alpha, double beta)
            {
                D3 dir = new(Math.Cos(alpha) * Math.Cos(beta),
                             Math.Sin(beta),
                             Math.Sin(alpha) * Math.Cos(beta));
                double cpMax = NewtonianAero.ModifiedNewtonianCpMax(flow.Mach, flow.Gamma);
                double fx = 0, fy = 0, fz = 0, mx = 0, my = 0, mz = 0;
                s.ForEachTriangle((a, b, c) =>
                {
                    D3 cr = D3.Cross(b - a, c - a);
                    double twoA = cr.Length;
                    if (twoA < 1e-14) return;
                    D3 n = cr * (1.0 / twoA);
                    double cosAng = D3.Dot(dir, n);
                    if (cosAng >= 0) return;                 // shadowed
                    double k = -cpMax * cosAng * cosAng * q * 0.5 * twoA;
                    D3 f = n * k;
                    D3 r = (a + b + c) * (1.0 / 3.0);
                    fx += f.X; fy += f.Y; fz += f.Z;
                    mx += r.Y * f.Z - r.Z * f.Y;
                    my += r.Z * f.X - r.X * f.Z;
                    mz += r.X * f.Y - r.Y * f.X;
                });
                return (new D3(fx, fy, fz), new D3(mx, my, mz));
            }

            // Pitch: central difference in alpha.
            var (Fp, Mp) = LoadsAt(+DeltaRad, 0);
            var (Fm, Mm) = LoadsAt(-DeltaRad, 0);
            double dFz = (Fp.Z - Fm.Z) / (2 * DeltaRad);
            double dMy = (Mp.Y - Mm.Y) / (2 * DeltaRad);
            double xnp = Math.Abs(dFz) > 1e-9 ? -dMy / dFz : cg.X;
            xnp = Math.Clamp(xnp, -0.5 * L, 1.5 * L);

            // Yaw: central difference in sideslip.
            var (Fbp, Mbp) = LoadsAt(0, +DeltaRad);
            var (Fbm, Mbm) = LoadsAt(0, -DeltaRad);
            double dFy = (Fbp.Y - Fbm.Y) / (2 * DeltaRad);
            double dMz = (Mbp.Z - Mbm.Z) / (2 * DeltaRad);
            double cnBody = 0;
            if (Math.Abs(dFy) > 1e-9)
            {
                double xLat = Math.Clamp(dMz / dFy, -0.5 * L, 1.5 * L);
                double cyBeta = Math.Abs(dFy) / (q * Sref);
                cnBody = cyBeta * (xLat - cg.X) / bref;      // + if lateral CP aft of CG
            }

            // ---- Fin contribution and sizing -----------------------------------
            double betaM = Math.Sqrt(Math.Max(flow.Mach * flow.Mach - 1.0, 0.1));
            var set = finSet ?? new FinSet();
            bool hypothetical = fins is null || fins.Count == 0;

            double cnFins = 0;
            double coeffPerArea = 0;   // d(Cn_beta)/d(area per fin), at current positions

            // Fin lift-curve slope with a simple finite-AR correction.
            double CLa(double areaM2, double meanChord)
            {
                double ar = areaM2 > 1e-9 ? areaM2 / (meanChord * meanChord) : 1.0;
                double cla = (4.0 / betaM) * (1.0 - 1.0 / Math.Max(2.0 * ar * betaM, 1.2));
                return Math.Max(cla, 1.2 / betaM);
            }

            double cosC2 = Math.Pow(Math.Cos(set.CantDeg * Math.PI / 180.0), 2.0);

            if (!hypothetical)
            {
                foreach (var fin in fins!)
                {
                    double xf = 0.5 * (fin.LERoot.X + fin.LETip.X) + 0.5 * fin.MeanChordM;
                    double c2 = fin.LERoot.Y == 0 && fin.LETip.Y == 0 ? 1.0 : cosC2; // center fin: no cant
                    double cla = CLa(fin.ExposedAreaM2, fin.MeanChordM);
                    cnFins += FinEfficiency * cla * c2 * (fin.ExposedAreaM2 / Sref) * ((xf - cg.X) / bref);
                    coeffPerArea += FinEfficiency * cla * c2 * (1.0 / Sref) * ((xf - cg.X) / bref);
                }
            }
            else
            {
                // Hypothetical mirrored pair with the current fin settings.
                double cRoot = Math.Max(set.RootChordFrac, 0.02) * L;
                double cTip  = Math.Clamp(set.TaperRatio, 0.05, 1.0) * cRoot;
                double h     = Math.Max(set.HeightFrac, 0.01) * L;
                double cMean = 0.5 * (cRoot + cTip);
                double area  = cMean * h;
                double x0    = L * (1.0 - Math.Clamp(set.TEInsetFrac, 0.0, 0.5)) - cRoot;
                double xf    = x0 + 0.5 * h * Math.Tan(set.SweepDeg * Math.PI / 180.0) + 0.5 * cMean;
                double cla   = CLa(area, cMean);
                coeffPerArea = 2.0 * FinEfficiency * cla * cosC2 * (1.0 / Sref) * ((xf - cg.X) / bref);
            }

            // Required exposed area per fin to reach the target.
            double reqArea = 0, reqHeightFrac = 0;
            bool reqValid = coeffPerArea > 1e-9;
            if (reqValid)
            {
                // coeffPerArea sums d(Cn_beta)/d(area) over all fins, so if every
                // fin gets area S: Cn_beta = cnBody + coeffPerArea * S.
                double need = targetCnBeta - cnBody;
                reqArea = Math.Max(0, need) / coeffPerArea;   // exposed area per fin

                double cRoot = Math.Max(set.RootChordFrac, 0.02) * L;
                double cTip  = Math.Clamp(set.TaperRatio, 0.05, 1.0) * cRoot;
                double cMean = Math.Max(0.5 * (cRoot + cTip), 1e-3);
                reqHeightFrac = (reqArea / cMean) / L;
            }

            return new StabilityInfo
            {
                CgX = cg.X, CgZ = cg.Z,
                XnpPitch = xnp,
                StaticMarginPctL = (xnp - cg.X) / L * 100.0,
                CnBetaBody = cnBody,
                CnBetaFins = cnFins,
                CnBetaTotal = cnBody + cnFins,
                TargetCnBeta = targetCnBeta,
                ReqAreaPerFinM2 = reqArea,
                ReqHeightFrac = reqHeightFrac,
                ReqValid = reqValid,
                FinsHypothetical = hypothetical,
            };
        }
    }
}
