//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - off-design aerodynamics (Modified Newtonian panel method).
//
// The on-design loads come from Taylor-Maccoll, which is only valid when the
// shock is attached at the design Mach. To evaluate the *fixed* geometry at
// arbitrary Mach numbers and angles of attack (i.e. off-design), this module
// uses Modified Newtonian impact theory panel-by-panel:
//
//     Cp = Cp_max * sin^2(delta)   on windward panels,   0 on shadowed panels
//
// where delta is the local surface inclination to the freestream and Cp_max is
// the stagnation pressure coefficient from the Rayleigh-Pitot (normal-shock)
// relation. This is the standard quick method for hypersonic off-design L/D
// trends. Viscous drag reuses the Eckert reference-temperature model.
//

namespace WaveriderForge
{
    public static class NewtonianAero
    {
        /// <summary>
        /// Evaluate the fixed waverider geometry at the given freestream and
        /// angle of attack (radians). Positive AoA increases compression on the
        /// lower surface (more lift).
        /// </summary>
        public static AeroResult Evaluate(WaveriderSurfaces s, FlightState flow, double alpha)
        {
            if (!s.Valid) return AeroResult.Invalid;

            double g    = flow.Gamma;
            double m    = flow.Mach;
            double qInf = flow.DynamicPressure;

            // Freestream and lift unit vectors in the body frame (x fwd, z up).
            D3 dir  = new(Math.Cos(alpha), 0, Math.Sin(alpha));
            D3 lift = new(-Math.Sin(alpha), 0, Math.Cos(alpha));

            double cpMax = ModifiedNewtonianCpMax(m, g);

            // Nose station (most-forward leading-edge point) for running length.
            double xNose = double.PositiveInfinity;
            for (int i = 0; i < s.Lower.Length; i++)
                if (s.Lower[i][0].X < xNose) xNose = s.Lower[i][0].X;

            double Fx = 0, Fy = 0, Fz = 0;   // pressure force
            double Dvisc = 0;

            s.ForEachTriangle((a, b, c) =>
            {
                D3 cr  = D3.Cross(b - a, c - a);
                double twoA = cr.Length;
                if (twoA < 1e-14) return;
                double area = 0.5 * twoA;
                D3 n = cr * (1.0 / twoA);          // outward unit normal

                double cosAng = D3.Dot(dir, n);    // <0 => windward (into the flow)
                double cp = cosAng < 0 ? cpMax * cosAng * cosAng : 0.0;

                // dF = (p_inf - p) n A = -Cp q A n
                double k = -cp * qInf * area;
                Fx += k * n.X; Fy += k * n.Y; Fz += k * n.Z;

                // Viscous contribution (drag along the flow direction).
                double xc  = (a.X + b.X + c.X) / 3.0;
                double len = Math.Max(xc - xNose, 1e-3);
                double pe  = flow.Pressure + cp * qInf;
                Dvisc += AeroPerformance.FrictionDrag(m, flow.Temperature, pe, len, area, g);
            });

            D3 F = new(Fx, Fy, Fz);
            double drag = D3.Dot(F, dir) + Dvisc;
            double Lift = D3.Dot(F, lift);

            double vol   = s.Volume();
            double splan = s.PlanformArea();
            double swet  = s.WettedArea();
            double tau   = splan > 0 ? Math.Pow(vol, 2.0 / 3.0) / splan : 0;

            double cl = Lift / (qInf * splan);
            double cd = drag / (qInf * splan);

            return new AeroResult(true, Lift, drag, cl, cd,
                                  drag - Dvisc, Dvisc, 0,
                                  vol, splan, swet, tau,
                                  s.Field.ConeAngle * 180.0 / Math.PI,
                                  s.Field.SurfaceCp());
        }

        /// <summary>
        /// Stagnation-point pressure coefficient Cp_max = (p0_2 - p_inf)/q_inf
        /// from the Rayleigh-Pitot relation (normal shock at Mach m).
        /// </summary>
        public static double ModifiedNewtonianCpMax(double m, double g)
        {
            if (m <= 1.0)
                return 2.0; // incompressible-like fallback
            double p02OverPinf = GasDynamics.StagPressureRatioAcrossShock(m, g) *
                                 GasDynamics.StagPressureRatio(m, g);
            return (p02OverPinf - 1.0) / (0.5 * g * m * m);
        }
    }
}
