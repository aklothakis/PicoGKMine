//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - aerodynamic performance estimation.
//
// Inviscid forces come from integrating the Taylor-Maccoll surface pressure
// over the compression (lower) surface; the upper surface is a freestream
// surface (Cp = 0); a simple base-pressure model closes the drag. Viscous drag
// uses Eckert's reference-temperature method, which the literature reports as
// the most accurate of the engineering-level corrections for waveriders.
//

namespace WaveriderForge
{
    public readonly struct AeroResult
    {
        public readonly bool   Valid;
        public readonly double Lift, Drag;          // N (per full vehicle)
        public readonly double LiftToDrag;
        public readonly double CL, CD;
        public readonly double DragPressure, DragFriction, DragBase;
        public readonly double Volume, PlanformArea, WettedArea, Tau;
        public readonly double ConeAngleDeg, ConeSurfaceCp;

        public AeroResult(bool valid, double lift, double drag, double cl, double cd,
                          double dP, double dF, double dB,
                          double vol, double splan, double swet, double tau,
                          double coneDeg, double coneCp)
        {
            Valid = valid; Lift = lift; Drag = drag;
            LiftToDrag = drag > 1e-9 ? lift / drag : 0;
            CL = cl; CD = cd;
            DragPressure = dP; DragFriction = dF; DragBase = dB;
            Volume = vol; PlanformArea = splan; WettedArea = swet; Tau = tau;
            ConeAngleDeg = coneDeg; ConeSurfaceCp = coneCp;
        }

        public static AeroResult Invalid => new AeroResult(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public static class AeroPerformance
    {
        const double Pr = 0.71;
        const double Rair = 287.05287;

        public static AeroResult Evaluate(WaveriderSurfaces s, FlightState flow)
        {
            if (!s.Valid) return AeroResult.Invalid;

            double g    = flow.Gamma;
            double qInf = flow.DynamicPressure;
            double T0   = flow.Temperature * (1.0 + 0.5 * (g - 1.0) * flow.Mach * flow.Mach);

            int Ns = s.Design.Nspan, Nc = s.Design.Nchord;

            double Fx = 0.0, Fz = 0.0;   // pressure force components
            double Dvisc = 0.0;

            // --- Lower (compression) surface ----------------------------------
            for (int i = 0; i < Ns - 1; i++)
            {
                double xLE = s.Lower[i][0].X;
                for (int t = 0; t < Nc; t++)
                {
                    D3 a = s.Lower[i][t], b = s.Lower[i + 1][t],
                       c = s.Lower[i + 1][t + 1], d = s.Lower[i][t + 1];

                    double thAvg = 0.25 * (s.Theta[i][t] + s.Theta[i + 1][t] +
                                           s.Theta[i + 1][t + 1] + s.Theta[i][t + 1]);
                    double cp = s.Field.Cp(thAvg);

                    AccumPanelPressure(a, b, c, cp, qInf, ref Fx, ref Fz);
                    AccumPanelPressure(a, c, d, cp, qInf, ref Fx, ref Fz);

                    // Viscous: edge conditions on the compression surface.
                    double me = s.Field.LocalMach(thAvg);
                    double pe = flow.Pressure + cp * qInf;
                    double te = T0 / (1.0 + 0.5 * (g - 1.0) * me * me);
                    double xc = 0.25 * (a.X + b.X + c.X + d.X);
                    double len = Math.Max(xc - xLE, 1e-3);
                    double area = TriArea(a, b, c) + TriArea(a, c, d);
                    Dvisc += FrictionDrag(me, te, pe, len, area, g);
                }
            }

            // --- Upper (freestream) surface: pressure ~ p_inf, friction only ---
            for (int i = 0; i < Ns - 1; i++)
            {
                double xLE = s.Upper[i][0].X;
                for (int t = 0; t < Nc; t++)
                {
                    D3 a = s.Upper[i][t], b = s.Upper[i][t + 1],
                       c = s.Upper[i + 1][t + 1], d = s.Upper[i + 1][t];
                    double xc = 0.25 * (a.X + b.X + c.X + d.X);
                    double len = Math.Max(xc - xLE, 1e-3);
                    double area = TriArea(a, b, c) + TriArea(a, c, d);
                    Dvisc += FrictionDrag(flow.Mach, flow.Temperature, flow.Pressure, len, area, g);
                }
            }

            double Dpress = Fx;
            double Lift   = Fz;

            // --- Base drag ------------------------------------------------------
            double baseArea = 0.0;
            for (int i = 0; i < Ns - 1; i++)
            {
                D3 a = s.Lower[i][Nc], b = s.Lower[i + 1][Nc],
                   c = s.Upper[i + 1][Nc], d = s.Upper[i][Nc];
                baseArea += ProjYZArea(a, b, c) + ProjYZArea(a, c, d);
            }
            double cpBase = -1.0 / (flow.Mach * flow.Mach);
            double Dbase  = -cpBase * qInf * baseArea;

            double Drag = Dpress + Dvisc + Dbase;

            double vol   = s.Volume();
            double splan = s.PlanformArea();
            double swet  = s.WettedArea();
            double tau   = splan > 0 ? Math.Pow(vol, 2.0 / 3.0) / splan : 0;

            double cl = Lift / (qInf * splan);
            double cd = Drag / (qInf * splan);

            return new AeroResult(true, Lift, Drag, cl, cd,
                                  Dpress, Dvisc, Dbase,
                                  vol, splan, swet, tau,
                                  s.Field.ConeAngle * 180.0 / Math.PI,
                                  s.Field.SurfaceCp());
        }

        static void AccumPanelPressure(D3 a, D3 b, D3 c, double cp, double qInf,
                                       ref double Fx, ref double Fz)
        {
            D3 n = D3.Cross(b - a, c - a);   // outward (downward) * 2 * area
            // dF = (p_inf - p) * n_hat * A = -cp*qInf * (n/2)  [since |n| = 2A]
            double k = -cp * qInf * 0.5;
            Fx += k * n.X;
            Fz += k * n.Z;
        }

        static double TriArea(D3 a, D3 b, D3 c) => 0.5 * D3.Cross(b - a, c - a).Length;

        static double ProjYZArea(D3 a, D3 b, D3 c)
        {
            // Area of the triangle projected onto the base (y-z) plane.
            double cross = (b.Y - a.Y) * (c.Z - a.Z) - (b.Z - a.Z) * (c.Y - a.Y);
            return 0.5 * Math.Abs(cross);
        }

        /// <summary>
        /// Skin-friction drag of a panel using Eckert's reference-temperature
        /// method with an adiabatic (radiative) wall assumption.
        /// </summary>
        public static double FrictionDrag(double me, double te, double pe,
                                          double length, double area, double g)
        {
            if (me < 1e-3 || te <= 0) return 0;

            double ue   = me * Math.Sqrt(g * Rair * te);
            double rhoE = pe / (Rair * te);
            double qe   = 0.5 * rhoE * ue * ue;

            // Recovery (adiabatic-wall) temperature; assume Tw = Tr.
            double rTurb = Math.Pow(Pr, 1.0 / 3.0);
            double tw    = te * (1.0 + rTurb * 0.5 * (g - 1.0) * me * me);

            // Reference temperature (Meador-Smart / Eckert).
            double tstar  = te * (1.0 + 0.032 * me * me + 0.58 * (tw / te - 1.0));
            double rhoS   = pe / (Rair * tstar);
            double muS    = Atmosphere.Viscosity(tstar);
            double reStar = rhoS * ue * length / muS;
            if (reStar < 1.0) return 0;

            // Local skin-friction coefficient (laminar below transition).
            double cfStar = reStar < 5.0e5
                ? 0.664 / Math.Sqrt(reStar)
                : 0.0592 / Math.Pow(reStar, 0.2);

            // Reference-temperature scaling of dynamic pressure (rho* / rho_e).
            double tauW = cfStar * qe * (rhoS / rhoE);
            return tauW * area;
        }
    }
}
