//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - compressible-flow relations used by the design tools.
//

namespace WaveriderForge
{
    /// <summary>
    /// Perfect-gas compressible-flow relations: isentropic, normal shock and
    /// oblique shock (theta-beta-Mach). Angles are in radians unless noted.
    /// </summary>
    public static class GasDynamics
    {
        /// <summary>Mach angle mu = asin(1/M).</summary>
        public static double MachAngle(double mach) => Math.Asin(1.0 / mach);

        /// <summary>Isentropic ratio p0/p for a given Mach number.</summary>
        public static double StagPressureRatio(double mach, double g)
            => Math.Pow(1.0 + 0.5 * (g - 1.0) * mach * mach, g / (g - 1.0));

        /// <summary>Isentropic static-pressure ratio p/p0 for a given Mach number.</summary>
        public static double StaticPressureRatio(double mach, double g)
            => 1.0 / StagPressureRatio(mach, g);

        /// <summary>Isentropic temperature ratio T0/T.</summary>
        public static double StagTemperatureRatio(double mach, double g)
            => 1.0 + 0.5 * (g - 1.0) * mach * mach;

        /// <summary>
        /// Flow deflection angle (delta) produced by an oblique shock for a given
        /// freestream Mach number and shock wave angle (beta). theta-beta-M relation.
        /// </summary>
        public static double DeflectionAngle(double mach, double beta, double g)
        {
            double m2  = mach * mach;
            double num = 2.0 / Math.Tan(beta) * (m2 * Math.Sin(beta) * Math.Sin(beta) - 1.0);
            double den = m2 * (g + Math.Cos(2.0 * beta)) + 2.0;
            return Math.Atan(num / den);
        }

        /// <summary>
        /// Static pressure ratio p2/p1 across an oblique shock (uses the
        /// shock-normal Mach number Mn1 = M1 sin(beta)).
        /// </summary>
        public static double ObliquePressureRatio(double mach, double beta, double g)
        {
            double mn1 = mach * Math.Sin(beta);
            return 1.0 + 2.0 * g / (g + 1.0) * (mn1 * mn1 - 1.0);
        }

        /// <summary>Downstream Mach number behind an oblique shock.</summary>
        public static double ObliqueDownstreamMach(double mach, double beta, double g)
        {
            double delta = DeflectionAngle(mach, beta, g);
            double mn1   = mach * Math.Sin(beta);
            double mn2   = Math.Sqrt((1.0 + 0.5 * (g - 1.0) * mn1 * mn1) /
                                     (g * mn1 * mn1 - 0.5 * (g - 1.0)));
            return mn2 / Math.Sin(beta - delta);
        }

        /// <summary>
        /// Total (stagnation) pressure ratio p0_2/p0_1 across a normal shock of
        /// upstream Mach number Mn.
        /// </summary>
        public static double StagPressureRatioAcrossShock(double mn, double g)
        {
            double a = (g + 1.0) * mn * mn / (2.0 + (g - 1.0) * mn * mn);
            double b = (g + 1.0) / (2.0 * g * mn * mn - (g - 1.0));
            return Math.Pow(a, g / (g - 1.0)) * Math.Pow(b, 1.0 / (g - 1.0));
        }

        /// <summary>
        /// Minimum shock angle for which an attached oblique shock exists at this
        /// Mach number, i.e. the Mach angle. Below this there is no compression.
        /// </summary>
        public static double MinShockAngle(double mach) => MachAngle(mach);

        /// <summary>
        /// Maximum oblique-shock angle before the deflection turns back over
        /// (the point of maximum deflection / shock detachment boundary).
        /// Found numerically by scanning beta.
        /// </summary>
        public static double ShockAngleOfMaxDeflection(double mach, double g)
        {
            double muMin  = MachAngle(mach);
            double best   = muMin;
            double bestD  = -1;
            int    n      = 400;
            for (int i = 1; i < n; i++)
            {
                double beta  = muMin + (Math.PI / 2.0 - muMin) * i / n;
                double delta = DeflectionAngle(mach, beta, g);
                if (delta > bestD)
                {
                    bestD = delta;
                    best  = beta;
                }
            }
            return best;
        }
    }
}
