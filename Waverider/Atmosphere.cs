//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - a hypersonic waverider design app built on PicoGK.
//

namespace WaveriderForge
{
    /// <summary>
    /// Freestream flight conditions derived from altitude and Mach number,
    /// using the 1976 U.S. Standard Atmosphere (valid to 86 km geometric).
    /// All quantities are SI (Kelvin, Pascal, kg/m^3, m/s).
    /// </summary>
    public readonly struct FlightState
    {
        public readonly double AltitudeM;       // geometric altitude (m)
        public readonly double Mach;            // freestream Mach number
        public readonly double Temperature;     // static temperature T_inf (K)
        public readonly double Pressure;        // static pressure p_inf (Pa)
        public readonly double Density;         // density rho_inf (kg/m^3)
        public readonly double SpeedOfSound;    // a_inf (m/s)
        public readonly double Velocity;        // V_inf (m/s)
        public readonly double DynamicPressure; // q_inf = 0.5 rho V^2 (Pa)
        public readonly double Viscosity;       // mu_inf (Pa.s), Sutherland
        public readonly double UnitReynolds;    // Re per metre (1/m)
        public readonly double Gamma;           // ratio of specific heats

        public FlightState( double altitudeM,
                            double mach,
                            double t,
                            double p,
                            double rho,
                            double a,
                            double mu,
                            double gamma)
        {
            AltitudeM       = altitudeM;
            Mach            = mach;
            Temperature     = t;
            Pressure        = p;
            Density         = rho;
            SpeedOfSound    = a;
            Velocity        = mach * a;
            DynamicPressure = 0.5 * rho * Velocity * Velocity;
            Viscosity       = mu;
            UnitReynolds    = rho * Velocity / mu;
            Gamma           = gamma;
        }
    }

    public static class Atmosphere
    {
        const double g0  = 9.80665;     // standard gravity (m/s^2)
        const double R   = 287.05287;   // specific gas constant for air (J/kg/K)
        const double Re  = 6356766.0;   // Earth radius for geopotential conversion (m)
        const double GAMMA = 1.4;

        // 1976 U.S. Standard Atmosphere layer base data (geopotential height in m,
        // base temperature in K, lapse rate in K/m).
        static readonly double[] H    = { 0,      11000,  20000,  32000,  47000,  51000,  71000,  84852 };
        static readonly double[] L    = { -0.0065, 0.0,    0.001,  0.0028, 0.0,   -0.0028,-0.002,  0.0   };
        static readonly double[] Tb   = new double[8];
        static readonly double[] Pb   = new double[8];

        static Atmosphere()
        {
            // Precompute base temperatures and pressures at each layer boundary.
            Tb[0] = 288.15;
            Pb[0] = 101325.0;

            for (int i = 1; i < H.Length; i++)
            {
                double dh = H[i] - H[i - 1];
                Tb[i] = Tb[i - 1] + L[i - 1] * dh;

                if (Math.Abs(L[i - 1]) > 1e-12)
                {
                    Pb[i] = Pb[i - 1] *
                            Math.Pow(Tb[i - 1] / Tb[i], g0 / (R * L[i - 1]));
                }
                else
                {
                    Pb[i] = Pb[i - 1] *
                            Math.Exp(-g0 * dh / (R * Tb[i - 1]));
                }
            }
        }

        /// <summary>Sutherland's law for the dynamic viscosity of air.</summary>
        public static double Viscosity(double tempK)
        {
            const double muRef = 1.716e-5;  // Pa.s at T0
            const double T0     = 273.15;   // K
            const double S      = 110.4;    // Sutherland temperature (K)
            return muRef * Math.Pow(tempK / T0, 1.5) * (T0 + S) / (tempK + S);
        }

        /// <summary>
        /// Evaluate the standard atmosphere and combine with the Mach number to
        /// produce a complete freestream <see cref="FlightState"/>.
        /// </summary>
        public static FlightState At(double geometricAltM, double mach)
        {
            // Convert geometric altitude to geopotential altitude.
            double h = Re * geometricAltM / (Re + geometricAltM);

            if (h < 0)        h = 0;
            if (h > H[^1])    h = H[^1]; // clamp to model ceiling (86 km)

            // Find the layer that contains h.
            int layer = 0;
            for (int i = 0; i < H.Length - 1; i++)
            {
                if (h >= H[i]) layer = i;
                else break;
            }

            double dh    = h - H[layer];
            double lapse = L[layer];
            double t, p;

            if (Math.Abs(lapse) > 1e-12)
            {
                t = Tb[layer] + lapse * dh;
                p = Pb[layer] * Math.Pow(Tb[layer] / t, g0 / (R * lapse));
            }
            else
            {
                t = Tb[layer];
                p = Pb[layer] * Math.Exp(-g0 * dh / (R * Tb[layer]));
            }

            double rho = p / (R * t);
            double a   = Math.Sqrt(GAMMA * R * t);
            double mu  = Viscosity(t);

            return new FlightState(geometricAltM, mach, t, p, rho, a, mu, GAMMA);
        }
    }
}
