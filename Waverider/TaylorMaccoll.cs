//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - Taylor-Maccoll conical flow solver.
//
// Solves the Taylor-Maccoll ordinary differential equation for steady,
// inviscid, axisymmetric supersonic flow over a sharp cone. Given the
// freestream Mach number and a chosen conical shock angle (beta), it returns
// the cone half-angle (theta_c), the post-shock conical flow field, the
// self-similar streamline shape used for streamline tracing, and surface
// pressure (Cp) anywhere between the shock and the cone surface.
//

namespace WaveriderForge
{
    /// <summary>
    /// A converged axisymmetric conical flow field behind a conical shock of
    /// angle <see cref="ShockAngle"/>, for freestream Mach <see cref="MachInf"/>.
    /// </summary>
    public sealed class ConicalFlowField
    {
        public double MachInf    { get; }
        public double ShockAngle { get; }   // beta (rad)
        public double ConeAngle  { get; }   // theta_c (rad)
        public double Gamma      { get; }
        public bool   Valid      { get; }

        // Tabulated solution, ascending in theta from cone surface (index 0)
        // to shock (last index). Velocities are normalised by V_max.
        readonly double[] m_theta;
        readonly double[] m_Vr;
        readonly double[] m_Vtheta;
        readonly double[] m_lnRStream;   // ln(r/r_shock) cumulative along a streamline
        readonly double   m_p0_2_over_pInf;

        ConicalFlowField(   double machInf,
                            double beta,
                            double gamma,
                            bool valid,
                            double thetaC,
                            double[] theta,
                            double[] vr,
                            double[] vtheta,
                            double[] lnR,
                            double p0_2_over_pInf)
        {
            MachInf          = machInf;
            ShockAngle       = beta;
            Gamma            = gamma;
            Valid            = valid;
            ConeAngle        = thetaC;
            m_theta          = theta;
            m_Vr             = vr;
            m_Vtheta         = vtheta;
            m_lnRStream      = lnR;
            m_p0_2_over_pInf = p0_2_over_pInf;
        }

        static double SoundSpeedSq(double vr, double vt, double g)
            => 0.5 * (g - 1.0) * (1.0 - vr * vr - vt * vt);

        /// <summary>
        /// Solve the Taylor-Maccoll equation for the given freestream Mach number
        /// and conical shock angle. Returns an invalid field if the shock is
        /// weaker than the Mach wave or fails to close on a cone surface.
        /// </summary>
        public static ConicalFlowField Solve(double machInf, double beta, double gamma)
        {
            double[] empty = Array.Empty<double>();

            if (beta <= GasDynamics.MachAngle(machInf) || beta >= Math.PI / 2.0)
                return new ConicalFlowField(machInf, beta, gamma, false, 0,
                                            empty, empty, empty, empty, 0);

            // --- Post-shock initial condition at theta = beta -------------------
            double delta = GasDynamics.DeflectionAngle(machInf, beta, gamma);
            if (delta <= 0)
                return new ConicalFlowField(machInf, beta, gamma, false, 0,
                                            empty, empty, empty, empty, 0);

            double m2    = GasDynamics.ObliqueDownstreamMach(machInf, beta, gamma);
            double vMag  = 1.0 / Math.Sqrt(1.0 + 2.0 / ((gamma - 1.0) * m2 * m2));

            double vr =  vMag * Math.Cos(beta - delta);
            double vt = -vMag * Math.Sin(beta - delta);

            // Total pressure behind the (oblique) shock, relative to freestream.
            double mn1     = machInf * Math.Sin(beta);
            double p0Ratio = GasDynamics.StagPressureRatioAcrossShock(mn1, gamma);
            double p0_2_over_pInf = p0Ratio * GasDynamics.StagPressureRatio(machInf, gamma);

            // --- Integrate inward (decreasing theta) with RK4 -------------------
            const int    maxSteps = 200000;
            double       dTheta   = -1.0e-5;      // rad, marching toward the axis
            double       theta    = beta;

            var lTheta = new List<double>();
            var lVr    = new List<double>();
            var lVt    = new List<double>();

            lTheta.Add(theta); lVr.Add(vr); lVt.Add(vt);

            bool   found  = false;
            double thetaC = 0;

            for (int step = 0; step < maxSteps && theta > 1.0e-4; step++)
            {
                double prevVt = vt;
                double prevTh = theta;

                Rk4Step(ref vr, ref vt, theta, dTheta, gamma);
                theta += dTheta;

                // Cone surface is where the radial flow becomes parallel to the
                // cone, i.e. the normal velocity component V_theta reaches zero.
                // Stop before storing the overshoot point so the tabulated grid
                // keeps V_theta strictly negative (no singular streamline factor).
                if (vt >= 0.0)
                {
                    double frac = prevVt / (prevVt - vt);   // linear interp to Vt=0
                    thetaC = prevTh + frac * dTheta;
                    found  = true;
                    break;
                }

                lTheta.Add(theta); lVr.Add(vr); lVt.Add(vt);
            }

            if (!found)
                return new ConicalFlowField(machInf, beta, gamma, false, 0,
                                            empty, empty, empty, empty, 0);

            // Reverse to ascending theta (cone surface -> shock) for clean lookup.
            lTheta.Reverse(); lVr.Reverse(); lVt.Reverse();
            double[] aTheta = lTheta.ToArray();
            double[] aVr    = lVr.ToArray();
            double[] aVt    = lVt.ToArray();

            // Cumulative streamline factor ln(r/r_shock): d(ln r)/dtheta = Vr/Vtheta.
            // Integrated from the shock (last index, value 0) toward the cone.
            int n = aTheta.Length;
            double[] lnR = new double[n];
            lnR[n - 1] = 0.0;
            for (int i = n - 2; i >= 0; i--)
            {
                double f1 = aVr[i]     / aVt[i];
                double f2 = aVr[i + 1] / aVt[i + 1];
                double dth = aTheta[i] - aTheta[i + 1];   // negative
                lnR[i] = lnR[i + 1] + 0.5 * (f1 + f2) * dth;
            }

            return new ConicalFlowField(machInf, beta, gamma, true, thetaC,
                                        aTheta, aVr, aVt, lnR, p0_2_over_pInf);
        }

        static void Rk4Step(ref double vr, ref double vt, double theta, double h, double g)
        {
            Deriv(vr, vt, theta, g, out double k1r, out double k1t);
            Deriv(vr + 0.5 * h * k1r, vt + 0.5 * h * k1t, theta + 0.5 * h, g, out double k2r, out double k2t);
            Deriv(vr + 0.5 * h * k2r, vt + 0.5 * h * k2t, theta + 0.5 * h, g, out double k3r, out double k3t);
            Deriv(vr + h * k3r, vt + h * k3t, theta + h, g, out double k4r, out double k4t);

            vr += h / 6.0 * (k1r + 2.0 * k2r + 2.0 * k3r + k4r);
            vt += h / 6.0 * (k1t + 2.0 * k2t + 2.0 * k3t + k4t);
        }

        static void Deriv(double vr, double vt, double theta, double g,
                          out double dVr, out double dVt)
        {
            double a2 = SoundSpeedSq(vr, vt, g);
            dVr = vt;
            double denom = a2 - vt * vt;
            if (Math.Abs(denom) < 1e-12) denom = denom < 0 ? -1e-12 : 1e-12;
            dVt = (vr * vt * vt - a2 * (2.0 * vr + vt / Math.Tan(theta))) / denom;
        }

        // Linear interpolation index helper on the ascending theta grid.
        int Bracket(double theta, out double frac)
        {
            if (theta <= m_theta[0]) { frac = 0; return 0; }
            int last = m_theta.Length - 1;
            if (theta >= m_theta[last]) { frac = 1; return last - 1; }

            int lo = 0, hi = last;
            while (hi - lo > 1)
            {
                int mid = (lo + hi) / 2;
                if (m_theta[mid] <= theta) lo = mid; else hi = mid;
            }
            frac = (theta - m_theta[lo]) / (m_theta[lo + 1] - m_theta[lo]);
            return lo;
        }

        /// <summary>
        /// Streamline radius relative to its shock-crossing radius, R(theta) =
        /// r(theta)/r_shock, for the self-similar conical streamline. Equals 1 at
        /// the shock (theta = beta) and grows toward the cone surface.
        /// </summary>
        public double StreamlineRadiusFactor(double theta)
        {
            int i = Bracket(theta, out double f);
            double lnR = m_lnRStream[i] * (1 - f) + m_lnRStream[i + 1] * f;
            return Math.Exp(lnR);
        }

        /// <summary>Local Mach number at polar angle theta within the cone field.</summary>
        public double LocalMach(double theta)
        {
            int i = Bracket(theta, out double f);
            double vr = m_Vr[i] * (1 - f) + m_Vr[i + 1] * f;
            double vt = m_Vtheta[i] * (1 - f) + m_Vtheta[i + 1] * f;
            double a2 = SoundSpeedSq(vr, vt, Gamma);
            double v2 = vr * vr + vt * vt;
            return Math.Sqrt(v2 / a2);
        }

        /// <summary>
        /// Static pressure coefficient Cp at polar angle theta on a compression
        /// streamline of this conical field.
        /// </summary>
        public double Cp(double theta)
        {
            double m   = LocalMach(theta);
            double pp0 = GasDynamics.StaticPressureRatio(m, Gamma);
            double pInf = pp0 * m_p0_2_over_pInf;        // p / p_inf
            return (pInf - 1.0) / (0.5 * Gamma * MachInf * MachInf);
        }

        /// <summary>Cp on the cone surface (theta = theta_c).</summary>
        public double SurfaceCp() => Cp(ConeAngle);
    }
}
