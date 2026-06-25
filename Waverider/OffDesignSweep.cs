//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - off-design performance sweeps.
//
// Holds the optimized geometry fixed and sweeps the flight condition to show
// how lift-to-drag and the force coefficients vary away from the design point:
//   * a Mach sweep at the design altitude and zero angle of attack, and
//   * an angle-of-attack sweep at the design Mach and altitude.
// Results print as tables and are written to CSV for plotting.
//

namespace WaveriderForge
{
    public readonly struct SweepRow
    {
        public readonly double X;        // Mach number or AoA (deg) depending on sweep
        public readonly AeroResult Aero;
        public SweepRow(double x, AeroResult a) { X = x; Aero = a; }
    }

    public static class OffDesignSweep
    {
        /// <summary>Sweep Mach number at fixed altitude, zero AoA.</summary>
        public static SweepRow[] MachSweep(WaveriderSurfaces s, double altM,
                                           double mMin, double mMax, int n, double gamma)
        {
            var rows = new List<SweepRow>(n);
            for (int i = 0; i < n; i++)
            {
                double mach = mMin + (mMax - mMin) * i / Math.Max(1, n - 1);
                var flow = Atmosphere.At(altM, mach);
                rows.Add(new SweepRow(mach, NewtonianAero.Evaluate(s, flow, 0.0)));
            }
            return rows.ToArray();
        }

        /// <summary>Sweep angle of attack at the design Mach and altitude.</summary>
        public static SweepRow[] AoASweep(WaveriderSurfaces s, FlightState design,
                                          double aMinDeg, double aMaxDeg, int n)
        {
            var rows = new List<SweepRow>(n);
            for (int i = 0; i < n; i++)
            {
                double aDeg = aMinDeg + (aMaxDeg - aMinDeg) * i / Math.Max(1, n - 1);
                var aero = NewtonianAero.Evaluate(s, design, aDeg * Math.PI / 180.0);
                rows.Add(new SweepRow(aDeg, aero));
            }
            return rows.ToArray();
        }

        // ---- Reporting ---------------------------------------------------------

        public static void PrintMachTable(SweepRow[] rows, double designMach)
        {
            Console.WriteLine("  Off-design Mach sweep (design altitude, AoA = 0, Modified Newtonian)");
            Console.WriteLine("    Mach     L/D       C_L       C_D     note");
            Console.WriteLine("    --------------------------------------------------");
            foreach (var r in rows)
                Console.WriteLine($"   {r.X,5:F2}   {r.Aero.LiftToDrag,6:F2}   " +
                                  $"{r.Aero.CL,7:F4}   {r.Aero.CD,7:F4}   {NearDesign(r.X, designMach)}");
            Console.WriteLine();
        }

        static string NearDesign(double x, double design)
            => Math.Abs(x - design) <= (design * 0.06) ? "(near design)" : "";

        public static void PrintAoATable(SweepRow[] rows)
        {
            Console.WriteLine("  Angle-of-attack sweep (design Mach & altitude, Modified Newtonian)");
            Console.WriteLine("    AoA(deg)   L/D       C_L       C_D");
            Console.WriteLine("    --------------------------------------------");
            double bestLD = double.NegativeInfinity, bestA = 0;
            foreach (var r in rows)
            {
                Console.WriteLine($"    {r.X,6:F1}    {r.Aero.LiftToDrag,6:F2}   " +
                                  $"{r.Aero.CL,7:F4}   {r.Aero.CD,7:F4}");
                if (r.Aero.LiftToDrag > bestLD) { bestLD = r.Aero.LiftToDrag; bestA = r.X; }
            }
            Console.WriteLine($"    Best L/D = {bestLD:F2} at AoA = {bestA:F1} deg");
            Console.WriteLine();
        }

        public static void WriteCsv(string path, string xLabel, SweepRow[] rows,
                                    double altKm, double fixedMachOrNaN)
        {
            using var w = new StreamWriter(path);
            w.WriteLine($"{xLabel},altitude_km,LD,CL,CD,Lift_kN,Drag_kN,Cone_deg");
            foreach (var r in rows)
                w.WriteLine(string.Join(",",
                    r.X.ToString("0.####"),
                    altKm.ToString("0.###"),
                    r.Aero.LiftToDrag.ToString("0.####"),
                    r.Aero.CL.ToString("0.######"),
                    r.Aero.CD.ToString("0.######"),
                    (r.Aero.Lift / 1000.0).ToString("0.###"),
                    (r.Aero.Drag / 1000.0).ToString("0.###"),
                    r.Aero.ConeAngleDeg.ToString("0.###")));
        }
    }
}
