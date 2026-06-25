//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - a hypersonic waverider design application on PicoGK.
//
// You give it a design Mach number and an altitude; it solves the conical
// (Taylor-Maccoll) flow field, runs a constrained volume-maximizing optimizer
// over the osculating-cone design space, reports the aerodynamic performance,
// and generates a watertight 3D model (STL / OpenVDB) with PicoGK.
//
// Usage:
//   Waverider <mach> <altitudeKm> [options]
//   Waverider --mach=8 --altitude-km=30 --length=22 --view
//
// Options:
//   --mach=<M>             design Mach number
//   --altitude-km=<km>     altitude in kilometres   (or --altitude-m=<m>)
//   --length=<m>           vehicle length, metres            (default 20)
//   --ld-floor=<value>     absolute L/D floor for the optimizer
//   --ld-retention=<0..1>  L/D floor as a fraction of the max achievable (default 0.90)
//   --q-allow-mw=<MW/m^2>  allowable LE stagnation heat flux (default 5)
//   --sharp                sharp leading edge (no blunting)
//   --voxel-mm=<mm>        voxel size override
//   --out=<dir>            output directory                  (default ./output)
//   --view                 open the interactive PicoGK viewer
//

using PicoGK;

namespace WaveriderForge
{
    public static class Program
    {
        public static int Main(string[] args)
        {
            var opt = CliOptions.Parse(args);

            Console.WriteLine();
            Console.WriteLine("  ============================================================");
            Console.WriteLine("    WAVERIDER FORGE  -  hypersonic osculating-cone designer");
            Console.WriteLine("  ============================================================");
            Console.WriteLine();

            if (double.IsNaN(opt.Mach))
                opt.Mach = Prompt("Design Mach number", 8.0, m => m > 1.2 && m < 40);
            if (double.IsNaN(opt.AltitudeM))
                opt.AltitudeM = 1000.0 * Prompt("Altitude (km)", 30.0, a => a >= 0 && a <= 86);

            FlightState flow = Atmosphere.At(opt.AltitudeM, opt.Mach);

            ReportFlight(flow);

            // --- Optimize -------------------------------------------------------
            var seed = new WaveriderDesign { LengthM = opt.LengthM };

            Console.WriteLine("  Optimizing (this can take a moment)...");
            // First pass: find the best achievable L/D so we can anchor the floor.
            var probe = Optimizer.Optimize(seed, flow, 0.0);
            double ldFloor = !double.IsNaN(opt.LDFloor)
                ? opt.LDFloor
                : opt.LDRetention * probe.MaxLDSeen;

            var result = Optimizer.Optimize(seed, flow, ldFloor,
                                            msg => Console.WriteLine("    " + msg));

            if (!result.Aero.Valid)
            {
                Console.WriteLine();
                Console.WriteLine("  No valid waverider could be generated for these inputs.");
                Console.WriteLine("  Try a different Mach/altitude combination.");
                return 1;
            }

            ReportDesign(result, flow, opt);

            // --- Geometry generation with PicoGK --------------------------------
            double leRadiusM = 0;
            if (!opt.Sharp)
            {
                leRadiusM = Heating.LeadingEdgeRadius(flow, opt.QAllowMW * 1.0e6);
                double qAt = Heating.StagHeatFlux(flow, leRadiusM);
                Console.WriteLine($"  Leading edge          : blunted, radius {leRadiusM * 1000:F1} mm");
                Console.WriteLine($"  Stagnation heat flux  : {qAt / 1.0e6:F2} MW/m^2 (at allowable)");
            }
            else
            {
                Console.WriteLine("  Leading edge          : sharp");
            }
            Console.WriteLine();

            double voxelMM = !double.IsNaN(opt.VoxelMM)
                ? opt.VoxelMM
                : Math.Max(opt.LengthM * WaveriderBuilder.MM / 500.0, 1.0);

            Directory.CreateDirectory(opt.OutDir);
            string stem = Path.Combine(opt.OutDir,
                $"waverider_M{opt.Mach:0.0}_{opt.AltitudeM / 1000.0:0}km");

            if (opt.View)
            {
                Library.Go((float)voxelMM, () =>
                    Generate(Library.oLibrary(), result, flow,
                             leRadiusM * WaveriderBuilder.MM, stem, true),
                    strWindowTitle: "Waverider Forge");
            }
            else
            {
                using Library lib = new((float)voxelMM);
                Generate(lib, result, flow, leRadiusM * WaveriderBuilder.MM, stem, false);
            }

            Console.WriteLine();
            Console.WriteLine("  Done.");
            return 0;
        }

        static void Generate(Library lib,
                             OptimizationResult result,
                             FlightState flow,
                             double leRadiusMM,
                             string stem,
                             bool view)
        {
            Console.WriteLine($"  Voxelizing at {lib.fVoxelSize:F1} mm voxels...");
            var surf = new WaveriderSurfaces(result.Design,
                ConicalFlowField.Solve(flow.Mach, result.Design.ShockAngleRad, flow.Gamma));

            Voxels vox = WaveriderBuilder.BuildVoxels(lib, surf, leRadiusMM);

            vox.CalculateProperties(out float volMM3, out _);
            Console.WriteLine($"  Voxel-model volume    : {volMM3 / 1.0e9:F3} m^3");

            string stl = stem + ".stl";
            using (Mesh mshOut = new(vox))
                mshOut.SaveToStlFile(stl, Mesh.EStlUnit.MM);
            Console.WriteLine($"  Wrote {stl}");

            string vdb = stem + ".vdb";
            vox.SaveToVdbFile(vdb);
            Console.WriteLine($"  Wrote {vdb}");

            if (view)
            {
                Library.oViewer().Add(vox);
                Console.WriteLine("  Viewer open - close the window to exit.");
            }
        }

        // ---- Reporting --------------------------------------------------------

        static void ReportFlight(FlightState f)
        {
            Console.WriteLine("  Flight condition (1976 U.S. Standard Atmosphere)");
            Console.WriteLine($"    Mach {f.Mach:F2}  |  altitude {f.AltitudeM / 1000.0:F1} km");
            Console.WriteLine($"    Velocity        : {f.Velocity:F0} m/s");
            Console.WriteLine($"    Static temp     : {f.Temperature:F1} K");
            Console.WriteLine($"    Static pressure : {f.Pressure / 1000.0:F2} kPa");
            Console.WriteLine($"    Density         : {f.Density:E3} kg/m^3");
            Console.WriteLine($"    Dynamic pressure: {f.DynamicPressure / 1000.0:F2} kPa");
            Console.WriteLine($"    Unit Reynolds   : {f.UnitReynolds:E2} 1/m");
            Console.WriteLine();
        }

        static void ReportDesign(OptimizationResult r, FlightState f, CliOptions o)
        {
            var d = r.Design;
            var a = r.Aero;
            Console.WriteLine();
            Console.WriteLine("  ----------------------- Optimized design -----------------------");
            Console.WriteLine($"  Evaluations           : {r.Evaluations}");
            Console.WriteLine($"  Shock angle (beta)    : {d.ShockAngleRad * 180.0 / Math.PI:F2} deg");
            Console.WriteLine($"  Cone angle (theta_c)  : {a.ConeAngleDeg:F2} deg");
            Console.WriteLine($"  Length x span         : {d.LengthM:F2} x {d.WidthM:F2} m");
            Console.WriteLine($"  Shock-curve depth     : {d.CurveDepthRatio:F3} (Hc/b)");
            Console.WriteLine($"  Compression fraction  : {d.CompressionFraction:F3}");
            Console.WriteLine();
            Console.WriteLine("  Aerodynamic performance (per full vehicle)");
            Console.WriteLine($"    Lift / Drag (L/D)   : {a.LiftToDrag:F2}" +
                              (r.MeetsLD ? "" : "   (below requested floor)"));
            Console.WriteLine($"    L/D floor used      : {r.LDFloor:F2}  (max seen {r.MaxLDSeen:F2})");
            Console.WriteLine($"    Lift                : {a.Lift / 1000.0:F1} kN");
            Console.WriteLine($"    Drag                : {a.Drag / 1000.0:F1} kN");
            Console.WriteLine($"      pressure drag     : {a.DragPressure / 1000.0:F1} kN");
            Console.WriteLine($"      friction drag     : {a.DragFriction / 1000.0:F1} kN");
            Console.WriteLine($"      base drag         : {a.DragBase / 1000.0:F1} kN");
            Console.WriteLine($"    C_L                 : {a.CL:F4}");
            Console.WriteLine($"    C_D                 : {a.CD:F4}");
            Console.WriteLine();
            Console.WriteLine("  Geometry");
            Console.WriteLine($"    Volume              : {a.Volume:F3} m^3");
            Console.WriteLine($"    Planform area       : {a.PlanformArea:F2} m^2");
            Console.WriteLine($"    Wetted area         : {a.WettedArea:F2} m^2");
            Console.WriteLine($"    Volumetric eff. tau : {a.Tau:F4}  (V^2/3 / S_plan)");
            Console.WriteLine("  ----------------------------------------------------------------");
            Console.WriteLine();
        }

        static double Prompt(string label, double dflt, Func<double, bool> ok)
        {
            while (true)
            {
                Console.Write($"  {label} [{dflt}]: ");
                string? line = Console.ReadLine();
                if (string.IsNullOrWhiteSpace(line)) return dflt;
                if (double.TryParse(line.Trim(), out double v) && ok(v)) return v;
                Console.WriteLine("    Please enter a valid value.");
            }
        }
    }

    /// <summary>Parsed command-line options.</summary>
    public sealed class CliOptions
    {
        public double Mach        = double.NaN;
        public double AltitudeM   = double.NaN;
        public double LengthM     = 20.0;
        public double LDFloor     = double.NaN;
        public double LDRetention = 0.90;
        public double QAllowMW    = 5.0;
        public double VoxelMM     = double.NaN;
        public bool   Sharp       = false;
        public bool   View        = false;
        public string OutDir      = "output";

        public static CliOptions Parse(string[] args)
        {
            var o = new CliOptions();
            var positional = new List<double>();

            foreach (string arg in args)
            {
                if (arg.StartsWith("--"))
                {
                    string key = arg[2..];
                    string val = "";
                    int eq = key.IndexOf('=');
                    if (eq >= 0) { val = key[(eq + 1)..]; key = key[..eq]; }

                    switch (key)
                    {
                        case "mach":         o.Mach = D(val); break;
                        case "altitude-km":  o.AltitudeM = D(val) * 1000.0; break;
                        case "altitude-m":   o.AltitudeM = D(val); break;
                        case "length":       o.LengthM = D(val); break;
                        case "ld-floor":     o.LDFloor = D(val); break;
                        case "ld-retention": o.LDRetention = D(val); break;
                        case "q-allow-mw":   o.QAllowMW = D(val); break;
                        case "voxel-mm":     o.VoxelMM = D(val); break;
                        case "sharp":        o.Sharp = true; break;
                        case "view":         o.View = true; break;
                        case "out":          o.OutDir = val; break;
                    }
                }
                else if (double.TryParse(arg, out double v))
                {
                    positional.Add(v);
                }
            }

            if (double.IsNaN(o.Mach) && positional.Count >= 1) o.Mach = positional[0];
            if (double.IsNaN(o.AltitudeM) && positional.Count >= 2) o.AltitudeM = positional[1] * 1000.0;
            return o;
        }

        static double D(string s) => double.TryParse(s, out double v) ? v : double.NaN;
    }
}
