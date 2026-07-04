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
//   Waverider <mach> <altitudeKm> [lengthM] [options]
//   Waverider --mach=8 --altitude-km=30 --length=6 --span=5 --view
//
// Options:
//   --mach=<M>             design Mach number
//   --altitude-km=<km>     altitude in kilometres   (or --altitude-m=<m>)
//   --length=<m>           vehicle length, metres            (default 20)
//   --span=<m>             fix the full span; omit to let the optimizer choose
//   --box=LxWxH            fit inside a length x width x height envelope (metres);
//                          maximizes enclosed volume at the L/D floor and
//                          overrides --length/--span
//   --ld-floor=<value>     absolute L/D floor for the optimizer
//   --ld-retention=<0..1>  L/D floor as a fraction of the max achievable (default 0.90)
//   --q-allow-mw=<MW/m^2>  allowable LE heat flux for the fillet recommendation (default 5)
//   --fillet               fillet the leading edge at the recommended radius
//   --fillet-mm=<r>        fillet the leading edge at a specific radius (mm)
//                          (default is a sharp leading edge; --sharp forces it)
//   --fins                 add a mirrored pair of diamond-airfoil fins
//   --center-fin           add a centerline fin
//   --fin-chord=<%L> --fin-taper=<r> --fin-height=<%L> --fin-sweep=<deg>
//   --fin-cant=<deg> --fin-pos=<%b/2> --fin-te-inset=<%L> --fin-thick=<%c>
//   --fin-radius-mm=<r>    fin leading-edge bluntness (0 = sharp)
//   --voxel-mm=<mm>        voxel size override
//   --out=<dir>            output directory                  (default ./output)
//   --view                 open the interactive PicoGK viewer
//   --sweep                run off-design Mach & AoA sweeps (CSV + tables)
//   --sweep-mach=min:max:count   custom Mach sweep range
//   --sweep-aoa=min:max:count    custom angle-of-attack sweep range (deg)
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
            if (!opt.HasBox && double.IsNaN(opt.LengthM))
                opt.LengthM = Prompt("Vehicle length (m)", 20.0, l => l > 0.05 && l < 200);

            FlightState flow = Atmosphere.At(opt.AltitudeM, opt.Mach);

            ReportFlight(flow);

            // --- Optimize -------------------------------------------------------
            double? fixedSpan = double.IsNaN(opt.SpanM) ? null : opt.SpanM;
            var seed = new WaveriderDesign
            {
                LengthM = opt.HasBox ? 1.0 : opt.LengthM,
                WidthM  = opt.HasBox ? 1.0 : (fixedSpan ?? opt.LengthM),
            };

            Console.WriteLine("  Optimizing (this can take a moment)...");

            OptimizationResult result;
            if (opt.HasBox)
            {
                Console.WriteLine($"  Fitting inside box    : {opt.BoxL:F2} x {opt.BoxW:F2} x {opt.BoxH:F2} m (LxWxH)");
                var probe = Optimizer.OptimizeInBox(seed, flow, opt.BoxL, opt.BoxW, opt.BoxH, 0.0);
                double ldFloor = !double.IsNaN(opt.LDFloor)
                    ? opt.LDFloor : opt.LDRetention * probe.MaxLDSeen;
                result = Optimizer.OptimizeInBox(seed, flow, opt.BoxL, opt.BoxW, opt.BoxH,
                                                 ldFloor, msg => Console.WriteLine("    " + msg));
            }
            else
            {
                // First pass: find the best achievable L/D so we can anchor the floor.
                var probe = Optimizer.Optimize(seed, flow, 0.0, null, fixedSpan);
                double ldFloor = !double.IsNaN(opt.LDFloor)
                    ? opt.LDFloor : opt.LDRetention * probe.MaxLDSeen;
                result = Optimizer.Optimize(seed, flow, ldFloor,
                                            msg => Console.WriteLine("    " + msg),
                                            fixedSpan);
            }

            if (!result.Aero.Valid)
            {
                Console.WriteLine();
                Console.WriteLine("  No valid waverider could be generated for these inputs.");
                Console.WriteLine("  Try a different Mach/altitude combination.");
                return 1;
            }

            ReportDesign(result, flow, opt);

            var surfFinal = new WaveriderSurfaces(result.Design,
                ConicalFlowField.Solve(flow.Mach, result.Design.ShockAngleRad, flow.Gamma));

            // Report the actual bounding box (gives the height, and the box fit).
            if (surfFinal.Valid)
            {
                surfFinal.Extents(out double ex, out double ey, out double ez);
                if (opt.HasBox)
                    Console.WriteLine($"  Box envelope target   : {opt.BoxL:F2} x {opt.BoxW:F2} x {opt.BoxH:F2} m (LxWxH)");
                Console.WriteLine($"  Actual bounding box   : {ex:F2} x {ey:F2} x {ez:F2} m (LxWxH)");
                Console.WriteLine();
            }

            // Fins (diamond airfoil), if requested.
            FinSet? finSet = opt.oFinSet();
            List<Fin>? fins = null;
            if (finSet != null && surfFinal.Valid)
            {
                fins = FinGeometry.Build(surfFinal, finSet);
                double dFins = 0;
                foreach (var fin in fins) dFins += FinGeometry.DragNewtons(flow, fin);
                double ldWith = (result.Aero.Drag + dFins) > 1e-9
                    ? result.Aero.Lift / (result.Aero.Drag + dFins) : 0;
                Console.WriteLine($"  Fins                  : {fins.Count} x diamond airfoil, " +
                                  $"est. drag {dFins / 1000.0:F1} kN, L/D incl. fins {ldWith:F2}");
                Console.WriteLine();
            }

            // Static stability + fin sizing recommendation.
            if (surfFinal.Valid && result.Aero.Valid)
            {
                var stab = Stability.Analyze(surfFinal, flow, result.Aero, fins, finSet);
                foreach (string line in stab.Describe(result.Design.LengthM)
                             .Split('\n', StringSplitOptions.RemoveEmptyEntries))
                    Console.WriteLine("  " + line.TrimEnd());
                Console.WriteLine();
            }

            Directory.CreateDirectory(opt.OutDir);
            string stem = Path.Combine(opt.OutDir,
                $"waverider_M{opt.Mach:0.0}_{opt.AltitudeM / 1000.0:0}km");

            // --- Off-design performance sweep -----------------------------------
            if (opt.Sweep)
                RunSweeps(result, flow, opt, stem);

            // --- Leading edge: sharp by default, optional fillet ----------------
            double recM = Heating.LeadingEdgeRadius(flow, opt.QAllowMW * 1.0e6);
            double leRadiusM = 0;
            if (opt.FilletLE)
            {
                leRadiusM = double.IsNaN(opt.FilletMM) ? recM : opt.FilletMM / 1000.0;
                double qAt = Heating.StagHeatFlux(flow, leRadiusM);
                Console.WriteLine($"  Leading edge          : filleted, radius {leRadiusM * 1000:F1} mm");
                Console.WriteLine($"  Stagnation heat flux  : {qAt / 1.0e6:F2} MW/m^2" +
                                  (qAt > opt.QAllowMW * 1.0e6 * 1.001 ? $"  EXCEEDS allowable {opt.QAllowMW:F1}" : ""));
            }
            else
            {
                Console.WriteLine("  Leading edge          : sharp");
            }
            Console.WriteLine($"  Recommended fillet    : >= {recM * 1000:F1} mm to keep q <= {opt.QAllowMW:F1} MW/m^2");
            Console.WriteLine();

            // Size voxels from the ACTUAL design length (in box mode opt.LengthM
            // is unset; the real length is result.Design.LengthM).
            double designLenM = result.Design.LengthM;
            double voxelMM = !double.IsNaN(opt.VoxelMM)
                ? opt.VoxelMM
                : WaveriderJob.AutoVoxelMM(designLenM);
            if (double.IsNaN(voxelMM) || voxelMM <= 0) voxelMM = 1.0;   // never pass NaN to the kernel

            if (opt.View)
            {
                Library.Go((float)voxelMM, () =>
                    Generate(Library.oLibrary(), surfFinal, fins,
                             leRadiusM * WaveriderBuilder.MM, stem, true),
                    strWindowTitle: "Waverider Forge");
            }
            else
            {
                using Library lib = new((float)voxelMM);
                Generate(lib, surfFinal, fins, leRadiusM * WaveriderBuilder.MM, stem, false);
            }

            Console.WriteLine();
            Console.WriteLine("  Done.");
            return 0;
        }

        static void RunSweeps(OptimizationResult result, FlightState flow, CliOptions opt, string stem)
        {
            // The geometry is fixed; rebuild it once (no PicoGK needed for sweeps).
            var field = ConicalFlowField.Solve(flow.Mach, result.Design.ShockAngleRad, flow.Gamma);
            var surf  = new WaveriderSurfaces(result.Design, field);
            if (!surf.Valid) { Console.WriteLine("  (sweep skipped: invalid geometry)"); return; }

            double altKm = opt.AltitudeM / 1000.0;

            // Mach sweep: default 0.35..1.4 x design Mach.
            (double mMin, double mMax, int mN) = ParseRange(
                opt.SweepMach,
                Math.Max(1.5, 0.35 * flow.Mach), 1.4 * flow.Mach, 14);

            var machRows = OffDesignSweep.MachSweep(surf, opt.AltitudeM, mMin, mMax, mN, flow.Gamma);
            OffDesignSweep.PrintMachTable(machRows, flow.Mach);
            string machCsv = stem + "_mach_sweep.csv";
            OffDesignSweep.WriteCsv(machCsv, "mach", machRows, altKm, double.NaN);
            Console.WriteLine($"  Wrote {machCsv}");

            // AoA sweep: default -4..+10 deg.
            (double aMin, double aMax, int aN) = ParseRange(opt.SweepAoa, -4.0, 10.0, 15);
            var aoaRows = OffDesignSweep.AoASweep(surf, flow, aMin, aMax, aN);
            OffDesignSweep.PrintAoATable(aoaRows);
            string aoaCsv = stem + "_aoa_sweep.csv";
            OffDesignSweep.WriteCsv(aoaCsv, "aoa_deg", aoaRows, altKm, flow.Mach);
            Console.WriteLine($"  Wrote {aoaCsv}");
            Console.WriteLine();
        }

        // Parse "min:max:count"; fall back to the supplied defaults for any part.
        static (double, double, int) ParseRange(string? spec, double dMin, double dMax, int dN)
        {
            if (string.IsNullOrWhiteSpace(spec)) return (dMin, dMax, dN);
            string[] p = spec.Split(':');
            double min = p.Length > 0 && double.TryParse(p[0], out double a) ? a : dMin;
            double max = p.Length > 1 && double.TryParse(p[1], out double b) ? b : dMax;
            int    n   = p.Length > 2 && int.TryParse(p[2], out int c) ? Math.Max(2, c) : dN;
            return (min, max, n);
        }

        static void Generate(Library lib,
                             WaveriderSurfaces surf,
                             List<Fin>? fins,
                             double leRadiusMM,
                             string stem,
                             bool view)
        {
            Console.WriteLine($"  Voxelizing at {lib.fVoxelSize:F1} mm voxels...");
            Voxels vox = WaveriderBuilder.BuildVoxels(lib, surf, leRadiusMM, fins);

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
        public double LengthM     = double.NaN;
        public double SpanM       = double.NaN;   // fix the span; NaN => optimizer chooses
        public double BoxL        = double.NaN;   // fit-in-box envelope (length x width x height)
        public double BoxW        = double.NaN;
        public double BoxH        = double.NaN;
        public bool   HasBox => !double.IsNaN(BoxL) && !double.IsNaN(BoxW) && !double.IsNaN(BoxH);
        public double LDFloor     = double.NaN;
        public double LDRetention = 0.90;
        public double QAllowMW    = 5.0;
        public double VoxelMM     = double.NaN;
        public bool   FilletLE    = false;        // sharp by default
        public double FilletMM    = double.NaN;   // NaN => recommended radius
        public bool   View        = true;    // open the viewer by default; --no-view to disable
        public bool   Sweep       = false;

        // Fins (diamond airfoil)
        public bool   FinPair     = false;
        public bool   FinCenter   = false;
        public double FinChordPct = 25, FinTaper = 0.45, FinHeightPct = 12,
                      FinSweep = 55, FinCant = 15, FinPosPct = 60,
                      FinInsetPct = 0, FinThickPct = 6, FinRadMM = 0;

        public FinSet? oFinSet()
            => (FinPair || FinCenter) ? new FinSet
               {
                   Pair = FinPair, Center = FinCenter,
                   RootChordFrac = FinChordPct / 100.0, TaperRatio = FinTaper,
                   HeightFrac = FinHeightPct / 100.0, SweepDeg = FinSweep,
                   CantDeg = FinCant, SpanPosFrac = FinPosPct / 100.0,
                   TEInsetFrac = FinInsetPct / 100.0, ThicknessRatio = FinThickPct / 100.0,
                   LERadiusMM = FinRadMM,
               } : null;
        public string? SweepMach  = null;   // "min:max:count"
        public string? SweepAoa   = null;   // "min:max:count"
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
                        case "span":         o.SpanM = D(val); break;
                        case "box":          ParseBox(o, val); break;
                        case "ld-floor":     o.LDFloor = D(val); break;
                        case "ld-retention": o.LDRetention = D(val); break;
                        case "q-allow-mw":   o.QAllowMW = D(val); break;
                        case "voxel-mm":     o.VoxelMM = D(val); break;
                        case "sharp":        o.FilletLE = false; break;
                        case "fillet":       o.FilletLE = true; break;
                        case "fillet-mm":    o.FilletLE = true; o.FilletMM = D(val); break;
                        case "fins":         o.FinPair = true; break;
                        case "center-fin":   o.FinCenter = true; break;
                        case "fin-chord":    o.FinChordPct = D(val); break;
                        case "fin-taper":    o.FinTaper = D(val); break;
                        case "fin-height":   o.FinHeightPct = D(val); break;
                        case "fin-sweep":    o.FinSweep = D(val); break;
                        case "fin-cant":     o.FinCant = D(val); break;
                        case "fin-pos":      o.FinPosPct = D(val); break;
                        case "fin-te-inset": o.FinInsetPct = D(val); break;
                        case "fin-thick":    o.FinThickPct = D(val); break;
                        case "fin-radius-mm": o.FinRadMM = D(val); break;
                        case "view":         o.View = true; break;
                        case "no-view":      o.View = false; break;
                        case "sweep":        o.Sweep = true; break;
                        case "sweep-mach":   o.SweepMach = val; o.Sweep = true; break;
                        case "sweep-aoa":    o.SweepAoa = val; o.Sweep = true; break;
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
            if (double.IsNaN(o.LengthM) && positional.Count >= 3) o.LengthM = positional[2];
            return o;
        }

        static double D(string s) => double.TryParse(s, out double v) ? v : double.NaN;

        // Parse a "LxWxH" envelope in metres (also accepts 'X' or '*').
        static void ParseBox(CliOptions o, string val)
        {
            string[] p = val.Split('x', 'X', '*');
            if (p.Length >= 1) o.BoxL = D(p[0]);
            if (p.Length >= 2) o.BoxW = D(p[1]);
            if (p.Length >= 3) o.BoxH = D(p[2]);
        }
    }
}
