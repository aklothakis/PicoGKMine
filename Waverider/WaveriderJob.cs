//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - high-level job facade.
//
// Wraps the full design pipeline (atmosphere -> optimize -> report -> export)
// behind a small parameter object so both the console front end and the GUI
// drive identical logic.
//

using System.Text;
using PicoGK;

namespace WaveriderForge
{
    /// <summary>User-facing design inputs (SI-ish: km for altitude, metres else).</summary>
    public sealed class WaveriderInputs
    {
        public double Mach        = 8.0;
        public double AltitudeKm  = 30.0;
        public double LengthM     = 20.0;
        public double SpanM       = double.NaN;   // fix span (non-box); NaN => optimizer
        public bool   UseBox      = false;
        public double BoxL        = 6.0;
        public double BoxW        = 5.0;
        public double BoxH        = 1.5;
        public bool   Blunt       = true;
        public double QAllowMW    = 5.0;
        public double LDRetention = 0.90;
        public double LDFloor     = double.NaN;
        public double VoxelMM     = double.NaN;   // voxel size; NaN => auto from length
        public bool   Sweep       = false;
        public string OutDir      = "output";
    }

    public sealed class WaveriderJobResult
    {
        public required OptimizationResult Opt       { get; init; }
        public required FlightState        Flow      { get; init; }
        public WaveriderSurfaces?          Surfaces  { get; init; }
        public double                      LeRadiusM { get; init; }
        public string                      Report    { get; set; } = "";
    }

    public static class WaveriderJob
    {
        /// <summary>Run the atmosphere model and optimizer; build the text report.</summary>
        public static WaveriderJobResult Design(WaveriderInputs inp, Action<string>? log = null)
        {
            FlightState flow = Atmosphere.At(inp.AltitudeKm * 1000.0, inp.Mach);
            double? fixedSpan = double.IsNaN(inp.SpanM) ? null : inp.SpanM;

            var seed = new WaveriderDesign
            {
                LengthM = inp.UseBox ? 1.0 : inp.LengthM,
                WidthM  = inp.UseBox ? 1.0 : (fixedSpan ?? inp.LengthM),
            };

            OptimizationResult result;
            if (inp.UseBox)
            {
                var probe = Optimizer.OptimizeInBox(seed, flow, inp.BoxL, inp.BoxW, inp.BoxH, 0.0);
                double floor = !double.IsNaN(inp.LDFloor) ? inp.LDFloor : inp.LDRetention * probe.MaxLDSeen;
                result = Optimizer.OptimizeInBox(seed, flow, inp.BoxL, inp.BoxW, inp.BoxH, floor, log);
            }
            else
            {
                var probe = Optimizer.Optimize(seed, flow, 0.0, null, fixedSpan);
                double floor = !double.IsNaN(inp.LDFloor) ? inp.LDFloor : inp.LDRetention * probe.MaxLDSeen;
                result = Optimizer.Optimize(seed, flow, floor, log, fixedSpan);
            }

            WaveriderSurfaces? surf = null;
            if (result.Aero.Valid)
                surf = new WaveriderSurfaces(result.Design,
                    ConicalFlowField.Solve(flow.Mach, result.Design.ShockAngleRad, flow.Gamma));

            double leR = inp.Blunt ? Heating.LeadingEdgeRadius(flow, inp.QAllowMW * 1.0e6) : 0.0;

            var jr = new WaveriderJobResult
            {
                Opt = result, Flow = flow, Surfaces = surf, LeRadiusM = leR,
            };
            jr.Report = BuildReport(jr, inp);
            return jr;
        }

        public static string Stem(WaveriderInputs inp)
            => Path.Combine(inp.OutDir, $"waverider_M{inp.Mach:0.0}_{inp.AltitudeKm:0}km");

        /// <summary>Default (auto) voxel size in mm for a given design length (m).</summary>
        public static double AutoVoxelMM(double lengthM) => Math.Max(lengthM * 1000.0 / 600.0, 0.5);

        /// <summary>Voxelize and write STL + VDB. Returns the voxel-model volume (m^3).</summary>
        public static double Export(WaveriderJobResult r, string stem, double voxelMM,
                                    out string stlPath, out string vdbPath)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(Path.GetFullPath(stem))!);

            if (double.IsNaN(voxelMM) || voxelMM <= 0)
                voxelMM = AutoVoxelMM(r.Opt.Design.LengthM);

            using Library lib = new((float)voxelMM);
            var surf = new WaveriderSurfaces(r.Opt.Design,
                ConicalFlowField.Solve(r.Flow.Mach, r.Opt.Design.ShockAngleRad, r.Flow.Gamma));
            Voxels vox = WaveriderBuilder.BuildVoxels(lib, surf, r.LeRadiusM * 1000.0);

            vox.CalculateProperties(out float volMM3, out _);

            stlPath = stem + ".stl";
            vdbPath = stem + ".vdb";
            using (Mesh msh = new(vox))
                msh.SaveToStlFile(stlPath, Mesh.EStlUnit.MM);
            vox.SaveToVdbFile(vdbPath);

            return volMM3 / 1.0e9;
        }

        public static string BuildReport(WaveriderJobResult r, WaveriderInputs inp)
        {
            var f = r.Flow;
            var sb = new StringBuilder();
            sb.AppendLine("Flight condition (1976 U.S. Standard Atmosphere)");
            sb.AppendLine($"  Mach {f.Mach:F2}  |  altitude {f.AltitudeM / 1000.0:F1} km");
            sb.AppendLine($"  Velocity         : {f.Velocity:F0} m/s");
            sb.AppendLine($"  Static temp      : {f.Temperature:F1} K");
            sb.AppendLine($"  Static pressure  : {f.Pressure / 1000.0:F2} kPa");
            sb.AppendLine($"  Dynamic pressure : {f.DynamicPressure / 1000.0:F2} kPa");
            sb.AppendLine($"  Unit Reynolds    : {f.UnitReynolds:E2} 1/m");
            sb.AppendLine();

            if (!r.Opt.Aero.Valid || r.Surfaces is null)
            {
                sb.AppendLine("No valid waverider could be generated for these inputs.");
                sb.AppendLine("Try a different Mach / altitude / envelope.");
                return sb.ToString();
            }

            var d = r.Opt.Design;
            var a = r.Opt.Aero;
            r.Surfaces.Extents(out double ex, out double ey, out double ez);

            sb.AppendLine("Optimized design");
            sb.AppendLine($"  Shock angle (beta)   : {d.ShockAngleRad * 180.0 / Math.PI:F2} deg");
            sb.AppendLine($"  Cone angle (theta_c) : {a.ConeAngleDeg:F2} deg");
            sb.AppendLine($"  Length x span        : {d.LengthM:F2} x {d.WidthM:F2} m");
            if (inp.UseBox)
                sb.AppendLine($"  Box envelope target  : {inp.BoxL:F2} x {inp.BoxW:F2} x {inp.BoxH:F2} m");
            sb.AppendLine($"  Bounding box (LxWxH) : {ex:F2} x {ey:F2} x {ez:F2} m");
            sb.AppendLine($"  Compression fraction : {d.CompressionFraction:F3}");
            sb.AppendLine();
            sb.AppendLine("Aerodynamics (per full vehicle)");
            sb.AppendLine($"  Lift / Drag (L/D)    : {a.LiftToDrag:F2}" + (r.Opt.MeetsLD ? "" : "  (below floor)"));
            sb.AppendLine($"  L/D floor used       : {r.Opt.LDFloor:F2}  (max seen {r.Opt.MaxLDSeen:F2})");
            sb.AppendLine($"  Lift                 : {a.Lift / 1000.0:F1} kN");
            sb.AppendLine($"  Drag                 : {a.Drag / 1000.0:F1} kN");
            sb.AppendLine($"    pressure / friction / base : {a.DragPressure / 1000.0:F1} / {a.DragFriction / 1000.0:F1} / {a.DragBase / 1000.0:F1} kN");
            sb.AppendLine($"  C_L / C_D            : {a.CL:F4} / {a.CD:F4}");
            sb.AppendLine();
            sb.AppendLine("Geometry");
            sb.AppendLine($"  Volume               : {a.Volume:F3} m^3");
            sb.AppendLine($"  Planform area        : {a.PlanformArea:F2} m^2");
            sb.AppendLine($"  Wetted area          : {a.WettedArea:F2} m^2");
            sb.AppendLine($"  Volumetric eff. tau  : {a.Tau:F4}");
            if (r.LeRadiusM > 0)
            {
                double q = Heating.StagHeatFlux(f, r.LeRadiusM);
                sb.AppendLine($"  Leading-edge radius  : {r.LeRadiusM * 1000.0:F1} mm  (q_stag {q / 1.0e6:F2} MW/m^2)");
            }
            else
            {
                sb.AppendLine("  Leading edge         : sharp");
            }
            return sb.ToString();
        }
    }
}
