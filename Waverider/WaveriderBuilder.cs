//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - bridge from the analytic surfaces to PicoGK geometry.
//
// Converts the metre-scale surface grids into a PicoGK millimetre mesh/voxel
// field, optionally blunting the leading edge with a swept cylinder whose radius
// is sized from the stagnation heat flux at the flight condition.
//

using System.Numerics;
using PicoGK;

namespace WaveriderForge
{
    public static class Heating
    {
        // Sutton-Graves stagnation-point convective heating constant (Earth air),
        // q[W/cm^2] = k * sqrt(rho_inf / R_n) * V_inf^3.
        const double K = 1.7415e-4;

        /// <summary>Stagnation heat flux (W/m^2) for a nose radius R_n (m).</summary>
        public static double StagHeatFlux(FlightState f, double noseRadiusM)
        {
            double qWcm2 = K * Math.Sqrt(f.Density / noseRadiusM) *
                           Math.Pow(f.Velocity, 3.0);
            return qWcm2 * 1.0e4;   // W/cm^2 -> W/m^2
        }

        /// <summary>
        /// Leading-edge radius (m) that keeps the stagnation heat flux at or below
        /// the allowable value at this flight condition.
        /// </summary>
        public static double LeadingEdgeRadius(FlightState f, double qAllowWm2)
        {
            double qWcm2 = qAllowWm2 * 1.0e-4;
            double ratio = K * Math.Pow(f.Velocity, 3.0) / qWcm2;
            return f.Density * ratio * ratio;
        }
    }

    public static class WaveriderBuilder
    {
        public const double MM = 1000.0;   // metres -> millimetres

        static Vector3 ToMM(D3 p) =>
            new Vector3((float)(p.X * MM), (float)(p.Y * MM), (float)(p.Z * MM));

        /// <summary>Build a sharp triangle mesh of the body (millimetres).</summary>
        public static Mesh BuildMesh(Library lib, WaveriderSurfaces s)
        {
            Mesh msh = new(lib);
            s.ForEachTriangle((a, b, c) =>
                msh.nAddTriangle(ToMM(a), ToMM(b), ToMM(c)));
            return msh;
        }

        /// <summary>
        /// Voxelize the body. If <paramref name="leRadiusMM"/> &gt; 0 the leading
        /// edge is blunted by unioning a swept cylinder of that radius.
        /// </summary>
        public static Voxels BuildVoxels(Library lib, WaveriderSurfaces s, double leRadiusMM)
        {
            using Mesh msh = BuildMesh(lib, s);
            Voxels vox = new(msh);

            if (leRadiusMM > 0)
            {
                float r = (float)Math.Max(leRadiusMM, 1.5 * lib.fVoxelSize);
                Lattice lat = new(lib);
                D3[] le = s.LeadingEdge();
                for (int i = 0; i < le.Length - 1; i++)
                    lat.AddBeam(ToMM(le[i]), r, ToMM(le[i + 1]), r, true);
                vox.BoolAdd(new Voxels(lat));
            }
            return vox;
        }
    }
}
