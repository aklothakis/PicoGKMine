//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - constrained volume-maximizing optimizer.
//
// Goal: maximize volumetric efficiency tau = V^(2/3)/S_planform (the standard
// "how much useful volume for a given footprint" metric) while keeping the
// inviscid+viscous lift-to-drag ratio above a floor, so the vehicle preserves
// good flight characteristics. The free variables are the conical shock angle
// (beta), the span, the shock-curve depth and the captured-streamtube fraction.
//
// The search is a coarse grid followed by a local refinement. Taylor-Maccoll
// fields are cached per shock angle because they do not depend on the planform
// variables.
//

namespace WaveriderForge
{
    public sealed class OptimizationResult
    {
        public required WaveriderDesign Design   { get; init; }
        public required AeroResult      Aero     { get; init; }
        public required bool            MeetsLD  { get; init; }
        public required double          LDFloor  { get; init; }
        public int                      Evaluations { get; init; }
        public double                   MaxLDSeen   { get; init; }
    }

    public static class Optimizer
    {
        public static OptimizationResult Optimize(
            WaveriderDesign seed,
            FlightState     flow,
            double          ldFloor,
            Action<string>? log = null,
            double?         fixedWidth = null)
        {
            double g       = flow.Gamma;
            double betaMin = GasDynamics.MachAngle(flow.Mach) * 1.06;
            double betaMax = GasDynamics.ShockAngleOfMaxDeflection(flow.Mach, g) * 0.95;

            // Use a lighter mesh while searching, then refine the winner.
            var coarse = new WaveriderDesign
            {
                LengthM = seed.LengthM, WidthM = seed.WidthM,
                ShockAngleRad = seed.ShockAngleRad,
                CurveDepthRatio = seed.CurveDepthRatio,
                CompressionFraction = seed.CompressionFraction,
                TipTaper = seed.TipTaper, CurveExponent = seed.CurveExponent,
                Nspan = 61, Nchord = 23,
            };

            var fieldCache = new Dictionary<int, ConicalFlowField>();
            ConicalFlowField Field(double beta)
            {
                int key = (int)Math.Round(beta * 1e5);
                if (!fieldCache.TryGetValue(key, out var f))
                {
                    f = ConicalFlowField.Solve(flow.Mach, beta, g);
                    fieldCache[key] = f;
                }
                return f;
            }

            int evals = 0;
            WaveriderDesign? bestDesign = null;
            AeroResult       bestAero   = AeroResult.Invalid;
            double           bestScore  = double.NegativeInfinity;
            bool             bestFeasible = false;

            // Fallback tracking (best L/D) in case nothing meets the floor.
            WaveriderDesign? fbDesign = null;
            AeroResult       fbAero   = AeroResult.Invalid;
            double           fbLD     = double.NegativeInfinity;

            void Consider(double beta, double width, double depth, double comp)
            {
                var f = Field(beta);
                if (!f.Valid) return;

                var design = new WaveriderDesign
                {
                    LengthM = coarse.LengthM, WidthM = width,
                    ShockAngleRad = beta, CurveDepthRatio = depth,
                    CompressionFraction = comp, TipTaper = coarse.TipTaper,
                    CurveExponent = coarse.CurveExponent,
                    Nspan = coarse.Nspan, Nchord = coarse.Nchord,
                };
                var surf = new WaveriderSurfaces(design, f);
                if (!surf.Valid) return;
                var aero = AeroPerformance.Evaluate(surf, flow);
                evals++;
                if (!aero.Valid) return;

                if (aero.LiftToDrag > fbLD)
                {
                    fbLD = aero.LiftToDrag; fbDesign = design; fbAero = aero;
                }

                bool feasible = aero.LiftToDrag >= ldFloor;
                // Score: feasible designs ranked by tau; infeasible never beat feasible.
                double score = feasible ? aero.Tau : aero.Tau - 1e3 * (ldFloor - aero.LiftToDrag);

                if ((feasible && !bestFeasible) ||
                    (feasible == bestFeasible && score > bestScore))
                {
                    bestScore = score; bestDesign = design; bestAero = aero;
                    bestFeasible = feasible;
                }
            }

            // ---- Stage 1: coarse grid ----------------------------------------
            const int NB = 7, NW = 5, ND = 5, NC = 5;
            int nw = fixedWidth.HasValue ? 1 : NW;
            for (int ib = 0; ib < NB; ib++)
            {
                double beta = betaMin + (betaMax - betaMin) * ib / (NB - 1);
                for (int iw = 0; iw < nw; iw++)
                {
                    // Free span ranges 0.5..1.0 x length (waveriders are wide,
                    // but keep span <= length unless the user fixes it).
                    double width = fixedWidth ?? seed.LengthM * (0.5 + 0.5 * iw / (NW - 1));
                    for (int id = 0; id < ND; id++)
                    {
                        double depth = 0.05 + 0.30 * id / (ND - 1);
                        for (int ic = 0; ic < NC; ic++)
                        {
                            double comp = 0.30 + 0.55 * ic / (NC - 1);
                            Consider(beta, width, depth, comp);
                        }
                    }
                }
            }

            log?.Invoke($"Coarse search: {evals} candidates, " +
                        (bestFeasible ? "feasible optimum found" : "no design met L/D floor (relaxing)"));

            var pick = bestDesign ?? fbDesign;
            var pickAero = bestDesign != null ? bestAero : fbAero;
            if (pick == null)
                return new OptimizationResult
                {
                    Design = seed, Aero = AeroResult.Invalid,
                    MeetsLD = false, LDFloor = ldFloor, Evaluations = evals,
                    MaxLDSeen = fbLD > 0 ? fbLD : 0
                };

            // ---- Stage 2: local refinement around the winner -----------------
            double b0 = pick.ShockAngleRad, w0 = pick.WidthM,
                   d0 = pick.CurveDepthRatio, c0 = pick.CompressionFraction;
            double db = (betaMax - betaMin) / (NB - 1) * 0.6;
            double dw = seed.LengthM * 0.5 / (NW - 1) * 0.6;
            double dd = 0.30 / (ND - 1) * 0.6;
            double dc = 0.55 / (NC - 1) * 0.6;

            int iwLo = fixedWidth.HasValue ? 0 : -1;
            int iwHi = fixedWidth.HasValue ? 0 : 1;

            for (int ib = -2; ib <= 2; ib++)
            for (int iw = iwLo; iw <= iwHi; iw++)
            for (int id = -1; id <= 1; id++)
            for (int ic = -1; ic <= 1; ic++)
            {
                double beta  = Math.Clamp(b0 + ib * db * 0.5, betaMin, betaMax);
                double width = fixedWidth ?? Math.Clamp(w0 + iw * dw, 0.4 * seed.LengthM, 1.05 * seed.LengthM);
                double depth = Math.Clamp(d0 + id * dd, 0.03, 0.40);
                double comp  = Math.Clamp(c0 + ic * dc, 0.15, 0.90);
                Consider(beta, width, depth, comp);
            }

            var finalDesign = (bestDesign ?? fbDesign)!;
            bool meets = bestFeasible;

            // Regenerate the chosen design at full resolution for the final result.
            var fullDesign = new WaveriderDesign
            {
                LengthM = seed.LengthM, WidthM = finalDesign.WidthM,
                ShockAngleRad = finalDesign.ShockAngleRad,
                CurveDepthRatio = finalDesign.CurveDepthRatio,
                CompressionFraction = finalDesign.CompressionFraction,
                TipTaper = seed.TipTaper, CurveExponent = seed.CurveExponent,
                Nspan = seed.Nspan, Nchord = seed.Nchord,
            };
            var fullField = ConicalFlowField.Solve(flow.Mach, fullDesign.ShockAngleRad, g);
            var fullSurf  = new WaveriderSurfaces(fullDesign, fullField);
            var fullAero  = AeroPerformance.Evaluate(fullSurf, flow);

            return new OptimizationResult
            {
                Design = fullDesign,
                Aero = fullAero.Valid ? fullAero : pickAero,
                MeetsLD = meets && fullAero.Valid && fullAero.LiftToDrag >= ldFloor,
                LDFloor = ldFloor,
                Evaluations = evals,
                MaxLDSeen = fbLD,
            };
        }

        static WaveriderDesign MakeDesign(WaveriderDesign seed, double length, double aspect,
                                          double beta, double depth, double comp, int ns, int nc)
            => new WaveriderDesign
            {
                LengthM = length, WidthM = aspect * length,
                ShockAngleRad = beta, CurveDepthRatio = depth, CompressionFraction = comp,
                TipTaper = seed.TipTaper, CurveExponent = seed.CurveExponent,
                Nspan = ns, Nchord = nc,
            };

        /// <summary>
        /// Maximize the volume enclosed inside a length x width x height box while
        /// keeping L/D above a floor. Because L/D and shape are scale invariant,
        /// each candidate shape is scaled up until it just touches a box face;
        /// the objective is the resulting absolute volume.
        /// </summary>
        public static OptimizationResult OptimizeInBox(
            WaveriderDesign seed,
            FlightState     flow,
            double boxL, double boxW, double boxH,
            double ldFloor,
            Action<string>? log = null)
        {
            double g       = flow.Gamma;
            double betaMin = GasDynamics.MachAngle(flow.Mach) * 1.06;
            double betaMax = GasDynamics.ShockAngleOfMaxDeflection(flow.Mach, g) * 0.95;

            var fieldCache = new Dictionary<int, ConicalFlowField>();
            ConicalFlowField Field(double beta)
            {
                int key = (int)Math.Round(beta * 1e5);
                if (!fieldCache.TryGetValue(key, out var f))
                {
                    f = ConicalFlowField.Solve(flow.Mach, beta, g);
                    fieldCache[key] = f;
                }
                return f;
            }

            const int csNs = 61, csNc = 23;

            int evals = 0;
            double bestScore = double.NegativeInfinity;
            bool   bestFeasible = false, haveBest = false;
            (double beta, double a, double depth, double comp) best = default;

            double fbLD = double.NegativeInfinity;
            (double beta, double a, double depth, double comp) fb = default;
            bool haveFb = false;

            void Consider(double beta, double a, double depth, double comp)
            {
                var f = Field(beta);
                if (!f.Valid) return;

                var d1 = MakeDesign(seed, 1.0, a, beta, depth, comp, csNs, csNc);
                var s1 = new WaveriderSurfaces(d1, f);
                if (!s1.Valid) return;

                s1.Extents(out double ex, out double ey, out double ez);
                if (ex <= 1e-6 || ey <= 1e-6 || ez <= 1e-6) return;

                double lMax = Math.Min(boxL / ex, Math.Min(boxW / ey, boxH / ez));
                if (lMax <= 0 || double.IsNaN(lMax) || double.IsInfinity(lMax)) return;

                var aero = AeroPerformance.Evaluate(s1, flow);
                if (!aero.Valid) return;
                evals++;

                double vol = aero.Volume * lMax * lMax * lMax;   // scales with size^3

                if (aero.LiftToDrag > fbLD)
                {
                    fbLD = aero.LiftToDrag; fb = (beta, a, depth, comp); haveFb = true;
                }

                bool feasible = aero.LiftToDrag >= ldFloor;
                double score = feasible ? vol : vol - 1e9 * (ldFloor - aero.LiftToDrag);

                if ((feasible && !bestFeasible) ||
                    (feasible == bestFeasible && score > bestScore))
                {
                    bestScore = score; best = (beta, a, depth, comp);
                    bestFeasible = feasible; haveBest = true;
                }
            }

            // ---- Coarse grid over shape + aspect ------------------------------
            const int NB = 7, NA = 6, ND = 5, NC = 5;
            for (int ib = 0; ib < NB; ib++)
            {
                double beta = betaMin + (betaMax - betaMin) * ib / (NB - 1);
                for (int ia = 0; ia < NA; ia++)
                {
                    double a = 0.4 + 1.0 * ia / (NA - 1);        // span/length 0.4..1.4
                    for (int id = 0; id < ND; id++)
                    {
                        double depth = 0.05 + 0.30 * id / (ND - 1);
                        for (int ic = 0; ic < NC; ic++)
                            Consider(beta, a, depth, 0.30 + 0.55 * ic / (NC - 1));
                    }
                }
            }

            if (!haveBest && !haveFb)
                return new OptimizationResult
                {
                    Design = seed, Aero = AeroResult.Invalid,
                    MeetsLD = false, LDFloor = ldFloor, Evaluations = evals, MaxLDSeen = 0
                };

            var sel = haveBest ? best : fb;

            // ---- Local refinement ---------------------------------------------
            double db = (betaMax - betaMin) / (NB - 1) * 0.6;
            double da = 1.0 / (NA - 1) * 0.6;
            double dd = 0.30 / (ND - 1) * 0.6;
            double dc = 0.55 / (NC - 1) * 0.6;

            for (int ib = -2; ib <= 2; ib++)
            for (int ia = -1; ia <= 1; ia++)
            for (int id = -1; id <= 1; id++)
            for (int ic = -1; ic <= 1; ic++)
            {
                double beta  = Math.Clamp(sel.beta + ib * db * 0.5, betaMin, betaMax);
                double a     = Math.Clamp(sel.a + ia * da, 0.30, 1.60);
                double depth = Math.Clamp(sel.depth + id * dd, 0.03, 0.40);
                double comp  = Math.Clamp(sel.comp + ic * dc, 0.15, 0.90);
                Consider(beta, a, depth, comp);
            }

            sel = haveBest ? best : fb;

            // ---- Finalize: scale the winner to the box at full resolution ------
            var field = ConicalFlowField.Solve(flow.Mach, sel.beta, g);
            var refSurf = new WaveriderSurfaces(
                MakeDesign(seed, 1.0, sel.a, sel.beta, sel.depth, sel.comp, seed.Nspan, seed.Nchord),
                field);
            refSurf.Extents(out double rex, out double rey, out double rez);
            double lMaxFull = Math.Min(boxL / rex, Math.Min(boxW / rey, boxH / rez));

            var fullDesign = MakeDesign(seed, lMaxFull, sel.a, sel.beta, sel.depth, sel.comp,
                                        seed.Nspan, seed.Nchord);
            var fullSurf = new WaveriderSurfaces(fullDesign, field);
            var fullAero = AeroPerformance.Evaluate(fullSurf, flow);

            return new OptimizationResult
            {
                Design = fullDesign,
                Aero = fullAero,
                MeetsLD = bestFeasible && fullAero.Valid && fullAero.LiftToDrag >= ldFloor,
                LDFloor = ldFloor,
                Evaluations = evals,
                MaxLDSeen = fbLD,
            };
        }
    }
}
