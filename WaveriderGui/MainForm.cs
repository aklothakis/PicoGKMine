//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - Windows Forms control panel.
//
// Enter the flight condition, size constraints, leading-edge treatment and fin
// arrangement; click Generate. The optimized waverider (with fins) is rendered
// live in the preview pane (drag to orbit) with the performance report below,
// and the STL/VDB are exported.
//

using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Windows.Forms;

namespace WaveriderForge.Gui
{
    public sealed class MainForm : Form
    {
        // Flight + size
        readonly NumericUpDown numMach, numAlt, numLen, numBoxL, numBoxW, numBoxH;
        readonly CheckBox      chkBox;
        // Leading edge
        readonly CheckBox      chkFillet;
        readonly NumericUpDown numFillet, numQ;
        readonly Label         lblRec;
        readonly Button        btnRec;
        // Fins
        readonly CheckBox      chkFinPair, chkFinCenter;
        readonly NumericUpDown numFinChord, numFinTaper, numFinHeight, numFinSweep,
                               numFinCant, numFinPos, numFinInset, numFinThick, numFinRad;
        // Solver + mesh
        readonly NumericUpDown numRet, numVoxel;
        readonly CheckBox      chkSweep, chkAutoVox;

        readonly Button        btnGen, btnStl, btnFolder;
        readonly TextBox       txtReport;
        readonly Viewport3D    viewport;
        readonly SplitContainer split;
        readonly Label         lblStatus;

        string? m_strLastStl;
        readonly string m_strOutDir;

        public MainForm()
        {
            Text          = "Waverider Forge";
            Width         = 1080;
            Height        = 760;
            MinimumSize   = new Size(940, 620);
            StartPosition = FormStartPosition.CenterScreen;
            Font          = new Font("Segoe UI", 9F);

            m_strOutDir = Path.Combine(AppContext.BaseDirectory, "output");

            // ---- Input panel (left) ------------------------------------------
            var pnl = new Panel { Dock = DockStyle.Left, Width = 372, Padding = new Padding(12), AutoScroll = true };

            int y = 8;
            AddHeader(pnl, "Flight condition", ref y);
            numMach = AddNumeric(pnl, "Design Mach", 1.5m, 40m, 1, 0.5m, 8m, ref y);
            numAlt  = AddNumeric(pnl, "Altitude (km)", 0m, 86m, 1, 1m, 30m, ref y);

            y += 4;
            AddHeader(pnl, "Size", ref y);
            chkBox = AddCheck(pnl, "Fit inside a box (L x W x H)", false, ref y);
            numLen  = AddNumeric(pnl, "Length (m)", 0.1m, 200m, 2, 1m, 20m, ref y);
            numBoxL = AddNumeric(pnl, "Box length (m)", 0.1m, 100m, 2, 0.5m, 6m, ref y);
            numBoxW = AddNumeric(pnl, "Box width (m)", 0.1m, 100m, 2, 0.5m, 5m, ref y);
            numBoxH = AddNumeric(pnl, "Box height (m)", 0.1m, 100m, 2, 0.25m, 1.5m, ref y);

            y += 4;
            AddHeader(pnl, "Leading edge", ref y);
            chkFillet = AddCheck(pnl, "Fillet leading edge (default: sharp)", false, ref y);
            numFillet = AddNumeric(pnl, "Fillet radius (mm)", 0.1m, 1000m, 1, 0.5m, 5m, ref y);
            numQ      = AddNumeric(pnl, "Allowable q (MW/m^2)", 0.5m, 50m, 1, 0.5m, 5m, ref y);
            lblRec = new Label { Left = 12, Top = y + 3, Width = 216, Height = 20, ForeColor = Color.FromArgb(30, 90, 30) };
            pnl.Controls.Add(lblRec);
            btnRec = new Button { Text = "Use recommended", Left = 228, Top = y, Width = 116, Height = 24 };
            btnRec.Click += (_, _) => UseRecommended();
            pnl.Controls.Add(btnRec);
            y += 30;

            y += 4;
            AddHeader(pnl, "Fins (diamond airfoil)", ref y);
            chkFinPair   = AddCheck(pnl, "Mirrored fin pair (outboard)", false, ref y);
            chkFinCenter = AddCheck(pnl, "Center fin", false, ref y);
            numFinChord  = AddNumeric(pnl, "Root chord (% length)", 5m, 60m, 1, 1m, 25m, ref y);
            numFinTaper  = AddNumeric(pnl, "Taper ratio (tip/root)", 0.05m, 1m, 2, 0.05m, 0.45m, ref y);
            numFinHeight = AddNumeric(pnl, "Height (% length)", 2m, 60m, 1, 1m, 12m, ref y);
            numFinSweep  = AddNumeric(pnl, "LE sweep (deg)", 0m, 80m, 1, 1m, 55m, ref y);
            numFinCant   = AddNumeric(pnl, "Cant from vertical (deg)", 0m, 60m, 1, 1m, 15m, ref y);
            numFinPos    = AddNumeric(pnl, "Span position (% half-span)", 0m, 95m, 1, 5m, 60m, ref y);
            numFinInset  = AddNumeric(pnl, "TE inset from base (% length)", 0m, 50m, 1, 1m, 0m, ref y);
            numFinThick  = AddNumeric(pnl, "Thickness (% chord)", 2m, 20m, 1, 0.5m, 6m, ref y);
            numFinRad    = AddNumeric(pnl, "Fin LE radius (mm, 0 = sharp)", 0m, 500m, 1, 0.5m, 0m, ref y);

            y += 4;
            AddHeader(pnl, "Solver & mesh", ref y);
            numRet     = AddNumeric(pnl, "L/D retention (0-1)", 0.50m, 1.00m, 2, 0.05m, 0.90m, ref y);
            chkSweep   = AddCheck(pnl, "Run off-design sweep (CSV)", false, ref y);
            chkAutoVox = AddCheck(pnl, "Auto voxel size", true, ref y);
            numVoxel   = AddNumeric(pnl, "Voxel size (mm)", 0.2m, 200m, 1, 0.5m, 8m, ref y);

            y += 8;
            btnGen = new Button { Text = "Generate", Left = 12, Top = y, Width = 332, Height = 36 };
            btnGen.Click += OnGenerate;
            pnl.Controls.Add(btnGen);
            y += 46;

            lblStatus = new Label { Left = 12, Top = y, Width = 332, Height = 40, Text = "Ready." };
            pnl.Controls.Add(lblStatus);

            // ---- Right side: 3D preview (top) + report (bottom) --------------
            viewport = new Viewport3D { Dock = DockStyle.Fill };
            txtReport = new TextBox
            {
                Dock = DockStyle.Fill,
                Multiline = true,
                ReadOnly = true,
                ScrollBars = ScrollBars.Both,
                WordWrap = false,
                Font = new Font("Consolas", 9.5F),
                BackColor = Color.White,
            };

            split = new SplitContainer
            {
                Dock = DockStyle.Fill,
                Orientation = Orientation.Horizontal,
                SplitterWidth = 6,
                Panel1MinSize = 140,
                Panel2MinSize = 90,
            };
            split.Panel1.Controls.Add(viewport);
            split.Panel2.Controls.Add(txtReport);

            // ---- Bottom button bar -------------------------------------------
            var bar = new Panel { Dock = DockStyle.Bottom, Height = 44, Padding = new Padding(8) };
            btnStl = new Button { Text = "Open 3D model (STL)", Left = 8, Top = 6, Width = 180, Height = 30, Enabled = false };
            btnStl.Click += (_, _) => OpenPath(m_strLastStl);
            bar.Controls.Add(btnStl);
            btnFolder = new Button { Text = "Open output folder", Left = 196, Top = 6, Width = 180, Height = 30 };
            btnFolder.Click += (_, _) => OpenPath(m_strOutDir);
            bar.Controls.Add(btnFolder);

            // Add Fill control first so the edge panels claim their space first.
            Controls.Add(split);
            Controls.Add(pnl);
            Controls.Add(bar);

            chkBox.CheckedChanged       += (_, _) => SyncEnables();
            chkFillet.CheckedChanged    += (_, _) => SyncEnables();
            chkFinPair.CheckedChanged   += (_, _) => SyncEnables();
            chkFinCenter.CheckedChanged += (_, _) => SyncEnables();
            chkAutoVox.CheckedChanged   += (_, _) => SyncEnables();
            numMach.ValueChanged += (_, _) => UpdateRecommendation();
            numAlt.ValueChanged  += (_, _) => UpdateRecommendation();
            numQ.ValueChanged    += (_, _) => UpdateRecommendation();

            SyncEnables();
            UpdateRecommendation();
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            try { split.SplitterDistance = Math.Max(160, (int)(split.Height * 0.6)); }
            catch { /* size not ready yet; default is fine */ }
        }

        void SyncEnables()
        {
            bool box = chkBox.Checked;
            numLen.Enabled  = !box;
            numBoxL.Enabled = box;
            numBoxW.Enabled = box;
            numBoxH.Enabled = box;

            numFillet.Enabled = chkFillet.Checked;
            btnRec.Enabled    = chkFillet.Checked;

            bool fins = chkFinPair.Checked || chkFinCenter.Checked;
            foreach (var n in new[] { numFinChord, numFinTaper, numFinHeight, numFinSweep,
                                      numFinCant, numFinPos, numFinInset, numFinThick, numFinRad })
                n.Enabled = fins;

            numVoxel.Enabled = !chkAutoVox.Checked;
        }

        double RecommendedMM()
        {
            var flow = Atmosphere.At((double)numAlt.Value * 1000.0, (double)numMach.Value);
            return Heating.LeadingEdgeRadius(flow, (double)numQ.Value * 1.0e6) * 1000.0;
        }

        void UpdateRecommendation()
        {
            try { lblRec.Text = $"Recommended: {RecommendedMM():F1} mm"; }
            catch { lblRec.Text = "Recommended: -"; }
        }

        void UseRecommended()
        {
            try
            {
                decimal rec = (decimal)Math.Clamp(RecommendedMM(),
                    (double)numFillet.Minimum, (double)numFillet.Maximum);
                numFillet.Value = Math.Round(rec, 1);
            }
            catch { /* keep current value */ }
        }

        FinSet? BuildFinSet()
        {
            if (!chkFinPair.Checked && !chkFinCenter.Checked) return null;
            return new FinSet
            {
                Pair           = chkFinPair.Checked,
                Center         = chkFinCenter.Checked,
                RootChordFrac  = (double)numFinChord.Value / 100.0,
                TaperRatio     = (double)numFinTaper.Value,
                HeightFrac     = (double)numFinHeight.Value / 100.0,
                SweepDeg       = (double)numFinSweep.Value,
                CantDeg        = (double)numFinCant.Value,
                SpanPosFrac    = (double)numFinPos.Value / 100.0,
                TEInsetFrac    = (double)numFinInset.Value / 100.0,
                ThicknessRatio = (double)numFinThick.Value / 100.0,
                LERadiusMM     = (double)numFinRad.Value,
            };
        }

        async void OnGenerate(object? sender, EventArgs e)
        {
            var inp = new WaveriderInputs
            {
                Mach        = (double)numMach.Value,
                AltitudeKm  = (double)numAlt.Value,
                LengthM     = (double)numLen.Value,
                UseBox      = chkBox.Checked,
                BoxL        = (double)numBoxL.Value,
                BoxW        = (double)numBoxW.Value,
                BoxH        = (double)numBoxH.Value,
                FilletLE    = chkFillet.Checked,
                FilletMM    = chkFillet.Checked ? (double)numFillet.Value : double.NaN,
                QAllowMW    = (double)numQ.Value,
                Fins        = BuildFinSet(),
                LDRetention = (double)numRet.Value,
                VoxelMM     = chkAutoVox.Checked ? double.NaN : (double)numVoxel.Value,
                Sweep       = chkSweep.Checked,
                OutDir      = m_strOutDir,
            };

            btnGen.Enabled = false;
            btnStl.Enabled = false;
            lblStatus.Text = "Designing and voxelizing...";
            Cursor = Cursors.WaitCursor;

            try
            {
                var job = await Task.Run(() =>
                {
                    var r = WaveriderJob.Design(inp);
                    string report = r.Report;
                    string? stl = null;

                    if (r.Opt.Aero.Valid && r.Surfaces is not null)
                    {
                        string stem = WaveriderJob.Stem(inp);
                        double vol = WaveriderJob.Export(r, stem, inp.VoxelMM, out string stlp, out _);
                        stl = stlp;
                        report += Environment.NewLine +
                                  $"Voxel-model volume   : {vol:F3} m^3" + Environment.NewLine +
                                  $"Wrote {stlp}";
                        if (inp.Sweep) report += Environment.NewLine + RunSweeps(r, inp, stem);
                    }
                    var tris = BuildPreview(r);
                    return (report, stl, tris);
                });

                txtReport.Text = job.report.Replace("\n", Environment.NewLine);
                m_strLastStl   = job.stl;
                btnStl.Enabled = job.stl is not null;

                if (job.tris.Count > 0) viewport.SetTriangles(job.tris);
                else                    viewport.Clear();

                lblStatus.Text = job.stl is not null ? "Done." : "Done - no valid geometry.";
            }
            catch (Exception ex)
            {
                txtReport.Text = "Error during generation:" + Environment.NewLine + Environment.NewLine + ex;
                lblStatus.Text = "Error.";
            }
            finally
            {
                btnGen.Enabled = true;
                Cursor = Cursors.Default;
            }
        }

        // Build a reduced-resolution triangle list (body + fins) for the preview.
        static List<(Vector3, Vector3, Vector3)> BuildPreview(WaveriderJobResult r)
        {
            var list = new List<(Vector3, Vector3, Vector3)>();
            var src = r.Surfaces;
            if (!r.Opt.Aero.Valid || src is null) return list;

            var d = r.Opt.Design;
            var preview = new WaveriderDesign
            {
                LengthM = d.LengthM, WidthM = d.WidthM,
                ShockAngleRad = d.ShockAngleRad, CurveDepthRatio = d.CurveDepthRatio,
                CompressionFraction = d.CompressionFraction, TipTaper = d.TipTaper,
                CurveExponent = d.CurveExponent, Nspan = 61, Nchord = 19,
            };
            var ps = new WaveriderSurfaces(preview, src.Field);
            if (!ps.Valid) return list;

            static Vector3 V(D3 p) => new((float)p.X, (float)p.Y, (float)p.Z);

            ps.ForEachTriangle((a, b, c) => list.Add((V(a), V(b), V(c))));

            if (r.Fins != null)
                foreach (var fin in r.Fins)
                    foreach (var (a, b, c) in fin.Tris)
                        list.Add((V(a), V(b), V(c)));

            return list;
        }

        static string RunSweeps(WaveriderJobResult r, WaveriderInputs inp, string stem)
        {
            var surf = r.Surfaces;
            if (surf is null) return "";
            double mDes = r.Flow.Mach, g = r.Flow.Gamma;
            double altM = inp.AltitudeKm * 1000.0;

            var mach = OffDesignSweep.MachSweep(surf, altM,
                Math.Max(1.5, 0.35 * mDes), 1.4 * mDes, 14, g);
            OffDesignSweep.WriteCsv(stem + "_mach_sweep.csv", "mach", mach, inp.AltitudeKm, double.NaN);

            var aoa = OffDesignSweep.AoASweep(surf, r.Flow, -4.0, 10.0, 15);
            OffDesignSweep.WriteCsv(stem + "_aoa_sweep.csv", "aoa_deg", aoa, inp.AltitudeKm, mDes);

            return $"Wrote {stem}_mach_sweep.csv and _aoa_sweep.csv (body only)";
        }

        static void OpenPath(string? path)
        {
            if (string.IsNullOrEmpty(path)) return;
            try { Process.Start(new ProcessStartInfo(path) { UseShellExecute = true }); }
            catch (Exception ex) { MessageBox.Show("Could not open:\n" + ex.Message); }
        }

        // ---- Small layout helpers --------------------------------------------

        static void AddHeader(Panel pnl, string text, ref int y)
        {
            pnl.Controls.Add(new Label
            {
                Text = text, Left = 8, Top = y, Width = 340, Height = 18,
                Font = new Font("Segoe UI", 9F, FontStyle.Bold), ForeColor = Color.FromArgb(60, 60, 90),
            });
            y += 24;
        }

        static NumericUpDown AddNumeric(Panel pnl, string label, decimal min, decimal max,
                                        int dp, decimal inc, decimal val, ref int y)
        {
            pnl.Controls.Add(new Label { Text = label, Left = 12, Top = y + 4, Width = 196, Height = 20 });
            var n = new NumericUpDown
            {
                Left = 214, Top = y, Width = 130,
                DecimalPlaces = dp, Minimum = min, Maximum = max, Increment = inc, Value = val,
            };
            pnl.Controls.Add(n);
            y += 30;
            return n;
        }

        static CheckBox AddCheck(Panel pnl, string label, bool chk, ref int y)
        {
            var c = new CheckBox { Text = label, Left = 12, Top = y, Width = 332, Height = 22, Checked = chk };
            pnl.Controls.Add(c);
            y += 28;
            return c;
        }
    }
}
