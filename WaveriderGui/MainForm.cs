//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - Windows Forms control panel.
//
// A simple GUI front end over the WaveriderJob facade: enter the flight
// condition and size constraints, click Generate, read the performance report,
// and open the exported STL in the system 3D viewer.
//

using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;

namespace WaveriderForge.Gui
{
    public sealed class MainForm : Form
    {
        readonly NumericUpDown numMach, numAlt, numLen, numBoxL, numBoxW, numBoxH, numQ, numRet;
        readonly CheckBox      chkBox, chkBlunt, chkSweep;
        readonly Button        btnGen, btnStl, btnFolder;
        readonly TextBox       txtReport;
        readonly Label         lblStatus;

        string? m_strLastStl;
        readonly string m_strOutDir;

        public MainForm()
        {
            Text          = "Waverider Forge";
            Width         = 960;
            Height        = 660;
            MinimumSize   = new Size(820, 560);
            StartPosition = FormStartPosition.CenterScreen;
            Font          = new Font("Segoe UI", 9F);

            m_strOutDir = Path.Combine(AppContext.BaseDirectory, "output");

            // ---- Input panel (left) ------------------------------------------
            var pnl = new Panel { Dock = DockStyle.Left, Width = 360, Padding = new Padding(12), AutoScroll = true };

            int y = 8;
            AddHeader(pnl, "Flight condition", ref y);
            numMach = AddNumeric(pnl, "Design Mach", 1.5m, 40m, 1, 0.5m, 8m, ref y);
            numAlt  = AddNumeric(pnl, "Altitude (km)", 0m, 86m, 1, 1m, 30m, ref y);

            y += 6;
            AddHeader(pnl, "Size", ref y);
            chkBox = AddCheck(pnl, "Fit inside a box (L x W x H)", false, ref y);
            numLen  = AddNumeric(pnl, "Length (m)", 0.1m, 200m, 2, 1m, 20m, ref y);
            numBoxL = AddNumeric(pnl, "Box length (m)", 0.1m, 100m, 2, 0.5m, 6m, ref y);
            numBoxW = AddNumeric(pnl, "Box width (m)", 0.1m, 100m, 2, 0.5m, 5m, ref y);
            numBoxH = AddNumeric(pnl, "Box height (m)", 0.1m, 100m, 2, 0.25m, 1.5m, ref y);

            y += 6;
            AddHeader(pnl, "Design options", ref y);
            chkBlunt = AddCheck(pnl, "Blunt leading edge (from heating)", true, ref y);
            numQ     = AddNumeric(pnl, "Allowable q (MW/m^2)", 0.5m, 50m, 1, 0.5m, 5m, ref y);
            numRet   = AddNumeric(pnl, "L/D retention (0-1)", 0.50m, 1.00m, 2, 0.05m, 0.90m, ref y);
            chkSweep = AddCheck(pnl, "Run off-design sweep (CSV)", false, ref y);

            y += 10;
            btnGen = new Button { Text = "Generate", Left = 12, Top = y, Width = 320, Height = 36 };
            btnGen.Click += OnGenerate;
            pnl.Controls.Add(btnGen);
            y += 46;

            lblStatus = new Label { Left = 12, Top = y, Width = 320, Height = 40, Text = "Ready." };
            pnl.Controls.Add(lblStatus);

            // ---- Report (right) ----------------------------------------------
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

            // ---- Bottom button bar -------------------------------------------
            var bar = new Panel { Dock = DockStyle.Bottom, Height = 44, Padding = new Padding(8) };

            btnStl = new Button { Text = "Open 3D model (STL)", Left = 8, Top = 6, Width = 180, Height = 30, Enabled = false };
            btnStl.Click += (_, _) => OpenPath(m_strLastStl);
            bar.Controls.Add(btnStl);

            btnFolder = new Button { Text = "Open output folder", Left = 196, Top = 6, Width = 180, Height = 30 };
            btnFolder.Click += (_, _) => OpenPath(m_strOutDir);
            bar.Controls.Add(btnFolder);

            // Add docked controls so the Fill report box sits behind the edge
            // panels in the z-order and they claim their space first.
            Controls.Add(txtReport);
            Controls.Add(pnl);
            Controls.Add(bar);

            chkBox.CheckedChanged += (_, _) => SyncBoxMode();
            SyncBoxMode();
        }

        void SyncBoxMode()
        {
            bool box = chkBox.Checked;
            numLen.Enabled  = !box;
            numBoxL.Enabled = box;
            numBoxW.Enabled = box;
            numBoxH.Enabled = box;
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
                Blunt       = chkBlunt.Checked,
                QAllowMW    = (double)numQ.Value,
                LDRetention = (double)numRet.Value,
                Sweep       = chkSweep.Checked,
                OutDir      = m_strOutDir,
            };

            btnGen.Enabled = false;
            btnStl.Enabled = false;
            lblStatus.Text = "Designing and voxelizing...";
            Cursor = Cursors.WaitCursor;

            try
            {
                var (report, stl) = await Task.Run(() =>
                {
                    var r = WaveriderJob.Design(inp);
                    if (!r.Opt.Aero.Valid || r.Surfaces is null)
                        return (r.Report, (string?)null);

                    string stem = WaveriderJob.Stem(inp);
                    double vol = WaveriderJob.Export(r, stem, out string stlp, out _);

                    string rep = r.Report + Environment.NewLine +
                                 $"Voxel-model volume   : {vol:F3} m^3" + Environment.NewLine +
                                 $"Wrote {stlp}";

                    if (inp.Sweep) rep += Environment.NewLine + RunSweeps(r, inp, stem);
                    return (rep, (string?)stlp);
                });

                txtReport.Text = report.Replace("\n", Environment.NewLine);
                m_strLastStl   = stl;
                btnStl.Enabled = stl is not null;
                lblStatus.Text = stl is not null ? "Done." : "Done - no valid geometry.";
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

            return $"Wrote {stem}_mach_sweep.csv and _aoa_sweep.csv";
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
            var l = new Label
            {
                Text = text, Left = 8, Top = y, Width = 330, Height = 18,
                Font = new Font("Segoe UI", 9F, FontStyle.Bold), ForeColor = Color.FromArgb(60, 60, 90),
            };
            pnl.Controls.Add(l);
            y += 24;
        }

        static NumericUpDown AddNumeric(Panel pnl, string label, decimal min, decimal max,
                                        int dp, decimal inc, decimal val, ref int y)
        {
            var l = new Label { Text = label, Left = 12, Top = y + 4, Width = 180, Height = 20 };
            var n = new NumericUpDown
            {
                Left = 198, Top = y, Width = 130,
                DecimalPlaces = dp, Minimum = min, Maximum = max, Increment = inc, Value = val,
            };
            pnl.Controls.Add(l);
            pnl.Controls.Add(n);
            y += 30;
            return n;
        }

        static CheckBox AddCheck(Panel pnl, string label, bool chk, ref int y)
        {
            var c = new CheckBox { Text = label, Left = 12, Top = y, Width = 320, Height = 22, Checked = chk };
            pnl.Controls.Add(c);
            y += 28;
            return c;
        }
    }
}
