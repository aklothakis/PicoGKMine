//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - Windows Forms GUI entry point.
//

using System.Windows.Forms;

namespace WaveriderForge.Gui
{
    internal static class GuiProgram
    {
        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new MainForm());
        }
    }
}
