//
// SPDX-License-Identifier: Apache-2.0
//
// Waverider Forge - lightweight in-window 3D preview.
//
// A self-contained shaded triangle renderer drawn with GDI+ (no OpenGL or
// native viewer needed). It orthographically projects the body's triangles,
// sorts them back-to-front (painter's algorithm) and flat-shades each face.
// Drag with the mouse to orbit, scroll to zoom.
//

using System.Drawing;
using System.Drawing.Drawing2D;
using System.Numerics;
using System.Windows.Forms;

namespace WaveriderForge.Gui
{
    public sealed class Viewport3D : Panel
    {
        Vector3[] m_v = Array.Empty<Vector3>();   // normalized vertices, 3 per triangle
        Vector3[] m_n = Array.Empty<Vector3>();   // per-face normals
        int       m_count;

        float m_yaw = -0.8f, m_pitch = 0.5f, m_zoom = 1.0f;
        Point m_last;
        bool  m_drag;

        public Viewport3D()
        {
            DoubleBuffered = true;
            BackColor      = Color.FromArgb(32, 34, 44);

            MouseDown  += (_, e) => { m_drag = true; m_last = e.Location; Focus(); };
            MouseUp    += (_, _) => m_drag = false;
            MouseMove  += OnMove;
            MouseWheel += (_, e) => { m_zoom = Math.Clamp(m_zoom * (e.Delta > 0 ? 1.1f : 0.9f), 0.25f, 6f); Invalidate(); };
        }

        void OnMove(object? sender, MouseEventArgs e)
        {
            if (!m_drag) return;
            m_yaw   += (e.X - m_last.X) * 0.01f;
            m_pitch  = Math.Clamp(m_pitch + (e.Y - m_last.Y) * 0.01f, -1.5f, 1.5f);
            m_last   = e.Location;
            Invalidate();
        }

        public void Clear() { m_count = 0; m_v = Array.Empty<Vector3>(); m_n = Array.Empty<Vector3>(); Invalidate(); }

        public void SetTriangles(IReadOnlyList<(Vector3 a, Vector3 b, Vector3 c)> tris)
        {
            m_count = tris.Count;
            m_v = new Vector3[m_count * 3];
            m_n = new Vector3[m_count];

            var min = new Vector3(float.MaxValue);
            var max = new Vector3(float.MinValue);
            foreach (var t in tris)
            {
                min = Vector3.Min(min, Vector3.Min(t.a, Vector3.Min(t.b, t.c)));
                max = Vector3.Max(max, Vector3.Max(t.a, Vector3.Max(t.b, t.c)));
            }

            Vector3 c   = (min + max) * 0.5f;
            float   ext = MathF.Max(max.X - min.X, MathF.Max(max.Y - min.Y, max.Z - min.Z));
            if (ext <= 0) ext = 1;
            float inv = 2f / ext;

            for (int i = 0; i < m_count; i++)
            {
                var t = tris[i];
                Vector3 a = (t.a - c) * inv, b = (t.b - c) * inv, d = (t.c - c) * inv;
                m_v[i * 3] = a; m_v[i * 3 + 1] = b; m_v[i * 3 + 2] = d;
                Vector3 nrm = Vector3.Cross(b - a, d - a);
                m_n[i] = nrm.LengthSquared() > 1e-12f ? Vector3.Normalize(nrm) : Vector3.UnitZ;
            }
            Invalidate();
        }

        // Model axes: X = length, Y = span, Z = up. Rotated into view space with
        // Y up on screen and Z toward the viewer.
        Vector3 Rot(Vector3 p)
        {
            float x = p.X, up = p.Z, depth = p.Y;
            float cy = MathF.Cos(m_yaw),   sy = MathF.Sin(m_yaw);
            float x1 =  x * cy + depth * sy;
            float z1 = -x * sy + depth * cy;
            float cp = MathF.Cos(m_pitch), sp = MathF.Sin(m_pitch);
            float y2 = up * cp - z1 * sp;
            float z2 = up * sp + z1 * cp;
            return new Vector3(x1, y2, z2);
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            var g = e.Graphics;

            if (m_count == 0)
            {
                TextRenderer.DrawText(g, "Generate to preview the waverider  (drag to orbit, scroll to zoom)",
                    Font, ClientRectangle, Color.Gray,
                    TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
                return;
            }

            g.SmoothingMode = SmoothingMode.AntiAlias;

            int w = ClientSize.Width, h = ClientSize.Height;
            float S = MathF.Min(w, h) * 0.42f * m_zoom;
            float cx = w * 0.5f, cy = h * 0.5f;
            var light = Vector3.Normalize(new Vector3(0.35f, 0.45f, 1.0f));

            // Depth sort (far first).
            var depth = new float[m_count];
            var order = new int[m_count];
            var rv = new Vector3[m_count * 3];
            for (int i = 0; i < m_count; i++)
            {
                Vector3 a = Rot(m_v[i * 3]), b = Rot(m_v[i * 3 + 1]), d = Rot(m_v[i * 3 + 2]);
                rv[i * 3] = a; rv[i * 3 + 1] = b; rv[i * 3 + 2] = d;
                depth[i] = (a.Z + b.Z + d.Z) / 3f;
                order[i] = i;
            }
            Array.Sort(depth, order);

            var pts = new PointF[3];
            foreach (int i in order)
            {
                Vector3 a = rv[i * 3], b = rv[i * 3 + 1], d = rv[i * 3 + 2];
                pts[0] = new PointF(cx + a.X * S, cy - a.Y * S);
                pts[1] = new PointF(cx + b.X * S, cy - b.Y * S);
                pts[2] = new PointF(cx + d.X * S, cy - d.Y * S);

                Vector3 nr = Rot(m_n[i]);
                float lit = MathF.Max(0.18f, MathF.Abs(Vector3.Dot(Vector3.Normalize(nr), light)));
                int R = (int)Math.Clamp(95 * lit * 1.4f, 0, 255);
                int G = (int)Math.Clamp(150 * lit * 1.4f, 0, 255);
                int B = (int)Math.Clamp(215 * lit * 1.4f, 0, 255);

                using var br = new SolidBrush(Color.FromArgb(R, G, B));
                g.FillPolygon(br, pts);
            }
        }
    }
}
