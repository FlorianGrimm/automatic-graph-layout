﻿using System;
using System.Linq;
using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Core.Routing;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// this class nudges the edges
    /// </summary>
    public class HubDebugger {

#if TEST_MSAGL
        private readonly MetroGraphData mgd;
        private readonly BundlingSettings bundlingSettings;
        static internal void ShowHubsWithAdditionalICurves(MetroGraphData mgd, BundlingSettings bundlingSettings, params ICurve[] iCurves
            ) {
            HubDebugger hd = new HubDebugger(mgd, bundlingSettings);
            if (iCurves != null) {
                var dc = hd.CreateDebugCurves(iCurves);
                LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(dc);
            }
        }

        static internal void ShowHubsTurnByTurn(MetroGraphData mgd, BundlingSettings bundlingSettings, bool withIdeal) {
            if (!withIdeal) {
                return;
            }

            foreach (var v in mgd.Stations) {
                ShowHubsWithHighligtedStation(mgd, bundlingSettings, v);
            }
        }

        static internal void ShowHubsWithHighligtedStation(MetroGraphData mgd, BundlingSettings bundlingSettings, Station highlightedNode) {
            HubDebugger hd = new HubDebugger(mgd, bundlingSettings);
            List<DebugCurve> debugCurves = hd.CreateDebugCurves();
            debugCurves.Add(new DebugCurve(100,1, "magenta", CurveFactory.CreateCircle(3, highlightedNode.Position)));
            debugCurves.Add(new DebugCurve(100, 0.1, "green", highlightedNode.BoundaryCurve));
            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(debugCurves);
        }

        internal HubDebugger(MetroGraphData mgd, BundlingSettings bundlingSettings) {
            this.mgd = mgd;
            this.bundlingSettings = bundlingSettings;
        }

        internal List<DebugCurve> CreateDebugCurves(params ICurve[] iCurves) {
            List<DebugCurve> curves = new List<DebugCurve>();
            curves.AddRange(this.Obstacles());
            //curves.AddRange(BBoxes());

            curves.AddRange(this.Hubs());
            curves.AddRange(this.Paths());
            //curves.AddRange(Bundles());
            //curves.AddRange(IdealHubs());
            //curves.AddRange(IdealHubsWithNeighbors());
            //curves.AddRange(IdealBundles());
            if (iCurves != null) {
                foreach (var iCurve in iCurves) {
                    if (iCurve != null) {
                        curves.Add(new DebugCurve(100, 0.2, "Brown", iCurve));
                    }
                }
            }
            return curves;
        }

        private IEnumerable<DebugCurve> Obstacles() {
            return this.mgd.TightTree.GetAllLeaves().Select(n => new DebugCurve(100,0.5,"black", n)).Concat(
                    this.mgd.LooseTree.GetAllLeaves().Select(n => new DebugCurve(100, 0.1, "green", n)));
        }

        private IEnumerable<DebugCurve> BBoxes() {
            return this.mgd.Stations.Where(n => n.IsRealNode).Select(n => new DebugCurve("red", CurveFactory.CreateCircle(n.BoundaryCurve.BoundingBox.Diagonal / 2, n.Position)));
        }

        private IEnumerable<DebugCurve> GraphNodes() {
            var nodes = new Set<ICurve>(this.mgd.Edges.Select(e => e.SourcePort.Curve).Concat(this.mgd.Edges.Select(e => e.TargetPort.Curve)));
            return nodes.Select(n => new DebugCurve(100, 1, "black", n));
        }

        private IEnumerable<DebugCurve> Hubs() {
            return this.mgd.Stations.Select(station => new DebugCurve(100, 1, "blue", CurveFactory.CreateCircle(Math.Max(station.Radius, 1.0), station.Position)));
        }

        private IEnumerable<DebugCurve> IdealHubs() {
            return this.mgd.VirtualNodes().Select(station => new DebugCurve(100, 1, "black",
                CurveFactory.CreateCircle(HubRadiiCalculator.CalculateIdealHubRadiusWithNeighbors(this.mgd, this.bundlingSettings, station), station.Position)));
        }

        private IEnumerable<DebugCurve> IdealHubsWithNeighbors() {
            return this.mgd.VirtualNodes().Select(station => new DebugCurve(200, 1, "black",
                CurveFactory.CreateCircle(HubRadiiCalculator.CalculateIdealHubRadiusWithNeighbors(this.mgd, this.bundlingSettings, station), station.Position)));
        }

        private IEnumerable<DebugCurve> Bundles() {
            return this.mgd.VirtualEdges().Select(pr => new DebugCurve(100, 1, "red", new LineSegment(pr.Item1.Position, pr.Item2.Position)));
        }

        private IEnumerable<DebugCurve> IdealBundles() {
            List<DebugCurve> dc = new List<DebugCurve>();
            foreach (var edge in this.mgd.VirtualEdges()) {
                var node = edge.Item1;
                var adj = edge.Item2;

                double width = this.mgd.GetWidth(node, adj, this.bundlingSettings.EdgeSeparation);
                dc.Add(new DebugCurve(0.1, "black", Intersections.Create4gon(node.Position, adj.Position, width, width)));
            }

            return dc;
        }

        private IEnumerable<DebugCurve> Paths() {
            List<DebugCurve> dc = new List<DebugCurve>();
            foreach (var metroline in this.mgd.Metrolines) {
                dc.Add(new DebugCurve(100, 1, "black", metroline.Polyline));
            }

            return dc;
        }
#endif

        /*void SaveAsBitmap(string filename, GViewer gViewer) {
            int w = (int)Math.Ceiling((double) gViewer.Graph.Width);
            int h = (int)Math.Ceiling((double) gViewer.Graph.Height);

            Bitmap bitmap = new Bitmap(w, h, PixelFormat.Format32bppPArgb);
            using (Graphics graphics = Graphics.FromImage(bitmap))
                DrawGeneral(w, h, graphics, gViewer);

            bitmap.Save(filename);
        }

        void DrawGeneral(int w, int h, Graphics graphics, GViewer gViewer) {
            graphics.SmoothingMode = SmoothingMode.HighQuality;
            graphics.TextRenderingHint = TextRenderingHint.AntiAliasGridFit;
            graphics.CompositingQuality = CompositingQuality.HighQuality;
            graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;

            DrawAll(w, h, graphics, gViewer);
        }

        void DrawAll(int w, int h, Graphics graphics, GViewer gViewer) {
            //fill the whole image
            graphics.FillRectangle(new SolidBrush(Draw.MsaglColorToDrawingColor(gViewer.Graph.Attr.BackgroundColor)),
                                   new RectangleF(0, 0, w, h));

            //calculate the transform
            double s = 1.0;
            Graph g = gViewer.Graph;
            double x = 0.5 * w - s * (g.Left + 0.5 * g.Width);
            double y = 0.5 * h + s * (g.Bottom + 0.5 * g.Height);

            graphics.Transform = new Matrix((float)s, 0, 0, (float)-s, (float)x, (float)y);
            Draw.DrawPrecalculatedLayoutObject(graphics, gViewer.DGraph);
        } */

    }
}