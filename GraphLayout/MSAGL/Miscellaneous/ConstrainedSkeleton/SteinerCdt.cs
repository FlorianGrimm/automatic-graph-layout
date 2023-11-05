using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.Visibility;
using Point = Microsoft.Msagl.Core.Geometry.Point;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;
//using Microsoft.Msagl.Miscellaneous.Rounded;

namespace Microsoft.Msagl.Miscellaneous.ConstrainedSkeleton
{
    internal class SteinerCdt
    {
        private Dictionary<Point, int> _pointsToIndices = new Dictionary<Point, int>();
        private readonly List<Point> _pointList = new List<Point>();
        private readonly Set<SymmetricTuple<int>> _segments = new Set<SymmetricTuple<int>>();

        public Dictionary<int, VisibilityVertex> _outPoints = new Dictionary<int, VisibilityVertex>();
        public VisibilityGraph _visGraph;
        private readonly IEnumerable<LgNodeInfo> _nodeInfos;
        private Rectangle _boundingBox;
        private readonly Random _random = new Random(3);
        public SteinerCdt(VisibilityGraph visGraph, IEnumerable<LgNodeInfo> nodeInfos)
        {
            this._visGraph = visGraph;
            this._nodeInfos = nodeInfos;
            //comment out by jyoti
            //MakeSureThatNodeBoundariesAreInVisGraph(nodeInfos);
        }

        private void MakeSureThatNodeBoundariesAreInVisGraph(IEnumerable<LgNodeInfo> nodeInfos)
        {
            foreach (var nodeInfo in nodeInfos) {
                foreach (var polypoint in nodeInfo.BoundaryOnLayer.PolylinePoints) {
                    this._visGraph.AddVertex(polypoint.Point);
                }
            }
        }

        public void SaveInputFilePoly(string path)
        {
            this.IndexPoints();
            this.InitSegments();

            Debug.Assert(this.TopologyForCallingTriangleIsCorrect());

            using (var file = new StreamWriter(path))
            {
                this.WritePoints(file);
                this.WriteSegments(file);
                this.WriteHoles(file);
            }
        }

        private void WriteHoles(StreamWriter file)
        {
            file.WriteLine("# holes");
            file.WriteLine(this._nodeInfos.Count());
            int j = 0;
            foreach (var ni in this._nodeInfos)
            {
                var c = ni.Center;
                file.WriteLine((j + 1) + " " + c.X + " " + c.Y);
                j++;
            }
        }

        private void WriteSegments(StreamWriter file)
        {
            file.WriteLine("# segments");
            file.WriteLine(this._segments.Count + " 0");
            int i = 1;
            foreach (var seg in this._segments)
            {
                file.WriteLine(i + " " + (seg.A + 1) + " " + (seg.B + 1));
                i++;
            }
        }

        private void WritePoints(StreamWriter file)
        {
            file.WriteLine("# vertices");
            file.WriteLine(this._pointsToIndices.Count + " 2 0 0");
            foreach (var tuple in this._pointsToIndices)
            {
                file.WriteLine(tuple.Value + 1 + " " + tuple.Key.X + " " + tuple.Key.Y);
            }
        }

        public void LoadOutputFileNode(string path)
        {
            int oldPoints = this._pointList.Count;
            using (var file = new StreamReader(path))
            {
                string line = file.ReadLine();
                if (line == null) {
                    throw new InvalidOperationException("unexpected end of file");
                }

                Regex.Split(line, @"\s{2,}");

                int numVertices = Int32.Parse(line.Split(' ').First());

                for (int i = 0; i < numVertices; i++)
                {
                    line = file.ReadLine();
                    if (line == null) {
                        break;
                    }

                    line = line.TrimStart(' ');
                    var lineParsed = Regex.Split(line, @"\s{2,}");
                    int ind = Int32.Parse(lineParsed[0]) - 1;
                    if (ind < oldPoints) {
                        this._outPoints[ind] = this._visGraph.FindVertex(this._pointList[ind]);
                    } else
                    {
                        double x = Double.Parse(lineParsed[1]);
                        double y = Double.Parse(lineParsed[2]);
                        this._outPoints[ind] = this._visGraph.AddVertex(new Point(x, y));
                    }
                }
            }
        }


        public void LoadOutputFileSides(string path)
        {
            this._visGraph.ClearEdges();
            using (var file = new StreamReader(path))
            {
                var line = file.ReadLine();
                if (line == null) {
                    throw new Exception("unexpected end of file");
                }

                line = line.TrimStart(' ');

                int numTriangles = Int32.Parse(line.Split(' ').First());

                for (int i = 0; i < numTriangles; i++)
                {
                    line = file.ReadLine();
                    if (line == null) {
                        break;
                    }

                    line = line.TrimStart(' ');
                    var lineParsed = Regex.Split(line, @"\s{2,}");
                    int id0 = Int32.Parse(lineParsed[1]) - 1;
                    int id1 = Int32.Parse(lineParsed[2]) - 1;
                    int id2 = Int32.Parse(lineParsed[3]) - 1;

                    var v0 = this._outPoints[id0];
                    var v1 = this._outPoints[id1];
                    var v2 = this._outPoints[id2];
                    this.AddVisEdge(v0, v1);
                    this.AddVisEdge(v1, v2);
                    this.AddVisEdge(v2, v0);
                }
            }
        }

        public void AddVisEdge(VisibilityVertex v0, VisibilityVertex v1)
        {
            if (v0 != v1)
            {
                VisibilityGraph.AddEdge(v0, v1);
            }
        }

        private void InitSegment(Point p1, Point p2)
        {
            int id0 = this._pointsToIndices[p1];
            int id1 = this._pointsToIndices[p2];
            this._segments.Insert(new SymmetricTuple<int>(id0, id1));
        }

        private void IndexPoints()
        {
            this._pointsToIndices.Clear();
            this.IndexVisGraphVertices();
            this.IndexNodeInfos();
            this.CreateAndIndexBoundingBox();
        }

        private void IndexNodeInfos()
        {
            foreach (var ni in this._nodeInfos)
            {
                this.AddNodeBoundaryToPointIndices(ni);
            }
        }

        private void IndexVisGraphVertices()
        {
            if (this._visGraph == null) {
                return;
            }

            foreach (var v in this._visGraph.Vertices())
            {
                this._pointsToIndices[v.Point] = this._pointsToIndices.Count;
                this._pointList.Add(v.Point);
            }
        }

        private void CreateAndIndexBoundingBox()
        {
            this._boundingBox = Rectangle.CreateAnEmptyBox();
            foreach (var p in this._pointsToIndices.Keys) {
                this._boundingBox.Add(p);
            }

            this._boundingBox.Pad(1);
            this.IndexAPoint(this._boundingBox.LeftBottom);
            this.IndexAPoint(this._boundingBox.RightBottom);
            this.IndexAPoint(this._boundingBox.LeftTop);
            this.IndexAPoint(this._boundingBox.RightTop);
            //SplineRouter.ShowVisGraph(_visGraph, null, new[] { _boundingBox.Perimeter() }, null);
        }

        private void AddNodeBoundaryToPointIndices(LgNodeInfo ni)
        {
            foreach (var polylinePoint in ni.BoundaryOnLayer.PolylinePoints) {
                this.IndexAPoint(polylinePoint.Point);
            }
        }

        private void IndexAPoint(Point p)
        {
            if (!this._pointsToIndices.ContainsKey(p))
            {
                this._pointsToIndices[p] = this._pointsToIndices.Count;
                this._pointList.Add(p);
                this._visGraph.AddVertex(p);
            }
        }

        private bool TopologyForCallingTriangleIsCorrect()
        {
            Point[] indexToPoints = new Point[this._pointsToIndices.Count];
            foreach (var pp in this._pointsToIndices)
            {
                indexToPoints[pp.Value] = pp.Key;
            }

            var tree =
                new RTree<Point,Point>(this._pointsToIndices.Keys.Select(p => new KeyValuePair<IRectangle<Point>, Point>(new Rectangle(p), p)));
            var badSegs = (from e in this._segments let overlaps = this.GetPointsOverlappingSeg(e, tree, indexToPoints) where overlaps.Count > 2 select e).ToList();

#if TEST_MSAGL
            if (badSegs.Any()) {
                this.ShowInputSegments(badSegs, indexToPoints);
            }
#endif
            return !badSegs.Any();
        }

#if TEST_MSAGL
        private void ShowInputSegments(List<SymmetricTuple<int>> badSegs, Point[] indexToPoints) {
            var l = new List<DebugCurve>();
            foreach (var seg in this._segments)
            {
                var p1 = indexToPoints[seg.A];
                var p2 = indexToPoints[seg.B];
                var ls = new LineSegment(p1, p2);

                string color = badSegs.Contains(seg) ? "red" : "black";
                double width = badSegs.Contains(seg) ? 3 : 1;
                l.Add(new DebugCurve(100, width, color, ls));
            }
            LayoutAlgorithmSettings.ShowDebugCurves(l.ToArray());
        }
#endif

        private List<Point> GetPointsOverlappingSeg(SymmetricTuple<int> seg, RTree<Point, Point> tree, Point[] indexToPoints)
        {
            Point p0 = indexToPoints[seg.A];
            Point p1 = indexToPoints[seg.B];
            var rect = new Rectangle(p0, p1);
            rect.Pad(1e-5);

            Point[] vts = tree.GetAllIntersecting(rect).ToArray();

            double t;
            var vtsOverlapping = vts.Where(p => Point.DistToLineSegment(p, p0, p1, out t) < 1e-5).ToList();

            vtsOverlapping = vtsOverlapping.OrderBy(p => (p - p0).Length).ToList();
            return vtsOverlapping;
        }

        private void InitSegments()
        {
            this._segments.Clear();
            this.InitSegsOfVisGraph();
            foreach (var lgNodeInfo in this._nodeInfos) {
                this.InitSegsOfPolyline(lgNodeInfo.BoundaryOnLayer);
            }

            this.InitSegments(this._boundingBox.LeftTop, this._boundingBox.RightTop, this._boundingBox.RightBottom, this._boundingBox.LeftBottom);
        }

        private void InitSegsOfPolyline(Polyline polyline)
        {
            var pp = polyline.StartPoint;
            int startId = this._pointsToIndices[pp.Point];
            int id = startId;
            do
            {
                pp = pp.Next;
                if (pp == null) {
                    break;
                }

                int nextId = this._pointsToIndices[pp.Point];
                this._segments.Insert(new SymmetricTuple<int>(id, nextId));
                id = nextId;
            } while (true);
            this._segments.Insert(new SymmetricTuple<int>(id, startId));
        }

        private void InitSegments(params Point[] pts)
        {
            for (int i = 0; i < pts.Length - 1; i++) {
                this.InitSegment(pts[i], pts[i + 1]);
            }

            this.InitSegment(pts[pts.Length - 1], pts[0]);
        }

        private void InitSegsOfVisGraph()
        {
            if (this._visGraph == null) {
                return;
            }

            foreach (var v in this._visGraph.Vertices())
            {
                int vId = this._pointsToIndices[v.Point];
                foreach (var e in v.OutEdges) {
                    this._segments.Insert(new SymmetricTuple<int>(vId, this._pointsToIndices[e.Target.Point]));
                }
            }
        }


        public void LaunchTriangleExe(string pathExe, string arguments)
        {
#if !SHARPKIT
            const string triangleMessage =
                "Cannot start Triangle.exe To build Triangle.exe, please open http://www.cs.cmu.edu/~quake/triangle.html and build it by following the instructions from the site. Copy Triange.exe to a directory in your PATH." +
                "Unfortunately we cannot distribute Triangle.exe because of the license restrictions.";
            ProcessStartInfo startInfo = new ProcessStartInfo
            {
                CreateNoWindow = false,
                UseShellExecute = false,
                FileName = pathExe,
                WindowStyle = ProcessWindowStyle.Hidden,
                Arguments = arguments
            };

            try
            {
                using (Process exeProcess = Process.Start(startInfo))
                {
                    if (exeProcess == null)
                    {
                        Environment.Exit(1);
                    }
                    exeProcess.WaitForExit();
                    if (exeProcess.ExitCode != 0) {
                        Environment.Exit(exeProcess.ExitCode);
                    }
                }
            }
            catch (Exception e)
            {
                System.Diagnostics.Debug.WriteLine(e.Message);
                System.Diagnostics.Debug.WriteLine(triangleMessage);
                System.Diagnostics.Debug.WriteLine("Exiting now.");
                Environment.Exit(1);
            }
#endif
        }

        internal void ReadTriangleOutputAndPopulateTheLevelVisibilityGraphFromTriangulation()
        {
            string outPath = Path.Combine(Path.GetTempPath(), "pointssegments" + this._random.Next(10000));
            this.SaveInputFilePoly(outPath + ".poly");
            const string exePath = "triangle.exe";
            string arguments = outPath + ".poly -c -q10 -S100000000";
            this.LaunchTriangleExe(exePath, arguments);
            this.LoadOutputFileNode(outPath + ".1.node");
            this.LoadOutputFileSides(outPath + ".1.ele");
            File.Delete(outPath + ".poly");
            File.Delete(outPath + ".1.node");
            File.Delete(outPath + ".1.ele");
            File.Delete(outPath + ".1.poly");
        }
    }
}
