using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// Wrapper for geometry graph with coinciding edges:
    ///  'real' nodes stand for edge ends (source,target)
    ///  'virtual' nodes stand for polyline control points
    ///  
    ///  'real' edges are original graph edges
    ///  'virtual' edges are polyline segments
    /// </summary>
    internal class MetroGraphData {
        internal List<Station> Stations;

        /// info on the edges passing through a couple
        private Dictionary<Tuple<Station, Station>, StationEdgeInfo> edgeInfoDictionary;

        /// current ink
        private double ink;

        /// Edges
        private List<Metroline> metrolines;

        ///  position -> (node)
        internal Dictionary<Point, Station> PointToStations;
        private readonly EdgeGeometry[] regularEdges;

        ///  objects to check crossings and calculate distances
        internal Intersections looseIntersections;
        internal Intersections tightIntersections;

        ///  objects to check crossings and calculate distances
        internal CdtIntersections cdtIntersections;

        private Dictionary<EdgeGeometry, Set<Polyline>> EdgeLooseEnterable { get; set; }
        private Dictionary<EdgeGeometry, Set<Polyline>> EdgeTightEnterable { get; set; }

        internal Func<Port, Polyline> LoosePolylineOfPort;

        /// <summary>
        /// triangulation
        /// </summary>
        internal Cdt Cdt;

        internal MetroGraphData(EdgeGeometry[] regularEdges,
            RectangleNode<Polyline, Point> looseTree, RectangleNode<Polyline, Point> tightTree,
            BundlingSettings bundlingSettings, Cdt cdt,
            Dictionary<EdgeGeometry, Set<Polyline>> edgeLooseEnterable, Dictionary<EdgeGeometry, Set<Polyline>> edgeTightEnterable, Func<Port, Polyline> loosePolylineOfPort) {
            //Debug.Assert(cdt != null);
            this.regularEdges = regularEdges;
            if (cdt != null) {
                this.Cdt = cdt;
            } else {
                this.Cdt = BundleRouter.CreateConstrainedDelaunayTriangulation(looseTree);
            }

            this.EdgeLooseEnterable = edgeLooseEnterable;
            this.EdgeTightEnterable = edgeTightEnterable;
            this.LoosePolylineOfPort = loosePolylineOfPort;

            this.looseIntersections = new Intersections(this, bundlingSettings, looseTree, station => station.EnterableLoosePolylines);
            this.tightIntersections = new Intersections(this, bundlingSettings, tightTree, station => station.EnterableTightPolylines);
            this.cdtIntersections = new CdtIntersections(this, bundlingSettings);

            this.Initialize(false);
        }

        internal double Ink {
            get { return this.ink; }
        }

        internal EdgeGeometry[] Edges {
            get { return this.regularEdges; }
        }

        internal IEnumerable<Station> VirtualNodes() {
            return this.Stations.Where(s => !s.IsRealNode);
        }

        internal List<Metroline> Metrolines { get { return this.metrolines; } }

        internal RectangleNode<Polyline, Point> LooseTree { get { return this.looseIntersections.obstacleTree; } }

        internal RectangleNode<Polyline, Point> TightTree { get { return this.tightIntersections.obstacleTree; } }

        internal IEnumerable<Tuple<Station, Station>> VirtualEdges() {
            return this.edgeInfoDictionary.Keys;
        }

        /// <summary>
        /// number of real edges passing the edge uv
        /// </summary>
        internal int RealEdgeCount(Station u, Station v) {
            var couple = u < v ? new Tuple<Station, Station>(u, v) : new Tuple<Station, Station>(v, u);
            StationEdgeInfo cw;
            if (this.edgeInfoDictionary.TryGetValue(couple, out cw)) {
                return cw.Count;
            }

            return 0;
        }

        /// <summary>
        /// real edges passing the node
        /// </summary>
        internal List<MetroNodeInfo> MetroNodeInfosOfNode(Station node) {
            return node.MetroNodeInfos;
        }

        /// <summary>
        /// real edges passing the edge uv
        /// </summary>
        internal StationEdgeInfo GetIjInfo(Station u, Station v) {
            var couple = u < v ? new Tuple<Station, Station>(u, v) : new Tuple<Station, Station>(v, u);
            return this.edgeInfoDictionary[couple];
        }

        /// <summary>
        /// Move node to the specified position
        /// </summary>
        internal void MoveNode(Station node, Point newPosition) {
            Point oldPosition = node.Position;
            this.PointToStations.Remove(oldPosition);
            this.PointToStations.Add(newPosition, node);
            node.Position = newPosition;

            //move curves
            foreach (MetroNodeInfo metroNodeInfo in this.MetroNodeInfosOfNode(node)) {
                metroNodeInfo.PolyPoint.Point = newPosition;
            }

            //update lengths
            foreach (MetroNodeInfo e in this.MetroNodeInfosOfNode(node)) {
                var metroLine = e.Metroline;
                var prev = e.PolyPoint.Prev.Point;
                var succ = e.PolyPoint.Next.Point;
                metroLine.Length += (succ - newPosition).Length + (prev - newPosition).Length
                    - (succ - oldPosition).Length - (prev - oldPosition).Length;
            }

            //update ink
            foreach (var adj in node.Neighbors) {
                this.ink += (newPosition - adj.Position).Length - (oldPosition - adj.Position).Length;
            }

            //update neighbors order
            this.SortNeighbors(node);
            foreach (var adj in node.Neighbors) {
                this.SortNeighbors(adj);
            }
        }

        internal double GetWidth(Station u, Station v, double edgeSeparation) {
            var couple = u < v ? new Tuple<Station, Station>(u, v) : new Tuple<Station, Station>(v, u);
            StationEdgeInfo cw;
            if (this.edgeInfoDictionary.TryGetValue(couple, out cw)) {
                return cw.Width + (cw.Count - 1) * edgeSeparation;
            }

            return 0;
        }

        internal double GetWidth(IEnumerable<Metroline> metrolines, double edgeSeparation) {
            double width = 0;
            foreach (var metroline in metrolines) {
                width += metroline.Width;
            }
            int count = metrolines.Count();
            width += count > 0 ? (count - 1) * edgeSeparation : 0;
            Debug.Assert(ApproximateComparer.GreaterOrEqual(width, 0));
            return width;
        }

        /// <summary>
        /// Initialize data
        /// </summary>
        internal void Initialize(bool initTightTree) {
            //TimeMeasurer.DebugOutput("bundle graph data initializing...");

            this.SimplifyRegularEdges();

            this.InitializeNodeData();

            this.InitializeEdgeData();

            this.InitializeVirtualGraph();

            this.InitializeEdgeNodeInfo(initTightTree);

            this.InitializeCdtInfo();

//            Debug.Assert(looseIntersections.HubPositionsAreOK());
  //          Debug.Assert(tightIntersections.HubPositionsAreOK());
        
        }

        /// <summary>
        /// remove self-cycles
        /// </summary>
        private void SimplifyRegularEdges() {
            foreach (var edge in this.regularEdges) {
                this.SimplifyRegularEdge(edge);
            }
        }

        /// <summary>
        /// change the polyline by removing cycles
        /// </summary>
        private void SimplifyRegularEdge(EdgeGeometry edge) {
            Polyline polyline = (Polyline)edge.Curve;

            var stack = new Stack<Point>();
            var seen = new Set<Point>();
            for (var p = polyline.EndPoint; p != null; p = p.Prev) {
                var v = p.Point;
                if (seen.Contains(p.Point)) {
                    var pp = p.Next;
                    do {
                        var u = stack.Peek();
                        if (u != v) {
                            seen.Remove(u);
                            stack.Pop();
                            pp = pp.Next;
                        }
                        else {
                            break;
                        }
                    } while (true);
                    pp.Prev = p.Prev;
                    pp.Prev.Next = pp;
                }
                else {
                    stack.Push(v);
                    seen.Insert(v);
                }
            }
        }

        private void InitializeNodeData() {
            this.Stations = new List<Station>();
            //create indexes
            this.PointToStations = new Dictionary<Point, Station>();
            foreach (var edge in this.regularEdges) {
                Polyline poly = (Polyline)edge.Curve;
                this.ProcessPolylinePoints(poly);
            }
        }

        private void ProcessPolylinePoints(Polyline poly) {
            var pp = poly.StartPoint;
            this.RegisterStation(pp, true);

            for (pp = pp.Next; pp != poly.EndPoint; pp = pp.Next) {
                this.RegisterStation(pp, false);
            }

            this.RegisterStation(pp, true);
            
        }

        private void RegisterStation(PolylinePoint pp, bool isRealNode) {
            if (!this.PointToStations.ContainsKey(pp.Point)) {
                // Filippo Polo: assigning the return value of the assignment operator (i.e. a = b = c) does not work well in Sharpkit.
                Station station = new Station(this.Stations.Count, isRealNode, pp.Point);
                this.PointToStations[pp.Point] = station;
                this.Stations.Add(station);
            }
            else {
#if TEST_MSAGL
                var s = this.PointToStations[pp.Point];
                Debug.Assert(s.IsRealNode == isRealNode);
#endif
            }
            
        }

        private void InitializeEdgeData() {
            this.metrolines = new List<Metroline>();
            for (int i = 0; i < this.regularEdges.Length; i++) {
                EdgeGeometry geomEdge= this.regularEdges[i];
                this.InitEdgeData(geomEdge, i);
            }
        }

        private void InitEdgeData(EdgeGeometry geomEdge, int index) {
            var metroEdge = new Metroline((Polyline)geomEdge.Curve, geomEdge.LineWidth, this.EdgeSourceAndTargetFunc(geomEdge), index);
            this.metrolines.Add(metroEdge);
            this.PointToStations[metroEdge.Polyline.Start].BoundaryCurve = geomEdge.SourcePort.Curve;
            this.PointToStations[metroEdge.Polyline.End].BoundaryCurve = geomEdge.TargetPort.Curve;
        }

        internal Func<Tuple<Polyline, Polyline>> EdgeSourceAndTargetFunc(EdgeGeometry geomEdge) {
            return
                () =>
                new Tuple<Polyline, Polyline>(this.LoosePolylineOfPort(geomEdge.SourcePort),
                                               this.LoosePolylineOfPort(geomEdge.TargetPort));
        }

        /// <summary>
        /// Initialize graph comprised of stations and their neighbors
        /// </summary>
        private void InitializeVirtualGraph() {
            Dictionary<Station, Set<Station>> neighbors = new Dictionary<Station, Set<Station>>();
            foreach (var metroline in this.metrolines) {
                Station u = this.PointToStations[metroline.Polyline.Start];
                Station v;
                for (var p = metroline.Polyline.StartPoint; p.Next != null; p = p.Next, u = v) {
                    v = this.PointToStations[p.Next.Point];
                    CollectionUtilities.AddToMap(neighbors, u, v);
                    CollectionUtilities.AddToMap(neighbors, v, u);
                }
            }

            foreach (var s in this.Stations) {
                s.Neighbors = neighbors[s].ToArray();
            }
        }

        private StationEdgeInfo GetUnorderedIjInfo(Station i, Station j) {
            Debug.Assert(i != j);
            return (i < j ? this.GetOrderedIjInfo(i, j) : this.GetOrderedIjInfo(j, i));
        }

        private StationEdgeInfo GetOrderedIjInfo(Station i, Station j) {
            Debug.Assert(i < j);
            var couple = new Tuple<Station, Station>(i, j);
            StationEdgeInfo cw;
            if (this.edgeInfoDictionary.TryGetValue(couple, out cw)) {
                return cw;
            }

            this.edgeInfoDictionary[couple] = cw = new StationEdgeInfo();
            return cw;
        }

        private void InitializeEdgeNodeInfo(bool initTightTree) {
            this.edgeInfoDictionary = new Dictionary<Tuple<Station, Station>, StationEdgeInfo>();

            this.InitMetroNodeInfos(initTightTree);
            this.SortNeighbors();
            this.InitEdgeIjInfos();
            this.ink = 0;
            foreach (var edge in this.VirtualEdges()) {
                this.ink += (edge.Item1.Position - edge.Item2.Position).Length;
            }
        }

        private void InitMetroNodeInfos(bool initTightTree) {
            for (int i = 0; i < this.metrolines.Count; i++) {
                var metroline = this.metrolines[i];
                this.InitMetroNodeInfos(metroline);
                this.InitNodeEnterableLoosePolylines(metroline, this.regularEdges[i]);
                if (initTightTree) {
                    this.InitNodeEnterableTightPolylines(metroline, this.regularEdges[i]);
                }

                metroline.UpdateLengths();
            }
        }

        private void InitMetroNodeInfos(Metroline metroline) {
            for (var pp = metroline.Polyline.StartPoint; pp != null; pp = pp.Next) {
                Station station = this.PointToStations[pp.Point];
                station.MetroNodeInfos.Add(new MetroNodeInfo(metroline, station, pp));
            }
        }

        private void InitNodeEnterableLoosePolylines(Metroline metroline, EdgeGeometry regularEdge) {
            //If we have groups, EdgeLooseEnterable are precomputed.
            var metrolineEnterable = this.EdgeLooseEnterable != null ? this.EdgeLooseEnterable[regularEdge] : new Set<Polyline>();

            for (var p = metroline.Polyline.StartPoint.Next; p!=null && p.Next != null; p = p.Next) {
                var v = this.PointToStations[p.Point];
                if (v.EnterableLoosePolylines != null) {
                    v.EnterableLoosePolylines *= metrolineEnterable;
                } else {
                    v.EnterableLoosePolylines = new Set<Polyline>(metrolineEnterable);
                }
            }

            this.AddLooseEnterableForMetrolineStartEndPoints(metroline);
        }

        private void AddLooseEnterableForMetrolineStartEndPoints(Metroline metroline) {
            this.AddLooseEnterableForEnd(metroline.Polyline.Start);
            this.AddLooseEnterableForEnd(metroline.Polyline.End);
        }

        private void AddTightEnterableForMetrolineStartEndPoints(Metroline metroline) {
            this.AddTightEnterableForEnd(metroline.Polyline.Start);
            this.AddTightEnterableForEnd(metroline.Polyline.End);
        }

        private Dictionary<Point, Set<Polyline>> cachedEnterableLooseForEnd = new Dictionary<Point, Set<Polyline>>();

        private void AddLooseEnterableForEnd(Point point) {
            Station station = this.PointToStations[point];
            if (!this.cachedEnterableLooseForEnd.ContainsKey(point)) {
                foreach (var poly in this.LooseTree.AllHitItems(point)) {
                    if (Curve.PointRelativeToCurveLocation(point, poly) == PointLocation.Inside) {
                        station.AddEnterableLoosePolyline(poly);
                    }
                }

                this.cachedEnterableLooseForEnd.Add(point, station.EnterableLoosePolylines);
            }
            else {
                station.EnterableLoosePolylines = this.cachedEnterableLooseForEnd[point];
            }

            //foreach (var poly in LooseTree.AllHitItems(point))
            //    if (Curve.PointRelativeToCurveLocation(point, poly) == PointLocation.Inside)
            //        station.EnterableLoosePolylines.Insert(poly);
        }

        private void AddTightEnterableForEnd(Point point) {
            Station station = this.PointToStations[point];
            foreach (var poly in this.TightTree.AllHitItems(point)) {
                if (Curve.PointRelativeToCurveLocation(point, poly) == PointLocation.Inside) {
                    station.AddEnterableTightPolyline(poly);
                }
            }
        }

        private void InitNodeEnterableTightPolylines(Metroline metroline, EdgeGeometry regularEdge) {
            //If we have groups, EdgeTightEnterable are precomputed.
            var metrolineEnterable = this.EdgeTightEnterable != null ? this.EdgeTightEnterable[regularEdge] : new Set<Polyline>();

            for (var p = metroline.Polyline.StartPoint.Next; p!=null && p.Next != null; p = p.Next) {
                var v = this.PointToStations[p.Point];
                Set<Polyline> nodeEnterable = v.EnterableTightPolylines;
                if (nodeEnterable != null) {
                    v.EnterableTightPolylines = nodeEnterable * metrolineEnterable;
                } else {
                    v.EnterableTightPolylines = new Set<Polyline>(metrolineEnterable);
                }
            }

            this.AddTightEnterableForMetrolineStartEndPoints(metroline);
        }

        private void SortNeighbors() {
            //counter-clockwise sorting
            foreach (var station in this.Stations) {
                this.SortNeighbors(station);
            }
        }

        private void SortNeighbors(Station station) {
            //nothing to sort
            if (station.Neighbors.Length <= 2) {
                return;
            }

            Point pivot = station.Neighbors[0].Position;
            Point center = station.Position;
            Array.Sort(station.Neighbors, delegate(Station u, Station v) {
                return Point.GetOrientationOf3Vectors(pivot - center, u.Position - center, v.Position - center);
            });              
        }

        private void InitEdgeIjInfos() {
            foreach (Metroline metroLine in this.metrolines) {
                var poly = metroLine.Polyline;
                var u = this.PointToStations[poly.Start];
                Station v;
                for (var p = metroLine.Polyline.StartPoint; p.Next != null; p = p.Next, u = v) {
                    v = this.PointToStations[p.Next.Point];
                    var info = this.GetUnorderedIjInfo(u, v);
                    info.Width += metroLine.Width;
                    info.Metrolines.Add(metroLine);
                }
            }
       }

        private void InitializeCdtInfo() {
            RectangleNode<CdtTriangle,Point> cdtTree = this.Cdt.GetCdtTree();
            foreach (var station in this.Stations) {
                station.CdtTriangle = cdtTree.FirstHitNode(station.Position, IntersectionCache.Test).UserData;
                Debug.Assert(station.CdtTriangle != null);
            }
        }

        internal bool PointIsAcceptableForEdge(Metroline metroline, Point point) {
            if (this.LoosePolylineOfPort == null) {
                return true;
            }

            var polys = metroline.sourceAndTargetLoosePolylines();
            return Curve.PointRelativeToCurveLocation(point, polys.Item1) == PointLocation.Outside &&
                   Curve.PointRelativeToCurveLocation(point, polys.Item2) == PointLocation.Outside;
        }
    }
}