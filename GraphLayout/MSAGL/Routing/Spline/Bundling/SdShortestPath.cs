using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Microsoft.Msagl.Routing.Visibility;
using Microsoft.Msagl.DebugHelpers;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class SdShortestPath {
        internal VisibilityGraph VisibilityGraph { get; set; }
        internal Func<EdgeGeometry, List<Shape>> MakeTransparentShapesOfEdgeGeometry { get; set; }
        internal BundlingSettings BundlingSettings { get; set; }
        internal EdgeGeometry[] EdgeGeometries { get; set; }
        internal RectangleNode<Polyline, Point> ObstacleHierarchy { get; set; }

        private SdVertex[] vertexArray;
        internal Cdt CdtProperty { get; set; }
        private Set<CdtEdge> Gates { get; set; }

        private readonly Dictionary<EdgeGeometry, List<SdBoneEdge>> EdgesToRoutes = new Dictionary<EdgeGeometry, List<SdBoneEdge>>();
        private readonly Dictionary<EdgeGeometry, SdVertex> EdgesToRouteSources = new Dictionary<EdgeGeometry, SdVertex>();
        private EdgeGeometry CurrentEdgeGeometry;
        private Dictionary<VisibilityVertex, SdVertex> VisibilityVerticesToSdVerts;
        private double LengthCoefficient;
        private GenericBinaryHeapPriorityQueue<SdVertex> Queue;
        private double LowestCostToTarget;
        private SdVertex ClosestTargetVertex;
        private double capacityOverlowPenaltyMultiplier;
        private Polyline sourceLoosePoly;
        private Polyline targetLoosePoly;

        internal SdShortestPath(Func<EdgeGeometry, List<Shape>> makeTransparentShapesOfEdgeGeometryAndGetTheShapes, Cdt cdt, Set<CdtEdge> gates) {
            this.MakeTransparentShapesOfEdgeGeometry = makeTransparentShapesOfEdgeGeometryAndGetTheShapes;
            this.CdtProperty = cdt;
            this.Gates = gates;
        }

        private void CreateGraphElements() {
            foreach (var sdVertex in this.vertexArray) {
                var vv = sdVertex.VisibilityVertex;
                foreach (var vEdge in vv.InEdges) {
                    var boneEdge = new SdBoneEdge(vEdge, this.VisibilityVerticesToSdVerts[vEdge.Source], this.VisibilityVerticesToSdVerts[vEdge.Target]);
                    var otherSdVertex = this.VisibilityVerticesToSdVerts[vEdge.Source];
                    sdVertex.InBoneEdges.Add(boneEdge);
                    otherSdVertex.OutBoneEdges.Add(boneEdge);
                }
            }
        }

        private void CreateRoutingGraph() {
            this.vertexArray = new SdVertex[this.VisibilityGraph.VertexCount];
            int i = 0;
            this.VisibilityVerticesToSdVerts = new Dictionary<VisibilityVertex, SdVertex>();
            foreach (var v in this.VisibilityGraph.Vertices()) {
                var sdVert = new SdVertex(v);
                this.vertexArray[i++] = sdVert;
                this.VisibilityVerticesToSdVerts[v] = sdVert;
            }

            this.CreateGraphElements();
        }

        /// <summary>
        /// routing of the edges minimizing (ink+path length+capacity penalty)
        /// </summary>
        internal void RouteEdges() {
            this.Initialize();
            this.RestoreCapacities();
            foreach (var edgeGeometry in this.EdgeGeometries) {
                this.EdgesToRoutes[edgeGeometry] = this.RouteEdge(edgeGeometry);
            }

            this.RerouteEdges();

            foreach (var edgeGeometry in this.EdgeGeometries) {
                this.SetEdgeGeometryCurve(edgeGeometry);
            }
        }

        private void SetEdgeGeometryCurve(EdgeGeometry edgeGeometry) {
            Polyline poly = new Polyline();
            SdVertex curV = this.EdgesToRouteSources[edgeGeometry];
            poly.AddPoint(curV.Point);
            foreach (var edge in this.EdgesToRoutes[edgeGeometry]) {
                if (edge.SourcePoint == curV.Point) {
                    poly.AddPoint(edge.TargetPoint);
                    curV = edge.Target;
                }
                else {
                    poly.AddPoint(edge.SourcePoint);
                    curV = edge.Source;
                }
            }

            edgeGeometry.Curve = poly;
            var clusterSourcePort = edgeGeometry.SourcePort as ClusterBoundaryPort;
            if (clusterSourcePort != null) {
                ExtendPolylineStartToClusterBoundary(poly, clusterSourcePort.Curve);
            }

            var clusterTargetPort = edgeGeometry.TargetPort as ClusterBoundaryPort;
            if (clusterTargetPort != null) {
                ExtendPolylineEndToClusterBoundary(poly, clusterTargetPort.Curve);
            }
        }

        private static void ExtendPolylineEndToClusterBoundary(Polyline poly, ICurve curve) {
            var par = curve.ClosestParameter(poly.End);
            poly.AddPoint(curve[par]);
        }

        private static void ExtendPolylineStartToClusterBoundary(Polyline poly, ICurve curve) {
            var par = curve.ClosestParameter(poly.Start);
            poly.PrependPoint(curve[par]);
        }

        private void RerouteEdges() {
            this.RestoreCapacities();
            foreach (var edgeGeometry in this.EdgeGeometries) {
                var newRoute = this.RerouteEdge(edgeGeometry);
                this.EdgesToRoutes[edgeGeometry] = newRoute;
            }
        }

        private void RestoreCapacities() {
            if (this.CdtProperty != null) {
                this.CdtProperty.RestoreEdgeCapacities();
            }
        }

        /// <summary>
        /// Reroute edge
        /// </summary>
        private List<SdBoneEdge> RerouteEdge(EdgeGeometry edgeGeometry) {
            var route = this.EdgesToRoutes[edgeGeometry];

            foreach (var edge in route) {
                edge.RemoveOccupiedEdge();
            }

            return this.RouteEdge(edgeGeometry);
        }

        private List<SdBoneEdge> RouteEdge(EdgeGeometry edgeGeometry) {
            this.CurrentEdgeGeometry = edgeGeometry;
            for (int i = 0; i < this.vertexArray.Length; i++) {
                var sdv = this.vertexArray[i];
                sdv.SetPreviousToNull();
                sdv.IsSourceOfRouting = sdv.IsTargetOfRouting = false;
            }

            var transparentShapes = this.MakeTransparentShapesOfEdgeGeometry(edgeGeometry);
            var ret = this.RouteEdgeWithGroups();

            foreach (var shape in transparentShapes) {
                shape.IsTransparent = false;
            }

            /*List<LineSegment> ls = new List<LineSegment>();
            foreach (var e in ret)
                ls.Add(new LineSegment(e.SourcePoint, e.TargetPoint));
            SplineRouter.ShowVisGraph(this.VisibilityGraph, ObstacleHierarchy.GetAllLeaves(), null, ls);*/

            return ret;
        }

        private List<SdBoneEdge> RouteEdgeWithGroups() {
            for (int i = 0; i < 2; i++) {
                this.SetLengthCoefficient();
                this.Queue = new GenericBinaryHeapPriorityQueue<SdVertex>();
                this.SetPortVerticesAndObstacles(this.CurrentEdgeGeometry.SourcePort, true, out this.sourceLoosePoly);
                this.SetPortVerticesAndObstacles(this.CurrentEdgeGeometry.TargetPort, false, out this.targetLoosePoly);
                List<SdBoneEdge> ret = this.RouteOnKnownSourceTargetVertices((this.CurrentEdgeGeometry.TargetPort.Location - this.CurrentEdgeGeometry.SourcePort.Location).Normalize(), i == 0);
                if (ret != null) {
                    return ret;
                }

                for (int j = 0; j < this.vertexArray.Length; j++) {
                    this.vertexArray[j].SetPreviousToNull();
                }
            }
            //SplineRouter.ShowVisGraph(this.VisibilityGraph, ObstacleHierarchy.GetAllLeaves(), null, new[] { new LineSegment(
           // CurrentEdgeGeometry.SourcePort.Location, CurrentEdgeGeometry.TargetPort.Location)});
            throw new InvalidOperationException(); //cannot find a path
        }

        private List<SdBoneEdge> RouteOnKnownSourceTargetVertices(Point pathDirection, bool lookingForMonotonePath) {
            this.LowestCostToTarget = Double.PositiveInfinity;
            this.ClosestTargetVertex = null;
            while (this.Queue.Count > 0) {
                double hu;
                SdVertex bestNode = this.Queue.Dequeue(out hu);
                if (hu >= this.LowestCostToTarget) {
                    continue;
                }
                //update the rest
                for (int i = 0; i < bestNode.OutBoneEdges.Count; i++) {
                    var outBoneEdge = bestNode.OutBoneEdges[i];
                    if (outBoneEdge.IsPassable) {
                        this.ProcessOutcomingBoneEdge(bestNode, outBoneEdge, pathDirection, lookingForMonotonePath);
                    }
                }

                for (int i = 0; i < bestNode.InBoneEdges.Count; i++) {
                    var inBoneEdge = bestNode.InBoneEdges[i];
                    if (inBoneEdge.IsPassable) {
                        this.ProcessIncomingBoneEdge(bestNode, inBoneEdge, pathDirection, lookingForMonotonePath);
                    }
                }
            }

            return this.GetPathAndUpdateRelatedCosts();
        }

        private void ProcessOutcomingBoneEdge(SdVertex v, SdBoneEdge outBoneEdge, Point pathDirection, bool lookingForMonotonePath) {
            Debug.Assert(v == outBoneEdge.Source);
            if (lookingForMonotonePath && pathDirection * (outBoneEdge.TargetPoint - outBoneEdge.SourcePoint) < 0) {
                return;
            }

            this.ProcessBoneEdge(v, outBoneEdge.Target, outBoneEdge);
        }

        private void ProcessIncomingBoneEdge(SdVertex v, SdBoneEdge inBoneEdge, Point pathDirection, bool lookingForMonotonePath) {
            Debug.Assert(v == inBoneEdge.Target);
            if (lookingForMonotonePath && pathDirection * (inBoneEdge.SourcePoint - inBoneEdge.TargetPoint) < 0) {
                return;
            }

            this.ProcessBoneEdge(v, inBoneEdge.Source, inBoneEdge);
        }

        private void ProcessBoneEdge(SdVertex v, SdVertex queueCandidate, SdBoneEdge boneEdge) {
            double newCost = this.GetEdgeAdditionalCost(boneEdge, v.Cost);
            if (queueCandidate.Cost <= newCost) {
                return;
            }

            queueCandidate.Cost = newCost;
            queueCandidate.PrevEdge = boneEdge;
            if (this.Queue.ContainsElement(queueCandidate)) {
                this.Queue.DecreasePriority(queueCandidate, newCost);
            } else {
                if (queueCandidate.IsTargetOfRouting) {
                    double costToTarget = 0;
                    if (this.CurrentEdgeGeometry.TargetPort is ClusterBoundaryPort) {
                        costToTarget = this.LengthCoefficient * (queueCandidate.Point - this.CurrentEdgeGeometry.TargetPort.Location).Length;
                    }

                    if (newCost + costToTarget < this.LowestCostToTarget) {
                        this.LowestCostToTarget = newCost + costToTarget;
                        this.ClosestTargetVertex = queueCandidate;
                    }
                    return; //do not enqueue the target vertices
                }
                this.Enqueue(queueCandidate);
            }
        }

#if TEST_MSAGL
        //        void DebugShow(SdSimpleVertex prevElement, SdBoneEdge outBoneEdge) {
        //            SplineRouter.ShowVisGraph(this.VisibilityGraph,
        //                                      this.ObstacleHierarchy.GetAllLeaves(),
        //                                      prevElement.BoneEdge != null ?
        //                                       new[] { new LineSegment(prevElement.BoneEdge.SourcePoint, prevElement.BoneEdge.TargetPoint) } : null,
        //                                      new[] {(ICurve) new LineSegment(outBoneEdge.SourcePoint, outBoneEdge.TargetPoint) ,
        //                                      new Ellipse(5,5,outBoneEdge.TargetPoint)});
        //        }
        //
        //        void DebugShowIn(SdSimpleVertex prevElement, SdBoneEdge inBoneEdge) {
        //            SplineRouter.ShowVisGraph(this.VisibilityGraph,
        //                                      this.ObstacleHierarchy.GetAllLeaves(),
        //                                      prevElement.BoneEdge != null ?
        //                                                                    new[] { new LineSegment(prevElement.BoneEdge.SourcePoint, prevElement.BoneEdge.TargetPoint) } : null,
        //                                      new[] {(ICurve) new LineSegment(inBoneEdge.SourcePoint, inBoneEdge.TargetPoint) ,
        //                                      new Ellipse(2,2,inBoneEdge.TargetPoint)});
        //        }
#endif

        private List<SdBoneEdge> GetPathAndUpdateRelatedCosts() {
            //restore the path by moving backwards
            var current = this.ClosestTargetVertex;
            if (current == null) {
                return null;
            }

            var result = new List<SdBoneEdge>();

            while (current.PrevEdge != null) {
                result.Add(current.PrevEdge);
                this.RegisterPathInBoneEdge(current.PrevEdge);
                current = current.Prev;
            }

            this.EdgesToRouteSources[this.CurrentEdgeGeometry] = current;

            result.Reverse();

            Debug.Assert(result.Count > 0);
            return result;
        }

        private void RegisterPathInBoneEdge(SdBoneEdge boneEdge) {
            boneEdge.AddOccupiedEdge();
            if (this.CdtProperty != null && this.BundlingSettings.CapacityOverflowCoefficient != 0) {
                this.UpdateResidualCostsOfCrossedCdtEdges(boneEdge);
            }
        }

        private void UpdateResidualCostsOfCrossedCdtEdges(SdBoneEdge boneEdge) {
            foreach (var cdtEdge in boneEdge.CrossedCdtEdges) {
                if (this.AdjacentToSourceOrTarget(cdtEdge)) {
                    continue;
                }

                if (cdtEdge.ResidualCapacity == cdtEdge.Capacity) {
                    cdtEdge.ResidualCapacity -= this.CurrentEdgeGeometry.LineWidth;
                } else {
                    cdtEdge.ResidualCapacity -= (this.CurrentEdgeGeometry.LineWidth + this.BundlingSettings.EdgeSeparation);
                }
                //TODO: can we have negative here?
                //Debug.Assert(cdtEdge.ResidualCapacity >= 0);
            }
        }

        private double H(SdVertex v) {
            return v.Cost + this.LengthCoefficient * (v.Point - this.CurrentEdgeGeometry.TargetPort.Location).Length;
        }

        private double GetEdgeAdditionalCost(SdBoneEdge boneEdge, double previousCost) {
            var len = (boneEdge.TargetPoint - boneEdge.SourcePoint).Length;
            return this.LengthCoefficient * len + previousCost +
                (boneEdge.IsOccupied ? 0 : this.BundlingSettings.InkImportance * len) + this.CapacityOverflowCost(boneEdge);
        }

        private double CapacityOverflowCost(SdBoneEdge boneEdge) {
            if (this.CdtProperty == null || this.BundlingSettings.CapacityOverflowCoefficient == 0) {
                return 0;
            }

            double ret = 0;
            foreach (var cdtEdge in this.CrossedCdtEdgesOfBoneEdge(boneEdge)) {
                ret += this.CostOfCrossingCdtEdgeLocal(this.capacityOverlowPenaltyMultiplier, this.BundlingSettings, this.CurrentEdgeGeometry, cdtEdge);
            }
            return ret;
        }

        private IEnumerable<CdtEdge> CrossedCdtEdgesOfBoneEdge(SdBoneEdge boneEdge) {
            if (boneEdge.CrossedCdtEdges != null) {
                return boneEdge.CrossedCdtEdges;
            }
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=368
            boneEdge.CrossedCdtEdges = ThreadBoneEdgeThroughCdt(boneEdge);
            return boneEdge.CrossedCdtEdges;
#else
            return boneEdge.CrossedCdtEdges = this.ThreadBoneEdgeThroughCdt(boneEdge);
#endif
        }

        private Set<CdtEdge> ThreadBoneEdgeThroughCdt(SdBoneEdge boneEdge) {
            var start = boneEdge.SourcePoint;
            var currentTriangle = boneEdge.Source.Triangle;
            Debug.Assert(Cdt.PointIsInsideOfTriangle(start, currentTriangle));
            var crossedEdges = new Set<CdtEdge>();
            var end = boneEdge.TargetPoint;
            if (Cdt.PointIsInsideOfTriangle(end, currentTriangle)) {
                return crossedEdges;
            }

            var threader = new CdtThreader(currentTriangle, start, end);
            while (threader.MoveNext()) {
                CdtEdge piercedEdge = threader.CurrentPiercedEdge;
                Debug.Assert(piercedEdge != null);
                if (this.Gates.Contains(piercedEdge)) {
                    crossedEdges.Insert(piercedEdge);
                }
            }

            /*
            CdtEdge piercedEdge = CdtIntersections.FindFirstPiercedEdge(currentTriangle, start, end, out negativeSign, out positiveSign, this.Cdt );
            Debug.Assert(piercedEdge != null);
      
            do {
                if (Gates.Contains(piercedEdge))
                    crossedEdges.Insert(piercedEdge);
            }
            while (CdtIntersections.FindNextPierced(start, end, ref currentTriangle, ref piercedEdge, ref negativeSign, ref positiveSign));
            */
            //if(ddd(boneEdge))
            //CdtSweeper.ShowFront(Cdt.GetTriangles(),null,new []{new LineSegment(boneEdge.SourcePoint,boneEdge.TargetPoint)}, crossedEdges.Select(e=>new LineSegment(e.upperSite.Point,e.lowerSite.Point)));

            return crossedEdges;
        }

        //TODO: method incorrect since id doesn't check AdjacentToSourceOrTarget condition
        internal static double CostOfCrossingCdtEdge(double capacityOverflMult, BundlingSettings bundlingSettings, EdgeGeometry currentEdgeGeometry, CdtEdge e) {
            var w = currentEdgeGeometry.LineWidth;
            if (e.Capacity != e.ResidualCapacity) {
                w += bundlingSettings.EdgeSeparation;
            }

            var del = e.ResidualCapacity - w;
            if (del >= 0) {
                return 0;
            }

            return -del * capacityOverflMult;
        }

        private double CostOfCrossingCdtEdgeLocal(double capacityOverflMult, BundlingSettings bundlingSettings, EdgeGeometry currentEdgeGeometry, CdtEdge e) {
            if (this.AdjacentToSourceOrTarget(e)) {
                return 0;
            }

            return CostOfCrossingCdtEdge(capacityOverflMult, bundlingSettings, currentEdgeGeometry, e);
        }

        private bool AdjacentToSourceOrTarget(CdtEdge e) {
            return e.upperSite.Owner == this.sourceLoosePoly || e.lowerSite.Owner == this.sourceLoosePoly || e.upperSite.Owner == this.targetLoosePoly || e.lowerSite.Owner == this.targetLoosePoly;
        }

        private void SetLengthCoefficient() {
            double idealEdgeLength = this.GetIdealDistanceBetweenSourceAndTarget(this.CurrentEdgeGeometry);
            this.LengthCoefficient = this.BundlingSettings.PathLengthImportance / idealEdgeLength;
        }

        private double GetIdealDistanceBetweenSourceAndTarget(EdgeGeometry edgeGeometry) {
            return (edgeGeometry.SourcePort.Location - edgeGeometry.TargetPort.Location).Length;
        }

        private void SetPortVerticesAndObstacles(Port port, bool sources, out Polyline poly) {
            var cbport = port as ClusterBoundaryPort;
            if (cbport != null) {
                //SplineRouter.ShowVisGraph(this.VisibilityGraph, this.ObstacleHierarchy.GetAllLeaves(), null, new[]{cbport.LoosePolyline});
                poly = cbport.LoosePolyline;
                foreach (var point in poly) {
                    double initialCost = 0;
                    if (sources) {
                        //we prefer paths starting from the center of the group
                        initialCost = this.LengthCoefficient * (point - this.CurrentEdgeGeometry.SourcePort.Location).Length;
                    }
                    this.AddAndEnqueueVertexToEnds(point, sources, initialCost);
                }
            }
            else {
                var anywherePort = port as HookUpAnywhereFromInsidePort;
                if (anywherePort != null) {
                    poly = anywherePort.LoosePolyline;
                    foreach (var point in poly) {
                        this.AddAndEnqueueVertexToEnds(point, sources, 0);
                    }
                }
                else {
                    this.AddAndEnqueueVertexToEnds(port.Location, sources, 0);
                    var polys = this.ObstacleHierarchy.GetNodeItemsIntersectingRectangle(port.Curve.BoundingBox).ToArray();
                    double mindiag = polys[0].BoundingBox.Diagonal;
                    poly = polys[0];
                    for (int i = 1; i < polys.Length; i++) {
                        var pl = polys[i];
                        var diag = pl.BoundingBox.Diagonal;
                        if (diag < mindiag) {
                            mindiag = diag;
                            poly = pl;
                        }
                    }

                }
            }
        }

        private void Enqueue(SdVertex simpleSdVertex) {
            this.Queue.Enqueue(simpleSdVertex, this.H(simpleSdVertex));
        }

        private void AddAndEnqueueVertexToEnds(Point point, bool isSource, double initialCost) {
            var v = this.FindVertex(point);
            var sdVert = this.VisibilityVerticesToSdVerts[v];
            if (isSource) {
                sdVert.IsSourceOfRouting = true;
                sdVert.Cost = initialCost;
                this.Enqueue(sdVert);
            }
            else {
                sdVert.IsTargetOfRouting = true;
            }
        }

        private VisibilityVertex FindVertex(Point p) {
            return this.VisibilityGraph.FindVertex(p) ?? this.VisibilityGraph.FindVertex(ApproximateComparer.Round(p));
            /*  if (r == null) {
                  SplineRouter.ShowVisGraph(this.VisibilityGraph, this.ObstacleHierarchy.GetAllLeaves(), null,
                  new[] { new Ellipse(5, 5, p) });               
              }
              return r;*/
        }

        private void Initialize() {
            this.CreateRoutingGraph();
            if (this.CdtProperty != null) {
                this.capacityOverlowPenaltyMultiplier = CapacityOverflowPenaltyMultiplier(this.BundlingSettings);
                this.SetVertexTriangles();
                this.CalculateCapacitiesOfTrianglulation();
            }
        }

        private void CalculateCapacitiesOfTrianglulation() {
            foreach (var e in this.Gates) {
                CalculateCdtEdgeCapacityForEdge(e);
            }
        }

        private static void CalculateCdtEdgeCapacityForEdge(CdtEdge e) {
            if (e.Constrained || e.CwTriangle == null || e.CcwTriangle == null) {
                return; //this is a convex hull edge or an obstacle edge
            }

            var startPoly = e.upperSite.Owner as Polyline;
            var endPoly = e.lowerSite.Owner as Polyline;
            if (startPoly != endPoly) {
                //e.Capacity = Polygon.Distance(new Polygon(startPoly), new Polygon(endPoly)); //todo: cache this
                //e.Capacity = (e.upperSite.Point - e.lowerSite.Point).Length;
                double distA = Polygon.Distance(new Polygon(startPoly), e.lowerSite.Point);
                double distB = Polygon.Distance(new Polygon(endPoly), e.upperSite.Point);
                e.Capacity = (distA + distB) / 2;
            }
            //else - it is a diagonal of an obstacle, do not care
        }

        private void SetVertexTriangles() {
            var triangleTree =
                RectangleNode<CdtTriangle,Point>.CreateRectangleNodeOnEnumeration(
                    this.CdtProperty.GetTriangles().Select(t => new RectangleNode<CdtTriangle,Point>(t, t.BoundingBox())));
            var vertexTree =
                RectangleNode<SdVertex,Point>.CreateRectangleNodeOnEnumeration(
                    this.vertexArray.Select(v => new RectangleNode<SdVertex,Point>(v, new Rectangle(v.Point))));

            RectangleNodeUtils.CrossRectangleNodes(triangleTree, vertexTree, this.TryToAssigenTriangleToVertex);
//            foreach (var v in vertexArray) {
  //              Debug.Assert(v.Triangle != null);
    //        }
        }

        private void TryToAssigenTriangleToVertex(CdtTriangle triangle, SdVertex vertex) {
            if (vertex.Triangle != null) {
                return;
            }

            if (Cdt.PointIsInsideOfTriangle(vertex.Point, triangle)) {
                vertex.Triangle = triangle;
            }
        }

        internal static double CapacityOverflowPenaltyMultiplier(BundlingSettings bundlingSettings) {
            return bundlingSettings.CapacityOverflowCoefficient * (bundlingSettings.PathLengthImportance + bundlingSettings.InkImportance);
        }

        /// <summary>
        /// compute cdt edges crossed by paths
        /// </summary>
        internal void FillCrossedCdtEdges(Dictionary<EdgeGeometry, Set<CdtEdge>> crossedCdtEdges) {
            foreach (var geometryEdge in this.EdgeGeometries) {
                this.SetPortVerticesAndObstacles(geometryEdge.SourcePort, true, out this.sourceLoosePoly);
                this.SetPortVerticesAndObstacles(geometryEdge.TargetPort, false, out this.targetLoosePoly);

                //crossedCdtEdges.Add(geometryEdge, new Set<CdtEdge>());
                foreach (var boneEdge in this.EdgesToRoutes[geometryEdge]) {
                    foreach (var cdtEdge in this.CrossedCdtEdgesOfBoneEdge(boneEdge)) {
                        if (this.AdjacentToSourceOrTarget(cdtEdge)) {
                            continue;
                        }

                        CollectionUtilities.AddToMap(crossedCdtEdges, geometryEdge, cdtEdge);
                    }
                }
            }
        }
    }
}
