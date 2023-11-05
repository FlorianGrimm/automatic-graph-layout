using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.LargeGraphLayout;
using Microsoft.Msagl.Miscellaneous.LayoutEditing;
using Microsoft.Msagl.Routing;
using GeomNode = Microsoft.Msagl.Core.Layout.Node;

namespace Microsoft.Msagl.Miscellaneous.LayoutEditing
{
    /// <summary>
    /// 
    /// </summary>
    public class IncrementalDragger {
        private GeometryGraph graph { get; set; }

        private readonly double nodeSeparation;
        private readonly LayoutAlgorithmSettings layoutSettings;
        private List<BumperPusher> listOfPushers = new List<BumperPusher>();
        private readonly GeomNode[] pushingNodesArray;
        /// <summary>
        /// it is smaller graph that needs to be refreshed by the viewer
        /// </summary>
        public GeometryGraph ChangedGraph;
        private Dictionary<EdgeGeometry,LabelFixture> labelFixtures=new Dictionary<EdgeGeometry, LabelFixture>();
        /// <summary>
        /// 
        /// </summary>
        /// <param name="pushingNodes">the nodes are already at the correct positions</param>
        /// <param name="graph"></param>
        /// <param name="layoutSettings"></param>
        public IncrementalDragger(IEnumerable<GeomNode> pushingNodes, GeometryGraph graph, LayoutAlgorithmSettings layoutSettings) {
            this.graph = graph;
            this.nodeSeparation = layoutSettings.NodeSeparation;
            this.layoutSettings = layoutSettings;
            this.pushingNodesArray = pushingNodes as GeomNode[] ?? pushingNodes.ToArray();
            Debug.Assert(this.pushingNodesArray.All(n => DefaultClusterParent(n) == null) ||
                          (new Set<GeomNode>(this.pushingNodesArray.Select(n => n.ClusterParent))).Count == 1,
                                    "dragged nodes have to belong to the same cluster");
            this.InitBumperPushers();
        }

        private void InitBumperPushers() {
            if (this.pushingNodesArray.Length == 0) {
                return;
            }

            var cluster = DefaultClusterParent(this.pushingNodesArray[0]);
            if (cluster == null) {
                this.listOfPushers.Add(new BumperPusher(this.graph.Nodes, this.nodeSeparation, this.pushingNodesArray));
            } else {
                this.listOfPushers.Add( new BumperPusher(cluster.Nodes.Concat(cluster.Clusters), this.nodeSeparation,
                                                             this.pushingNodesArray));
                do {
                    var pushingCluster = cluster;
                    cluster = DefaultClusterParent(cluster);
                    if (cluster == null) {
                        break;
                    }

                    this.listOfPushers.Add(new BumperPusher(cluster.Nodes.Concat(cluster.Clusters), this.nodeSeparation,
                                                       new[] {pushingCluster}));

                } while (true);
            }
        }

        private static Cluster DefaultClusterParent(GeomNode n) {
            return n.ClusterParent;
        }

        private void RunPushers() {
            for (int i = 0; i < this.listOfPushers.Count;i++ ) {
                var bumperPusher = this.listOfPushers[i];
                bumperPusher.PushNodes();
                var cluster = DefaultClusterParent(bumperPusher.FirstPushingNode());
                if (cluster == null || cluster== this.graph.RootCluster) {
                    break;
                }

                var box = cluster.BoundaryCurve.BoundingBox;
                cluster.CalculateBoundsFromChildren(this.layoutSettings.ClusterMargin);
                Debug.Assert(cluster.Nodes.All(n => cluster.BoundingBox.Contains(n.BoundingBox)));
                var newBox = cluster.BoundaryCurve.BoundingBox;
                if (newBox == box) {
                    break;
                }
                this.listOfPushers[i + 1].UpdateRTreeByChangedNodeBox(cluster, box);
            } 
        }




        /// <summary>
        /// 
        /// </summary>
        /// <param name="delta"></param>
        public void Drag(Point delta) {
            if(delta.Length>0) {
                foreach (var n in this.pushingNodesArray) {
                    n.Center += delta;
                    var cl = n as Cluster;
                    if (cl != null) {
                        cl.DeepContentsTranslation(delta, true);
                    }
                }
            }

            this.RunPushers();
            this.RouteChangedEdges();
        }

        private void RouteChangedEdges() {
            this.ChangedGraph = this.GetChangedFlatGraph();
            var changedClusteredGraph = LgInteractor.CreateClusteredSubgraphFromFlatGraph(this.ChangedGraph, this.graph);


            this.InitLabelFixtures(changedClusteredGraph);
            var router = new SplineRouter(changedClusteredGraph, this.layoutSettings.EdgeRoutingSettings.Padding,
                                          this.layoutSettings.EdgeRoutingSettings.PolylinePadding,
                                          this.layoutSettings.EdgeRoutingSettings.ConeAngle,
                                          this.layoutSettings.EdgeRoutingSettings.BundlingSettings) {
                                              ContinueOnOverlaps
                                                  = true
                                          };

            router.Run();
            this.PositionLabels(changedClusteredGraph);

        }

        private void PositionLabels(GeometryGraph changedClusteredGraph) {
            foreach (var edge in changedClusteredGraph.Edges) {
                this.PositionEdge(edge);
            }
        }

        private void PositionEdge(Edge edge) {
            LabelFixture lf;
            if (!this.labelFixtures.TryGetValue(edge.EdgeGeometry, out lf)) {
                return;
            }

            var curve = edge.Curve;
            var lenAtLabelAttachment = curve.Length*lf.RelativeLengthOnCurve;
            var par = curve.GetParameterAtLength(lenAtLabelAttachment);
            var tang = curve.Derivative(par);
            var norm = (lf.RightSide ? tang.Rotate90Cw() : tang.Rotate90Ccw()).Normalize()*lf.NormalLength;
            edge.Label.Center = curve[par] + norm;           
        }

        private void InitLabelFixtures(GeometryGraph changedClusteredGraph) {
            foreach (var edge in changedClusteredGraph.Edges) {
                this.InitLabelFixture(edge);
            }
        }

        private void InitLabelFixture(Edge edge) {
            if (edge.Label == null) {
                return;
            }

            if (this.labelFixtures.ContainsKey(edge.EdgeGeometry)) {
                return;
            }

            var attachmentPar = edge.Curve.ClosestParameter(edge.Label.Center);
            
            var curve = edge.Curve;
            var tang = curve.Derivative(attachmentPar);
            var normal = tang.Rotate90Cw();
            var fromCurveToLabel = edge.Label.Center - curve[attachmentPar];
            var fixture = new LabelFixture() {
                RelativeLengthOnCurve = curve.LengthPartial(0, attachmentPar)/curve.Length,
                NormalLength = fromCurveToLabel.Length,
                RightSide = fromCurveToLabel*normal>0
            };

            this.labelFixtures[edge.EdgeGeometry] = fixture;
        }

        private GeometryGraph GetChangedFlatGraph() {
            var changedNodes = this.GetChangedNodes();
            var changedEdges = this.GetChangedEdges(changedNodes);
            foreach (var e in changedEdges) {
                changedNodes.Insert(e.Source);
                changedNodes.Insert(e.Target);
            }

            var changedGraph = new GeometryGraph {
                Nodes = new SimpleNodeCollection(changedNodes),
                Edges = new SimpleEdgeCollection(changedEdges)
            };
            return changedGraph;
        }

        private List<Edge> GetChangedEdges(Set<Node> changedNodes) {
            var list = new List<Edge>();
            var box = Rectangle.CreateAnEmptyBox();
            foreach(var node in changedNodes) {
                box.Add(node.BoundaryCurve.BoundingBox);
            }

            var boxPoly = box.Perimeter();

            foreach (var e in this.graph.Edges) {
                if (this.EdgeNeedsRouting(ref box, e, boxPoly, changedNodes)) {
                    list.Add(e);
                }
            }

            return list;
        }

        private bool EdgeNeedsRouting(ref Rectangle box, Edge edge, Polyline boxPolyline, Set<Node> changedNodes) {
            if (edge.Curve == null) {
                return true;
            }

            if (changedNodes.Contains(edge.Source) || changedNodes.Contains(edge.Target)) {
                return true;
            }

            if (edge.Source.BoundaryCurve.BoundingBox.Intersects(box) ||
                edge.Target.BoundaryCurve.BoundingBox.Intersects(box)) {
                return true;
            }

            if (!edge.BoundingBox.Intersects(box)) {
                return false;
            }

            return Curve.CurveCurveIntersectionOne(boxPolyline, edge.Curve, false) != null;
        }

        private Set<Node> GetChangedNodes() {
            return new Set<Node>(this.listOfPushers.SelectMany(p => p.FixedNodes));
        }
    }
}