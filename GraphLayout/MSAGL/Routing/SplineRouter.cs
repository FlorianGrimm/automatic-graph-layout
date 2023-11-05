using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Layout.LargeGraphLayout;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Microsoft.Msagl.Routing.Spline.Bundling;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;
using Microsoft.Msagl.Routing.Visibility;

#if TEST_MSAGL
using Microsoft.Msagl.DebugHelpers;
using System.Diagnostics.CodeAnalysis;
#endif

namespace Microsoft.Msagl.Routing {
  ///<summary>
  /// routing splines around shapes
  ///</summary>
  public class SplineRouter : AlgorithmBase {
        /// <summary>
        /// setting this to true forces the calculation to go on even when node overlaps are present
        /// </summary>
        /// 
        private bool continueOnOverlaps = true;
    public bool ContinueOnOverlaps { get { return this.continueOnOverlaps; } set { this.continueOnOverlaps = value; } }

        private Shape[] rootShapes;

        private IEnumerable<EdgeGeometry> edgeGeometriesEnumeration {
      get {
        if (this._edges != null) {
          foreach (var item in this._edges.Select(e => e.EdgeGeometry)) {
            yield return item;
          }
        }
      }
    }

        private double coneAngle;
        private readonly double tightPadding;

        private double LoosePadding { get; set; }

        private bool rootWasCreated;
        private Shape root;
        private VisibilityGraph visGraph;
        private Dictionary<Shape, Set<Shape>> ancestorSets;
        private readonly Dictionary<Shape, TightLooseCouple> shapesToTightLooseCouples = new Dictionary<Shape, TightLooseCouple>();
        private Dictionary<Port, Shape> portsToShapes;
        private Dictionary<Port, Set<Shape>> portsToEnterableShapes;
        private RTree<Point,Point> portRTree;
        private readonly Dictionary<Point, Polyline> portLocationsToLoosePolylines = new Dictionary<Point, Polyline>();
        private Shape looseRoot;
    internal BundlingSettings BundlingSettings { get; set; }

        private Dictionary<EdgeGeometry, Set<Polyline>> enterableLoose;
        private Dictionary<EdgeGeometry, Set<Polyline>> enterableTight;
        private readonly GeometryGraph geometryGraph;
        private double multiEdgesSeparation = 5;
        private bool routeMultiEdgesAsBundles = true;
    internal bool UseEdgeLengthMultiplier;

    /// <summary>
    /// if set to true the algorithm will try to shortcut a shortest polyline inner points
    /// </summary>
    public bool UsePolylineEndShortcutting = true;
    /// <summary>
    /// if set to true the algorithm will try to shortcut a shortest polyline start and end
    /// </summary>
    public bool UseInnerPolylingShortcutting = true;

    internal bool AllowedShootingStraightLines = true;

    internal double MultiEdgesSeparation {
      get { return this.multiEdgesSeparation; }
      set { this.multiEdgesSeparation = value; }
    }

    /// <summary>
    /// Creates a spline group router for the given graph.
    /// </summary>
    public SplineRouter(GeometryGraph graph, EdgeRoutingSettings edgeRoutingSettings) :
        this(
        graph, edgeRoutingSettings.Padding, edgeRoutingSettings.PolylinePadding, edgeRoutingSettings.ConeAngle,
        edgeRoutingSettings.BundlingSettings) { }


    /// <summary>
    /// Creates a spline group router for the given graph.
    /// </summary>
    public SplineRouter(GeometryGraph graph, double tightTightPadding, double loosePadding, double coneAngle) :
        this(graph, graph.Edges, tightTightPadding, loosePadding, coneAngle, null) {
    }

    /// <summary>
    /// Creates a spline group router for the given graph
    /// </summary>
    public SplineRouter(GeometryGraph graph, double tightTightPadding, double loosePadding, double coneAngle, BundlingSettings bundlingSettings) :
        this(graph, graph.Edges, tightTightPadding, loosePadding, coneAngle, bundlingSettings) {
    }


    /// <summary>
    /// Creates a spline group router for the given graph.
    /// </summary>
    public SplineRouter(GeometryGraph graph, IEnumerable<Edge> edges, double tightPadding, double loosePadding, double coneAngle, BundlingSettings bundlingSettings) {
      ValidateArg.IsNotNull(graph, "graph");
      ValidateArg.IsPositive(tightPadding, "tightPadding");
      ValidateArg.IsPositive(loosePadding, "loosePadding");
      ValidateArg.IsNotNull(edges, "edges");
#if TEST_MSAGL
      // do not run the following check, cluster containment may be incorrect in float mode - need to make sure we handle this
      //graph.CheckClusterConsistency();
#endif

      this._edges = edges.ToArray();

            this.BundlingSettings = bundlingSettings;
            this.geometryGraph = graph;
            this.LoosePadding = loosePadding;
      this.tightPadding = tightPadding;
      IEnumerable<Shape> obstacles = ShapeCreator.GetShapes(this.geometryGraph);
            this.Initialize(obstacles, coneAngle);
    }

        private readonly Edge[] _edges;

    internal Action<Edge> ReplaceEdgeByRails;

    /// <summary>
    /// 
    /// </summary>
    /// <param name="graph"></param>
    /// <param name="tightPadding"></param>
    /// <param name="loosePadding"></param>
    /// <param name="coneAngle"></param>
    /// <param name="inParentEdges"></param>
    /// <param name="outParentEdges"></param>
    public SplineRouter(GeometryGraph graph, double tightPadding, double loosePadding, double coneAngle, List<Edge> inParentEdges, List<Edge> outParentEdges) {
#if TEST_MSAGL
      graph.CheckClusterConsistency();
#endif
            this.geometryGraph = graph;
            this.LoosePadding = loosePadding;
      this.tightPadding = tightPadding;
      IEnumerable<Shape> obstacles = ShapeCreatorForRoutingToParents.GetShapes(inParentEdges, outParentEdges);
            this.Initialize(obstacles, coneAngle);
    }

        private void Initialize(IEnumerable<Shape> obstacles,
                    double coneAngleValue) {
            this.rootShapes = obstacles.Where(s => s.Parents == null || !s.Parents.Any()).ToArray();
            this.coneAngle = coneAngleValue;
      if (this.coneAngle == 0) {
                this.coneAngle = Math.PI / 6;
            }
        }

    /// <summary>
    /// Executes the algorithm.
    /// </summary>
    protected override void RunInternal() {
      if (!this.edgeGeometriesEnumeration.Any()) {
                return;
            }

            this.GetOrCreateRoot();
            this.RouteOnRoot();
            this.RemoveRoot();
      /*     var ll = new List<DebugCurve>();
           ll.AddRange(rootShapes.Select(s=>shapesToTightLooseCouples[s].TightPolyline).Select(p=>new DebugCurve(100,0.05,"black", p)));
           ll.AddRange(geometryGraph.Edges.Select(s => new DebugCurve(100, 0.05, "black", s.Curve)));               
           LayoutAlgorithmSettings.ShowDebugCurvesEnumeration (ll);
           LayoutAlgorithmSettings.ShowGraph(geometryGraph);*/
    }

        private void RouteOnRoot() {
            this.CalculatePortsToShapes();
            this.CalculatePortsToEnterableShapes();
            this.CalculateShapeToBoundaries(this.root);
      if (this.OverlapsDetected && !this.ContinueOnOverlaps) {
                return;
            }

            this.BindLooseShapes();
            this.SetLoosePolylinesForAnywherePorts();
            this.CalculateVisibilityGraph();
            this.RouteOnVisGraph();
    }


        /*
        IEnumerable<Polyline> AllPolysDeb() {
            foreach (var rootShape in looseRoot.Children) {
                foreach (var poly in PolysDebugUnderShape(rootShape)) {
                    yield return poly;
                }
            }
        }

        IEnumerable<Polyline> PolysDebugUnderShape(Shape shape) {
            yield return (Polyline)shape.BoundaryCurve;
            foreach (var child in shape.Children) {
                foreach (var poly in PolysDebugUnderShape(child)) {
                    yield return poly;
                }
            }
        }
        */
        private void CalculatePortsToEnterableShapes() {
            this.portsToEnterableShapes = new Dictionary<Port, Set<Shape>>();
      foreach (var portsToShape in this.portsToShapes) {
        var port = portsToShape.Key;
        var set = new Set<Shape>();
        if (!(EdgesAttachedToPortAvoidTheNode(port))) {
                    set.Insert(portsToShape.Value);
                }

                this.portsToEnterableShapes[port] = set;
      }

      foreach (var rootShape in this.rootShapes) {
        foreach (var sh in rootShape.Descendants) {
                    foreach (var port in sh.Ports) {
            var enterableSet = this.portsToEnterableShapes[port];
            enterableSet.InsertRange(sh.Ancestors.Where(s => s.BoundaryCurve != null));
          }
                }
            }
    }

        private static bool EdgesAttachedToPortAvoidTheNode(Port port) {
      return port is CurvePort || port is ClusterBoundaryPort;
    }

        private void SetLoosePolylinesForAnywherePorts() {
      foreach (var shapesToTightLooseCouple in this.shapesToTightLooseCouples) {
        var shape = shapesToTightLooseCouple.Key;
        foreach (var port in shape.Ports) {
          var aport = port as HookUpAnywhereFromInsidePort;
          if (aport != null) {
                        aport.LoosePolyline = (Polyline)shapesToTightLooseCouple.Value.LooseShape.BoundaryCurve;
                    }

                    var clusterBoundaryPort = port as ClusterBoundaryPort;
          if (clusterBoundaryPort != null) {
                        clusterBoundaryPort.LoosePolyline = (Polyline)shapesToTightLooseCouple.Value.LooseShape.BoundaryCurve;
                    }
                }
      }
    }

        private void BindLooseShapes() {
            this.looseRoot = new Shape();
#if TEST_MSAGL
            this.looseRoot.UserData = (string)this.root.UserData + "x";
#endif
      foreach (var shape in this.root.Children) {
        var looseShape = this.shapesToTightLooseCouples[shape].LooseShape;
                this.BindLooseShapesUnderShape(shape);
                this.looseRoot.AddChild(looseShape);
      }
    }

        private void BindLooseShapesUnderShape(Shape shape) {
      var loose = this.shapesToTightLooseCouples[shape].LooseShape;
      foreach (var child in shape.Children) {
        var childLooseShape = this.shapesToTightLooseCouples[child].LooseShape;
        loose.AddChild(childLooseShape);
                this.BindLooseShapesUnderShape(child);
      }
    }

        private void CalculateShapeToBoundaries(Shape shape) {
            this.ProgressStep();
      if (!shape.Children.Any()) {
                return;
            }

            foreach (Shape child in shape.Children) {
                this.CalculateShapeToBoundaries(child);
            }

            var obstacleCalculator = new ShapeObstacleCalculator(shape, this.tightPadding, this.AdjustedLoosePadding,
                                                           this.shapesToTightLooseCouples);
      obstacleCalculator.Calculate();

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=370
            //SharpKit/Colin - not supported directly!
            OverlapsDetected = OverlapsDetected | obstacleCalculator.OverlapsDetected;
#else
            this.OverlapsDetected |= obstacleCalculator.OverlapsDetected;
#endif
    }
    /// <summary>
    /// set to true if and only if there are overlaps in tight obstacles
    /// </summary>
    public bool OverlapsDetected { get; set; }

    internal double AdjustedLoosePadding {
      get { return this.BundlingSettings == null ? this.LoosePadding : this.LoosePadding * BundleRouter.SuperLoosePaddingCoefficient; }
    }

        private void RouteOnVisGraph() {
            this.ancestorSets = GetAncestorSetsMap(this.root.Descendants);
      if (this.BundlingSettings == null) {
        foreach (var edgeGroup in this._edges.GroupBy(this.EdgePassport)) {
          var passport = edgeGroup.Key;
          Set<Shape> obstacleShapes = this.GetObstaclesFromPassport(passport);
          var interactiveEdgeRouter = this.CreateInteractiveEdgeRouter(obstacleShapes);
                    this.RouteEdgesWithTheSamePassport(edgeGroup, interactiveEdgeRouter, obstacleShapes);
        }
      }
      else {
                this.RouteBundles();
            }
        }

        private void RouteEdgesWithTheSamePassport(IGrouping<Set<Shape>, Edge> edgeGeometryGroup, InteractiveEdgeRouter interactiveEdgeRouter, Set<Shape> obstacleShapes) {
      List<Edge> regularEdges;
      List<Edge[]> multiEdges;

      if (this.RouteMultiEdgesAsBundles) {
                this.SplitOnRegularAndMultiedges(edgeGeometryGroup, out regularEdges, out multiEdges);
        foreach (var edge in regularEdges) {
                    this.RouteEdge(interactiveEdgeRouter, edge);
                }

                if (multiEdges != null) {
                    this.ScaleDownLooseHierarchy(interactiveEdgeRouter, obstacleShapes);
                    this.RouteMultiEdges(multiEdges, interactiveEdgeRouter, edgeGeometryGroup.Key);
        }
      }
      else {
                foreach (var eg in edgeGeometryGroup) {
                    this.RouteEdge(interactiveEdgeRouter, eg);
                }
            }
        }

    /// <summary>
    /// if set to true routes multi edges as ordered bundles
    /// </summary>
    public bool RouteMultiEdgesAsBundles {
      get { return this.routeMultiEdgesAsBundles; }
      set { this.routeMultiEdgesAsBundles = value; }
    }

        private void RouteEdge(InteractiveEdgeRouter interactiveEdgeRouter, Edge edge) {
      var transparentShapes = this.MakeTransparentShapesOfEdgeGeometryAndGetTheShapes(edge.EdgeGeometry);
            this.ProgressStep();
            this.RouteEdgeGeometry(edge, interactiveEdgeRouter);
      SetTransparency(transparentShapes, false);
    }

        private void ScaleDownLooseHierarchy(InteractiveEdgeRouter interactiveEdgeRouter, Set<Shape> obstacleShapes) {
      var loosePolys = new List<Polyline>();
      foreach (var obstacleShape in obstacleShapes) {
        var tl = this.shapesToTightLooseCouples[obstacleShape];
        loosePolys.Add(InteractiveObstacleCalculator.LoosePolylineWithFewCorners(tl.TightPolyline, tl.Distance / BundleRouter.SuperLoosePaddingCoefficient));
      }

      interactiveEdgeRouter.LooseHierarchy = CreateLooseObstacleHierarachy(loosePolys);

      interactiveEdgeRouter.ClearActivePolygons();

      interactiveEdgeRouter.AddActivePolygons(loosePolys.Select(polyline => new Polygon(polyline)));
    }

        private void RouteMultiEdges(List<Edge[]> multiEdges, InteractiveEdgeRouter interactiveEdgeRouter, Set<Shape> parents) {
      var mer = new MultiEdgeRouter(multiEdges, interactiveEdgeRouter, parents.SelectMany(p => p.Children).Select(s => s.BoundaryCurve),
           new BundlingSettings { InkImportance = 0.00001, EdgeSeparation = this.MultiEdgesSeparation }, this.MakeTransparentShapesOfEdgeGeometryAndGetTheShapes);
      //giving more importance to ink might produce weird routings with huge detours, maybe 0 is the best value here
      mer.Run();

    }



        //        void ScaleLoosePolylinesOfInvolvedShapesDown(Set<Shape> parents) {        
        //            foreach (var parent in parents) {
        //                foreach (var shape in parent.Descendands) {
        //                    TightLooseCouple tl = this.shapesToTightLooseCouples[shape];
        //                    tl.LooseShape.BoundaryCurveBackup = tl.LooseShape.BoundaryCurve;
        //                    tl.LooseShape.BoundaryCurve = InteractiveObstacleCalculator.LoosePolylineWithFewCorners(tl.TightPolyline, tl.Distance / BundleRouter.SuperLoosePaddingCoefficient);
        //                }
        //            }
        //        }
        //
        //        void RestoreLoosePolylinesOfInvolvedShapes(Set<Shape> parents) {
        //            foreach (var parent in parents) {
        //                foreach (var shape in parent.Descendands) {
        //                    TightLooseCouple tl = shapesToTightLooseCouples[shape];
        //                    tl.LooseShape.BoundaryCurve = tl.LooseShape.BoundaryCurveBackup;
        //                }
        //            }
        //        }

        private void SplitOnRegularAndMultiedges(IEnumerable<Edge> edges, out List<Edge> regularEdges, out List<Edge[]> multiEdges) {

      regularEdges = new List<Edge>();

      var portLocationPairsToEdges = new Dictionary<PointPair, List<Edge>>();
      foreach (var eg in edges) {
        if (IsEdgeToParent(eg.EdgeGeometry)) {
                    regularEdges.Add(eg);
                } else {
                    RegisterInPortLocationsToEdges(eg, portLocationPairsToEdges);
                }
            }

      multiEdges = null;

      foreach (var edgeGroup in portLocationPairsToEdges.Values) {
        if (edgeGroup.Count == 1 || this.OverlapsDetected) {
                    regularEdges.AddRange(edgeGroup);
                } else {
          if (multiEdges == null) {
                        multiEdges = new List<Edge[]>();
                    }

                    multiEdges.Add(edgeGroup.ToArray());
        }
      }
    }

        private static void RegisterInPortLocationsToEdges(Edge eg, Dictionary<PointPair, List<Edge>> portLocationPairsToEdges) {
      List<Edge> list;
      var pp = new PointPair(eg.SourcePort.Location, eg.TargetPort.Location);
      if (!portLocationPairsToEdges.TryGetValue(pp, out list)) {
                portLocationPairsToEdges[pp] = list = new List<Edge>();
            }

            list.Add(eg);
    }

        private static bool IsEdgeToParent(EdgeGeometry e) {
      return e.SourcePort is HookUpAnywhereFromInsidePort || e.TargetPort is HookUpAnywhereFromInsidePort;
    }

        private InteractiveEdgeRouter CreateInteractiveEdgeRouter(IEnumerable<Shape> obstacleShapes) {
      //we need to create a set here because one loose polyline can hold several original shapes
      var loosePolys = new Set<Polyline>(obstacleShapes.Select(sh => this.shapesToTightLooseCouples[sh].LooseShape.BoundaryCurve as Polyline));
            var router = new InteractiveEdgeRouter {
                VisibilityGraph = this.visGraph,
                TightHierarchy =
                    this.CreateTightObstacleHierarachy(obstacleShapes),
                LooseHierarchy =
                    CreateLooseObstacleHierarachy(loosePolys),
                UseSpanner = true,
                LookForRoundedVertices = true,
                TightPadding = this.tightPadding,
                LoosePadding = this.LoosePadding,
                UseEdgeLengthMultiplier = this.UseEdgeLengthMultiplier,
                UsePolylineEndShortcutting = this.UsePolylineEndShortcutting,
                UseInnerPolylingShortcutting = this.UseInnerPolylingShortcutting,
                AllowedShootingStraightLines = this.AllowedShootingStraightLines,
            };

      router.AddActivePolygons(loosePolys.Select(polyline => new Polygon(polyline)));
      return router;
    }

        /// <summary>
        /// 
        private Set<Shape> GetObstaclesFromPassport(Set<Shape> passport) {
      if (passport.Count == 0) {
                return new Set<Shape>(this.root.Children);
            }

            var commonAncestors = this.GetCommonAncestorsAbovePassport(passport);
      var allAncestors = this.GetAllAncestors(passport);
      var ret = new Set<Shape>(passport.SelectMany(p => p.Children.Where(child => !allAncestors.Contains(child))));
      var enqueued = new Set<Shape>(passport.Concat(ret));
      var queue = new Queue<Shape>();
      foreach (var shape in passport.Where(shape => !commonAncestors.Contains(shape))) {
                queue.Enqueue(shape);
            }

            while (queue.Count > 0) {
        var a = queue.Dequeue();
        foreach (var parent in a.Parents) {
          foreach (var sibling in parent.Children) {
                        if (!allAncestors.Contains(sibling)) {
                            ret.Insert(sibling);
                        }
                    }

                    if (!commonAncestors.Contains(parent) && !enqueued.Contains(parent)) {
            queue.Enqueue(parent);
            enqueued.Insert(parent);
          }
        }
      }
      return ret;
    }

        private Set<Shape> GetAllAncestors(Set<Shape> passport) {
      if (!passport.Any()) {
                return new Set<Shape>();
            }

            var ret = new Set<Shape>(passport);
      foreach (var shape in passport) {
                ret += this.ancestorSets[shape];
            }

            return ret;
    }

        private Set<Shape> GetCommonAncestorsAbovePassport(Set<Shape> passport) {
      if (!passport.Any()) {
                return new Set<Shape>();
            }

            var ret = this.ancestorSets[passport.First()];
      foreach (var shape in passport.Skip(1)) {
                ret *= this.ancestorSets[shape];
            }

            return ret;
    }

        private void RouteBundles() {
            this.ScaleLooseShapesDown();

            this.CalculateEdgeEnterablePolylines();
      var looseHierarchy = this.GetLooseHierarchy();
      var cdt = BundleRouter.CreateConstrainedDelaunayTriangulation(looseHierarchy);
      // CdtSweeper.ShowFront(cdt.GetTriangles(), null, null,this.visGraph.Edges.Select(e=>new LineSegment(e.SourcePoint,e.TargetPoint)));

      var shortestPath = new SdShortestPath(this.MakeTransparentShapesOfEdgeGeometryAndGetTheShapes, cdt, this.FindCdtGates(cdt));
      var bundleRouter = new BundleRouter(this.geometryGraph, shortestPath, this.visGraph, this.BundlingSettings,
                                                   this.LoosePadding, this.GetTightHierarchy(),
                                                   looseHierarchy, this.enterableLoose, this.enterableTight,
                                                   port => this.LoosePolyOfOriginalShape(this.portsToShapes[port]));

      bundleRouter.Run();
    }

        private void CreateTheMapToParentLooseShapes(Shape shape, Dictionary<ICurve, Shape> loosePolylinesToLooseParentShapeMap) {
      foreach (var childShape in shape.Children) {
        var tightLooseCouple = this.shapesToTightLooseCouples[childShape];
        var poly = tightLooseCouple.LooseShape.BoundaryCurve;
        loosePolylinesToLooseParentShapeMap[poly] = shape;
                this.CreateTheMapToParentLooseShapes(childShape, loosePolylinesToLooseParentShapeMap);
      }
    }

        private Set<CdtEdge> FindCdtGates(Cdt cdt) {
      Dictionary<ICurve, Shape> loosePolylinesToLooseParentShapeMap = new Dictionary<ICurve, Shape>();

            this.CreateTheMapToParentLooseShapes(this.root, loosePolylinesToLooseParentShapeMap);
      //looking for Cdt edges connecting two siblings; only those we define as gates
      var gates = new Set<CdtEdge>();
      foreach (var cdtSite in cdt.PointsToSites.Values) {
        foreach (var cdtEdge in cdtSite.Edges) {

          if (cdtEdge.CwTriangle == null && cdtEdge.CcwTriangle == null) {
                        continue;
                    }

                    var a = (Polyline)cdtSite.Owner;
          var b = (Polyline)cdtEdge.lowerSite.Owner;
          if (a == b) {
                        continue;
                    }

                    Shape aParent;
          Shape bParent;
          if (loosePolylinesToLooseParentShapeMap.TryGetValue(a, out aParent)
              && loosePolylinesToLooseParentShapeMap.TryGetValue(b, out bParent) && aParent == bParent) {
                        gates.Insert(cdtEdge);
                    }
                }
      }
      //CdtSweeper.ShowFront(cdt.GetTriangles(), null,
      //                    gates.Select(g => new LineSegment(g.upperSite.Point, g.lowerSite.Point)), null);
      return gates;
    }

        private void CalculateEdgeEnterablePolylines() {
            this.enterableLoose = new Dictionary<EdgeGeometry, Set<Polyline>>();
            this.enterableTight = new Dictionary<EdgeGeometry, Set<Polyline>>();
      foreach (var edgeGeometry in this.edgeGeometriesEnumeration) {
        Set<Polyline> looseSet;
        Set<Polyline> tightSet;
                this.GetEdgeEnterablePolylines(edgeGeometry, out looseSet, out tightSet);
                this.enterableLoose[edgeGeometry] = looseSet;
                this.enterableTight[edgeGeometry] = tightSet;
      }
    }

        private void GetEdgeEnterablePolylines(EdgeGeometry edgeGeometry, out Set<Polyline> looseEnterable,
        out Set<Polyline> tightEnterable) {
      looseEnterable = new Set<Polyline>();
      tightEnterable = new Set<Polyline>();
      var sourceShape = this.portsToShapes[edgeGeometry.SourcePort];
      var targetShape = this.portsToShapes[edgeGeometry.TargetPort];

      if (sourceShape != this.root) {
        looseEnterable.InsertRange(this.ancestorSets[sourceShape].Select(this.LoosePolyOfOriginalShape).Where(p => p != null));
        tightEnterable.InsertRange(this.ancestorSets[sourceShape].Select(this.TightPolyOfOriginalShape).Where(p => p != null));
      }

      if (targetShape != this.root) {
        looseEnterable.InsertRange(this.ancestorSets[targetShape].Select(this.LoosePolyOfOriginalShape).Where(p => p != null));
        tightEnterable.InsertRange(this.ancestorSets[targetShape].Select(this.TightPolyOfOriginalShape).Where(p => p != null));
      }
    }

        private RectangleNode<Polyline, Point> GetTightHierarchy() {
      return RectangleNode<Polyline, Point>.CreateRectangleNodeOnEnumeration(this.shapesToTightLooseCouples.Values.
          Select(tl => new RectangleNode<Polyline, Point>(tl.TightPolyline, tl.TightPolyline.BoundingBox)));
    }

        private RectangleNode<Polyline, Point> GetLooseHierarchy() {
      var loosePolylines = new Set<Polyline>(this.shapesToTightLooseCouples.Values.Select(tl => (Polyline)(tl.LooseShape.BoundaryCurve)));
      return RectangleNode<Polyline, Point>.CreateRectangleNodeOnEnumeration(loosePolylines.Select(p => new RectangleNode<Polyline, Point>(p, p.BoundingBox)));
    }

        private void ScaleLooseShapesDown() {
      foreach (var shapesToTightLooseCouple in this.shapesToTightLooseCouples) {
        var tl = shapesToTightLooseCouple.Value;
        tl.LooseShape.BoundaryCurve = InteractiveObstacleCalculator.LoosePolylineWithFewCorners(tl.TightPolyline, tl.Distance / BundleRouter.SuperLoosePaddingCoefficient);
      }
    }

        /// <summary>
        ///  The set of shapes where the edgeGeometry source and target ports shapes are citizens.
        ///  In the simple case it is the union of the target port shape parents and the sourceport shape parents.
        ///  When one end shape contains another, the passport is the set consisting of the end shape and all other shape parents.
        /// </summary>
        /// <param name="edge"></param>
        /// <returns></returns>
        private Set<Shape> EdgePassport(Edge edge) {
      EdgeGeometry edgeGeometry = edge.EdgeGeometry;
      var ret = new Set<Shape>();
      var sourceShape = this.portsToShapes[edgeGeometry.SourcePort];
      var targetShape = this.portsToShapes[edgeGeometry.TargetPort];

      if (this.IsAncestor(sourceShape, targetShape)) {
        ret.InsertRange(targetShape.Parents);
        ret.Insert(sourceShape);
        return ret;
      }
      if (this.IsAncestor(targetShape, sourceShape)) {
        ret.InsertRange(sourceShape.Parents);
        ret.Insert(targetShape);
        return ret;
      }

      if (sourceShape != this.looseRoot) {
                ret.InsertRange(sourceShape.Parents);
            }

            if (targetShape != this.looseRoot) {
                ret.InsertRange(targetShape.Parents);
            }

            return ret;
    }

        private IEnumerable<Port> AllPorts() {
      foreach (var edgeGeometry in this.edgeGeometriesEnumeration) {
        yield return edgeGeometry.SourcePort;
        yield return edgeGeometry.TargetPort;
      }
    }

        private void CalculatePortsToShapes() {
            this.portsToShapes = new Dictionary<Port, Shape>();
      foreach (var shape in this.root.Descendants) {
                foreach (var port in shape.Ports) {
                    this.portsToShapes[port] = shape;
                }
            }
            //assign all orphan ports to the root 
            foreach (var port in this.AllPorts().Where(p => !this.portsToShapes.ContainsKey(p))) {
                this.root.Ports.Insert(port);
                this.portsToShapes[port] = this.root;
      }
    }

        private void RouteEdgeGeometry(Edge edge, InteractiveEdgeRouter iRouter) {
      var edgeGeometry = edge.EdgeGeometry;

      var addedEdges = new List<VisibilityEdge>();
      if (!(edgeGeometry.SourcePort is HookUpAnywhereFromInsidePort)) {
                addedEdges.AddRange(this.AddVisibilityEdgesFromPort(edgeGeometry.SourcePort));
            }

            if (!(edgeGeometry.TargetPort is HookUpAnywhereFromInsidePort)) {
                addedEdges.AddRange(this.AddVisibilityEdgesFromPort(edgeGeometry.TargetPort));
            }

            SmoothedPolyline smoothedPolyline;
      if (!ApproximateComparer.Close(edgeGeometry.SourcePort.Location, edgeGeometry.TargetPort.Location)) {
                edgeGeometry.Curve = iRouter.RouteSplineFromPortToPortWhenTheWholeGraphIsReady(
        edgeGeometry.SourcePort, edgeGeometry.TargetPort, true, out smoothedPolyline);
            } else {
        edgeGeometry.Curve = Edge.RouteSelfEdge(edgeGeometry.SourcePort.Curve, Math.Max(this.LoosePadding * 2, edgeGeometry.GetMaxArrowheadLength()), out smoothedPolyline);
      }
      edgeGeometry.SmoothedPolyline = smoothedPolyline;

      if (edgeGeometry.Curve == null) {
                throw new NotImplementedException();
            }

            foreach (var visibilityEdge in addedEdges) {
                VisibilityGraph.RemoveEdge(visibilityEdge);
            }

            Arrowheads.TrimSplineAndCalculateArrowheads(edgeGeometry, edgeGeometry.SourcePort.Curve,
                                                  edgeGeometry.TargetPort.Curve, edgeGeometry.Curve,
                                                  false);
      if (this.ReplaceEdgeByRails != null) {
                this.ReplaceEdgeByRails(edge);
            }
            //  SetTransparency(transparentShapes, false);
        }
    /// <summary>
    /// if set to true the original spline is kept under the corresponding EdgeGeometry
    /// </summary>
    public bool KeepOriginalSpline { get; set; }

    /// <summary>
    /// 
    /// </summary>
    public double ArrowHeadRatio { get; set; }

    internal Point[] LineSweeperPorts { get; set; }

        /// <summary>
        /// 
        /// </summary>


        private IEnumerable<VisibilityEdge> AddVisibilityEdgesFromPort(Port port) {

      Shape portShape;
      TightLooseCouple boundaryCouple;
      if (port is CurvePort || !this.portsToShapes.TryGetValue(port, out portShape) ||
          !this.shapesToTightLooseCouples.TryGetValue(portShape, out boundaryCouple)) {
                return new VisibilityEdge[] { };
            }

            var portLoosePoly = boundaryCouple.LooseShape;
      return (from point in portLoosePoly.BoundaryCurve as Polyline
              where this.visGraph.FindEdge(port.Location, point) == null
              select this.visGraph.AddEdge(port.Location, point));
    }

        private List<Shape> MakeTransparentShapesOfEdgeGeometryAndGetTheShapes(EdgeGeometry edgeGeometry) {
      //it is OK here to repeat a shape in the returned list
      Shape sourceShape = this.portsToShapes[edgeGeometry.SourcePort];
      Shape targetShape = this.portsToShapes[edgeGeometry.TargetPort];

      var transparentLooseShapes = new List<Shape>();

      foreach (var shape in this.GetTransparentShapes(edgeGeometry.SourcePort, edgeGeometry.TargetPort, sourceShape, targetShape)) {
                if (shape != null) {
                    transparentLooseShapes.Add(this.LooseShapeOfOriginalShape(shape));
                }
            }

            foreach (var shape in this.portsToEnterableShapes[edgeGeometry.SourcePort]) {
                transparentLooseShapes.Add(this.LooseShapeOfOriginalShape(shape));
            }

            foreach (var shape in this.portsToEnterableShapes[edgeGeometry.TargetPort]) {
                transparentLooseShapes.Add(this.LooseShapeOfOriginalShape(shape));
            }

            SetTransparency(transparentLooseShapes, true);
      return transparentLooseShapes;
    }

        private Shape LooseShapeOfOriginalShape(Shape s) {
      if (s == this.root) {
                return this.looseRoot;
            }

            return this.shapesToTightLooseCouples[s].LooseShape;
    }

        private Polyline LoosePolyOfOriginalShape(Shape s) {
      return (Polyline)(this.LooseShapeOfOriginalShape(s).BoundaryCurve);
    }

        private Polyline TightPolyOfOriginalShape(Shape s) {
      if (s == this.root) {
                return null;
            }

            return this.shapesToTightLooseCouples[s].TightPolyline;
    }
    #region debugging

#if TEST_MSAGL
    [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
    static internal void AnotherShowMethod(VisibilityGraph visGraph,
        Port sourcePort, Port targetPort, IEnumerable<Shape> obstacleShapes, ICurve curve) {
      var dd = new List<DebugCurve>(
   visGraph.Edges.Select(
      e =>
      new DebugCurve(100, 0.1, GetEdgeColor(e, sourcePort, targetPort),
                     new LineSegment(e.SourcePoint, e.TargetPoint))));
      if (obstacleShapes != null) {
                dd.AddRange(
            obstacleShapes.Select(s => new DebugCurve(1, s.BoundaryCurve)));
            }

            if (sourcePort != null && targetPort != null) {
                dd.AddRange(new[] {
                                    new DebugCurve(CurveFactory.CreateDiamond(3, 3, sourcePort.Location)),
                                    new DebugCurve(CurveFactory.CreateEllipse(3, 3, targetPort.Location)),
                                });
            }

            if (curve != null) {
                dd.Add(new DebugCurve(5, "purple", curve));
            }

            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(dd);
    }

        private static string GetEdgeColor(VisibilityEdge e, Port sourcePort, Port targetPort) {
      if (sourcePort == null || targetPort == null) {
                return "green";
            }

            if (ApproximateComparer.Close(e.SourcePoint, sourcePort.Location) ||
          ApproximateComparer.Close(e.SourcePoint, targetPort.Location)
          || ApproximateComparer.Close(e.TargetPoint, sourcePort.Location) ||
          ApproximateComparer.Close(e.TargetPoint, targetPort.Location)) {
                return "lightgreen";
            }

            return e.IsPassable == null || e.IsPassable() ? "green" : "red";
    }
#endif

        #endregion

        private IEnumerable<Shape> GetTransparentShapes(
        Port sourcePort,
        Port targetPort, Shape sourceShape, Shape targetShape) {
      foreach (var s in this.ancestorSets[sourceShape]) {
                yield return s;
            }

            foreach (var s in this.ancestorSets[targetShape]) {
                yield return s;
            }

            var routingOutsideOfSourceBoundary = EdgesAttachedToPortAvoidTheNode(sourcePort);
      var routingOutsideOfTargetBoundary = EdgesAttachedToPortAvoidTheNode(targetPort);
      if (!routingOutsideOfSourceBoundary && !routingOutsideOfTargetBoundary) {
        yield return sourceShape;
        yield return targetShape;
      }
      else if (routingOutsideOfSourceBoundary) {
        if (this.IsAncestor(sourceShape, targetShape)) {
                    yield return sourceShape;
                }
            }
      else {
        if (this.IsAncestor(targetShape, sourceShape)) {
                    yield return targetShape;
                }
            }
    }

        private static void SetTransparency(IEnumerable<Shape> shapes, bool v) {
      foreach (Shape shape in shapes) {
                shape.IsTransparent = v;
            }
        }


        /*
        /// <summary>
        /// it is the set of parents with all children being obstacle candidates
        /// </summary>
        /// <param name="port"></param>
        /// <param name="shape"></param>
        /// <param name="otherShape"></param>
        /// <returns></returns>
        IEnumerable<Shape> FindMinimalParents(Port port, Shape shape, Shape otherShape) {
            if (shape == null)
                yield return rootOfLooseShapes;
            else if (port is CurvePort) {
                if (IsLgAncestor(shape, otherShape))
                    yield return shape;
                else
                    foreach (Shape parent in shape.Parents)
                        yield return parent;
            } else if (PortIsInsideOfShape(port, shape) && shape.Children.Any())
                yield return shape;
            else
                foreach (Shape parent in shape.Parents)
                    yield return parent;
        }
    */

        private bool IsAncestor(Shape possibleAncestor, Shape possiblePredecessor) {
      Set<Shape> ancestors;
      return possiblePredecessor != null &&
             this.ancestorSets.TryGetValue(possiblePredecessor, out ancestors) && ancestors != null &&
             ancestors.Contains(possibleAncestor);
    }

        /*
                static bool PortIsInsideOfShape(Port port, Shape shape) {
                    if (shape == null)
                        return false;
                    return Curve.PointRelativeToCurveLocation(ApproximateComparer.Round(port.Location), shape.BoundaryCurve) ==
                           PointLocation.Inside;
                }
        */

        private static RectangleNode<Polyline, Point> CreateLooseObstacleHierarachy(IEnumerable<Polyline> loosePolys) {
      return
          RectangleNode<Polyline, Point>.CreateRectangleNodeOnEnumeration(
              loosePolys.Select(
                  poly => new RectangleNode<Polyline, Point>(poly, poly.BoundingBox)));
    }

        private RectangleNode<Polyline, Point> CreateTightObstacleHierarachy(IEnumerable<Shape> obstacles) {
      var tightPolys = obstacles.Select(sh => this.shapesToTightLooseCouples[sh].TightPolyline);
      return
          RectangleNode<Polyline, Point>.CreateRectangleNodeOnEnumeration(
              tightPolys.Select(tightPoly => new RectangleNode<Polyline, Point>(
                                                 tightPoly,
                                                 tightPoly.BoundingBox)));
    }

        private void CalculateVisibilityGraph() {
      var setOfPortLocations = this.LineSweeperPorts != null ? new Set<Point>(this.LineSweeperPorts) : new Set<Point>();
            this.ProcessHookAnyWherePorts(setOfPortLocations);
            this.portRTree =
          new RTree<Point,Point>(
              setOfPortLocations.Select(p => new KeyValuePair<IRectangle<Point>, Point>(new Rectangle(p), p)));
            this.visGraph = new VisibilityGraph();

            this.FillVisibilityGraphUnderShape(this.root);
                  //ShowVisGraph(visGraph, new Set<Polyline>(shapesToTightLooseCouples.Values.Select(tl => (Polyline)(tl.LooseShape.BoundaryCurve))),
                    //  geometryGraph.Nodes.Select(n => n.BoundaryCurve).Concat(root.Descendants.Select(d => d.BoundaryCurve)), null);
    }

    private void ProcessHookAnyWherePorts(Set<Point> setOfPortLocations) {
      foreach (var edgeGeometry in this.edgeGeometriesEnumeration) {
        if (!(edgeGeometry.SourcePort is HookUpAnywhereFromInsidePort || edgeGeometry.SourcePort is ClusterBoundaryPort)) {
                    setOfPortLocations.Insert(edgeGeometry.SourcePort.Location);
                }

                if (!(edgeGeometry.TargetPort is HookUpAnywhereFromInsidePort || edgeGeometry.TargetPort is ClusterBoundaryPort)) {
                    setOfPortLocations.Insert(edgeGeometry.TargetPort.Location);
                }
            }
    }

        /// <summary>
        /// this function might change the shape's loose polylines by inserting new points
        /// </summary>
        private void FillVisibilityGraphUnderShape(Shape shape) {
      //going depth first 
      var children = shape.Children;
      foreach (Shape child in children) {
                this.FillVisibilityGraphUnderShape(child);
            }

            TightLooseCouple tightLooseCouple;
      Polyline looseBoundary = this.shapesToTightLooseCouples.TryGetValue(shape, out tightLooseCouple) ? tightLooseCouple.LooseShape.BoundaryCurve as Polyline : null;
      Shape looseShape = tightLooseCouple != null ? tightLooseCouple.LooseShape : this.looseRoot;
      var obstacles = new Set<Polyline>(looseShape.Children.Select(c => c.BoundaryCurve as Polyline));

      var portLocations = this.RemoveInsidePortsAndSplitBoundaryIfNeeded(looseBoundary);
      //this run will split the polyline enough to route later from the inner ports
      var tmpVisGraph = new VisibilityGraph();
      var coneSpanner = new ConeSpanner(new Polyline[] { }, tmpVisGraph, this.coneAngle, portLocations, looseBoundary);
      coneSpanner.Run();
      //now run the spanner again to create the correct visibility graph around the inner obstacles
      tmpVisGraph = new VisibilityGraph();
      coneSpanner = new ConeSpanner(obstacles, tmpVisGraph, this.coneAngle, portLocations, looseBoundary) {
        Bidirectional = this.Bidirectional && obstacles.Count > 0
      };
      coneSpanner.Run();

            this.ProgressStep();

      foreach (VisibilityEdge edge in tmpVisGraph.Edges) {
                this.TryToCreateNewEdgeAndSetIsPassable(edge, looseShape);
            }

            this.AddBoundaryEdgesToVisGraph(looseBoundary);
      //            if (obstacles.Count > 0)
      //                SplineRouter.ShowVisGraph(tmpVisGraph, obstacles, null, null);
    }

    /// <summary>
    /// If set to true then a smaller visibility graph is created.
    /// An edge is added to the visibility graph only if it is found at least twice: 
    /// once sweeping with a direction d and the second time with -d
    /// </summary>
    internal bool Bidirectional { get; set; }

#if TEST_MSAGL
    [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        static internal void ShowVisGraph(VisibilityGraph tmpVisGraph, IEnumerable<Polyline> obstacles, IEnumerable<ICurve> greenCurves = null, IEnumerable<ICurve> redCurves = null) {
          var l = new List<DebugCurve>(tmpVisGraph.Edges.Select(e => new DebugCurve(100, 1,
              e.IsPassable != null && e.IsPassable() ? "green" : "black"
              , new LineSegment(e.SourcePoint, e.TargetPoint))));
          if (obstacles != null) {
                l.AddRange(obstacles.Select(p => new DebugCurve(100, 1, "brown", p)));
            }

            if (greenCurves != null) {
                l.AddRange(greenCurves.Select(p => new DebugCurve(100, 10, "navy", p)));
            }

            if (redCurves != null) {
                l.AddRange(redCurves.Select(p => new DebugCurve(100, 10, "red", p)));
            }

            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
        }
#endif
        private void TryToCreateNewEdgeAndSetIsPassable(VisibilityEdge edge, Shape looseShape) {
      var e = this.visGraph.FindEdge(edge.SourcePoint, edge.TargetPoint);
      if (e != null) {
                return;
            }

            e = this.visGraph.AddEdge(edge.SourcePoint, edge.TargetPoint);
      if (looseShape != null) {
                e.IsPassable = () => looseShape.IsTransparent;
            }
        }

        private void AddBoundaryEdgesToVisGraph(Polyline boundary) {
      if (boundary == null) {
                return;
            }

            var p = boundary.StartPoint;
      do {
        var pn = p.NextOnPolyline;
                this.visGraph.AddEdge(p.Point, pn.Point);
        if (pn == boundary.StartPoint) {
                    break;
                }

                p = pn;
      } while (true);
    }

        private Set<Point> RemoveInsidePortsAndSplitBoundaryIfNeeded(Polyline boundary) {
      var ret = new Set<Point>();

      if (boundary == null) {
        foreach (var point in this.portRTree.GetAllLeaves()) {
                    ret.Insert(point);
                }

                this.portRTree.Clear();
        return ret;
      }
      Rectangle boundaryBox = boundary.BoundingBox;
      var portLocationsInQuestion = this.portRTree.GetAllIntersecting(boundaryBox).ToArray();
      foreach (var point in portLocationsInQuestion) {
        switch (Curve.PointRelativeToCurveLocation(point, boundary)) {
          case PointLocation.Inside:
            ret.Insert(point);
                        this.portLocationsToLoosePolylines[point] = boundary;
                        this.portRTree.Remove(new Rectangle(point), point);
            break;
          case PointLocation.Boundary:
                        this.portRTree.Remove(new Rectangle(point), point);
                        this.portLocationsToLoosePolylines[point] = boundary;
            PolylinePoint polylinePoint = FindPointOnPolylineToInsertAfter(boundary, point);
            if (polylinePoint != null) {
                            LineSweeper.InsertPointIntoPolylineAfter(boundary, polylinePoint, point);
                        } else {
                            throw new InvalidOperationException();
                        }

                        break;
        }
      }
      return ret;
    }

        private static PolylinePoint FindPointOnPolylineToInsertAfter(Polyline boundary, Point point) {
      for (PolylinePoint p = boundary.StartPoint; ;) {
        PolylinePoint pn = p.NextOnPolyline;

        if (ApproximateComparer.Close(point, p.Point) || ApproximateComparer.Close(point, pn.Point)) {
                    return null; //the point is already inside
                }

                double par;
        if (ApproximateComparer.Close(Point.DistToLineSegment(point, p.Point, pn.Point, out par), 0)) {
                    return p;
                }

                p = pn;
        if (p == boundary.StartPoint) {
                    throw new InvalidOperationException();
                }
            }
    }


        /// <summary>
        /// creates a root; a shape with BoundaryCurve set to null 
        /// </summary>
        private void GetOrCreateRoot() {
      if (this.rootShapes.Count() == 0) {
                return;
            }

            if (this.rootShapes.Count() == 1) {
        Shape r = this.rootShapes.First();
        if (r.BoundaryCurve == null) {
                    this.root = r;
          return;
        }
      }
            this.rootWasCreated = true;
            this.root = new Shape();
#if TEST_MSAGL
            this.root.UserData = "root";
#endif
      foreach (var rootShape in this.rootShapes) {
                this.root.AddChild(rootShape);
            }
        }

        private void RemoveRoot() {
      if (this.rootWasCreated) {
                foreach (var rootShape in this.rootShapes) {
                    rootShape.RemoveParent(this.root);
                }
            }
        }


#if TEST_MSAGL
    // ReSharper disable UnusedMember.Local
    [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        private static void Show(
        IEnumerable<EdgeGeometry> edgeGeometries, IEnumerable<Shape> listOfShapes) {
      // ReSharper restore UnusedMember.Local
      var r = new Random(1);
      LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(
          listOfShapes.Select(s => s.BoundaryCurve).Select(
              c => new DebugCurve(50, 1, DebugCurve.Colors[r.Next(DebugCurve.Colors.Length - 1)], c)).Concat(
                  edgeGeometries.Select(e => new DebugCurve(100, 1, "red", e.Curve))));
    }
#endif


    internal static Dictionary<Shape, Set<Shape>> GetAncestorSetsMap(IEnumerable<Shape> shapes) {
      var ancSets = new Dictionary<Shape, Set<Shape>>();
      foreach (var child in shapes.Where(child => !ancSets.ContainsKey(child))) {
                ancSets[child] = GetAncestorSet(child, ancSets);
            }

            return ancSets;
    }

        private static Set<Shape> GetAncestorSet(Shape child, Dictionary<Shape, Set<Shape>> ancSets) {
      var ret = new Set<Shape>(child.Parents);
      foreach (var parent in child.Parents) {
        Set<Shape> grandParents;
        ret += ancSets.TryGetValue(parent, out grandParents)
                   ? grandParents
                   : ancSets[parent] = GetAncestorSet(parent, ancSets);
      }
      return ret;
    }

    static internal void CreatePortsIfNeeded(IEnumerable<Edge> edges) {
      foreach (var edge in edges) {
        if (edge.SourcePort == null) {
          var e = edge;
#if SHARPKIT // Lambdas bind differently in JS
                    edge.SourcePort = ((Func<Edge,RelativeFloatingPort>)(ed => new RelativeFloatingPort(() => ed.Source.BoundaryCurve,
                        () => ed.Source.Center)))(e);
#else
          edge.SourcePort = new RelativeFloatingPort(() => e.Source.BoundaryCurve, () => e.Source.Center);
#endif
        }
        if (edge.TargetPort == null) {
          var e = edge;
#if SHARPKIT // Lambdas bind differently in JS
                    edge.TargetPort = ((Func<Edge, RelativeFloatingPort>)(ed => new RelativeFloatingPort(() => ed.Target.BoundaryCurve,
                        () => ed.Target.Center)))(e);
#else
          edge.TargetPort = new RelativeFloatingPort(() => e.Target.BoundaryCurve, () => e.Target.Center);
#endif
        }
      }
    }

    /// <summary>
    ///  computes loosePadding for spline routing obstacles from node separation and EdgePadding.
    /// </summary>
    /// <param name="nodeSeparation"></param>
    /// <param name="edgePadding"></param>
    /// <returns></returns>
    static public double ComputeLooseSplinePadding(double nodeSeparation, double edgePadding) {
      Debug.Assert(edgePadding > 0, "require EdgePadding > 0");
      double twicePadding = 2.0 * edgePadding;
      Debug.Assert(nodeSeparation > twicePadding, "require OverlapSeparation > 2*EdgePadding");

      // the 8 divisor is just to guarantee the final postcondition
      double loosePadding = (nodeSeparation - twicePadding) / 8;
      Debug.Assert(loosePadding > 0, "require LoosePadding > 0");
      Debug.Assert(twicePadding + (2 * loosePadding) < nodeSeparation, "EdgePadding too big!");
      return loosePadding;
    }


  }
}
