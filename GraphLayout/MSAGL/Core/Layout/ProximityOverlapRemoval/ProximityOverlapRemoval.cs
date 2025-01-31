﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.StressEnergy;
#if TEST_MSAGL
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.DebugHelpers;
#endif
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Point = Microsoft.Msagl.Core.Geometry.Point;
using Rectangle = Microsoft.Msagl.Core.Geometry.Rectangle;

namespace Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval {
    /// <summary>
    ///  Proximity Stress Model as suggested by Gansner et. al, Fast Node Overlap Removal.
    /// </summary>
    public class ProximityOverlapRemoval :IOverlapRemoval {
        private Node[] _nodes;
#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        public static bool DebugMode { get; set; }

        private List<Polyline> trajectories;

        /// <summary>
        /// 
        /// </summary>
        public List<int> crossingsOverTime = new List<int>();
#endif

        private OverlapRemovalSettings settings;

        /// <summary>
        ///     Overlap Removal Parameters
        /// </summary>
        public OverlapRemovalSettings Settings {
            get { return this.settings; }
            set {
                this.settings = value;
                if (this.StressSolver != null && value != null) {
                    this.StressSolver.Settings = value.StressSettings;
                }
            }
        }

        /// <summary>
        /// Stores the needed number of iterations for the last run.
        /// </summary>
        public int LastRunIterations { get; set; }

        private TimeSpan lastCpuTime;

        /// <summary>
        /// Bounding boxes of nodes.
        /// </summary>
        private Size[] nodeSizes;

        /// <summary>
        /// Node positions.
        /// </summary>
        private Point[] nodePositions;
        
        /// <summary>
        /// Current Node Boxes
        /// </summary>
        public Size[] NodeSizes {
            get { return this.nodeSizes; }
        }

        private StressMajorization stressSolver;

        /// <summary>
        ///     Solver for Stress Majorization
        /// </summary>
        public StressMajorization StressSolver {
            get { return this.stressSolver; }
            set {
                this.stressSolver = value;
                if (this.Settings != null && this.stressSolver != null) {
                    this.stressSolver.Settings = this.Settings.StressSettings;
                }
            }
        }

         
        /// <summary>
        ///     Graph for Overlap Removal.
        /// </summary>
        public GeometryGraph Graph {
            set {
                this._nodes = value.Nodes.ToArray();
                this.InitWithGraph();
            }
        }

        private void InitializeSettings() {
            this.Settings = new OverlapRemovalSettings();
        }


        /// <summary>
        /// Constructor
        /// </summary>
        public ProximityOverlapRemoval(OverlapRemovalSettings settings, GeometryGraph graph) {
            this.Graph = graph;
            this.Settings = settings;
        }

        /// <summary>
        ///     Constructor
        /// </summary>
        public ProximityOverlapRemoval() {
            this.InitializeSettings();
        }

        /// <summary>
        ///     Constructor
        /// </summary>
        /// <param name="graph"></param>
        public ProximityOverlapRemoval(GeometryGraph graph) {
            this.Graph = graph;
            this.InitializeSettings();
        }

        /// <summary>
        ///     Inits the datastructures, later forces can be defined on the nodes.
        /// </summary>
        /// <param name="majorizer"></param>
        /// <param name="nodes"></param>
        /// <param name="nodePositions"></param>
        private static void InitStressWithGraph(StressMajorization majorizer, Node[] nodes, Point[] nodePositions) {
            majorizer.Positions = new List<Point>(nodePositions);
            majorizer.NodeVotings = new List<NodeVoting>(nodes.Length);

            for (int i = 0; i < nodes.Length; i++) {
                var nodeVote = new NodeVoting(i);
                //add second block for separate weighting of the overlap distances
                var voteBlock = new VoteBlock(new List<Vote>(), 100);
//                nodeVote.VotingBlocks[0].BlockWeight = 0;
                nodeVote.VotingBlocks.Add(voteBlock);
                majorizer.NodeVotings.Add(nodeVote);
            }
        }

        /// <summary>
        ///     Inits some structures.
        /// </summary>
        protected virtual void InitWithGraph() {
            if (this.StressSolver != null) {
                //possibly call destructor to free memory...
            }

            if (this._nodes == null || this._nodes.Length == 0) {
                return;
            }

            if (this.StressSolver == null) {
                this.StressSolver = new StressMajorization();
                if (this.Settings != null) {
                    this.StressSolver.Settings = this.Settings.StressSettings;
                }
            }
        }

        /// <summary>
        ///     Coincidence of points is resolved by randomly moving the second of two points, until the coincidence is resolved.
        ///     Points are also slightly randomized if randomizeAll is true, to avoid degenerate cases which will break the algorithm.
        /// </summary>
        /// <param name="points"></param>
        /// <param name="random"></param>
        /// <param name="epsilon"></param>
        /// <param name="randomizeAll"></param>
        public static void RandomizePoints(Point[] points, Random random, double epsilon, bool randomizeAll) {
            var pointSet = new HashSet<Point>();
            for (int i = 0; i < points.Length; i++) {
                Point p = points[i];
                if (pointSet.Contains(p) || randomizeAll) {
                    do {
                        double newX = p.X + (2*random.NextDouble() - 1)*epsilon;
                        double newY = p.Y + (2*random.NextDouble() - 1) + epsilon;
                        p = new Point(newX, newY);
                    } while (pointSet.Contains(p));
                }
                points[i] = p;
                pointSet.Add(p);
            }
        }

        /// <summary>
        /// Determines the edges of the triangulation together with their desired length (distance between nodes).
        /// </summary>
        /// <param name="originalGraph"></param>
        /// <param name="cdt"></param>
        /// <param name="targetSizes"></param>
        /// <param name="desiredEdgeDistances"></param>
        /// <returns></returns>
        public static int GetProximityEdgesWithDistance(Node[] originalGraph, Cdt cdt,
                                                        Size[] targetSizes,
                                                        out List<Tuple<int, int, double, double>> desiredEdgeDistances) {
            desiredEdgeDistances = new List<Tuple<int, int, double, double>>();
            int numberOverlappingPairs = 0;
            var edgeSet = new HashSet<CdtEdge>();
            //add edges
            foreach (CdtTriangle triangle in cdt.GetTriangles()) {
                foreach (CdtEdge triangleEdge in triangle.Edges) {
                    CdtSite site1 = triangleEdge.upperSite;
                    CdtSite site2 = triangleEdge.lowerSite;
                    var nodeId1 = (int) site1.Owner;
                    var nodeId2 = (int) site2.Owner;
                    if (edgeSet.Contains(triangleEdge)) {
                        continue; //edge already included 
                    }

                    edgeSet.Add(triangleEdge);

                    Point point1 = site1.Point;
                    Point point2 = site2.Point;

                    double t;
                    double distance = GetIdealDistanceBetweenNodes(nodeId1, nodeId2, point1, point2, targetSizes,
                                                                   out t);
                    if (t > 1) {
                        numberOverlappingPairs++;
                    }

                    int nodeIdSmall = nodeId1;
                    int nodeIdBig = nodeId2;
                    if (nodeId1 > nodeId2) {
                        nodeIdSmall = nodeId2;
                        nodeIdBig = nodeId1;
                    }
                    var tuple = new Tuple<int, int, double, double>(nodeIdSmall, nodeIdBig, distance, t);
                    desiredEdgeDistances.Add(tuple);
                }
            }
            return numberOverlappingPairs;
        }


        /// <summary>
        /// temporary due to merge problems.
        /// </summary>
        /// <param name="nodeId1"></param>
        /// <param name="nodeId2"></param>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <param name="nodeBoxes"></param>
        /// <param name="tRes"></param>
        /// <returns></returns>
        public static double GetOverlapFactorBetweenNodes(int nodeId1, int nodeId2, Point point1, Point point2,
                                                          Size[] nodeBoxes, out double tRes) {
            return GetIdealDistanceBetweenNodes(nodeId1, nodeId2, point1, point2, nodeBoxes, out tRes);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="nodeId1"></param>
        /// <param name="nodeId2"></param>
        /// <param name="point1"></param>
        /// <param name="point2"></param>
        /// <param name="nodeBoxes"></param>
        /// <param name="tRes"></param>
        /// <returns></returns>
        public static double GetIdealDistanceBetweenNodes(int nodeId1, int nodeId2, Point point1, Point point2,
                                                          Size[] nodeBoxes, out double tRes) {
            if (nodeBoxes == null) {
                throw new ArgumentNullException("nodeBoxes");
            }

            tRes = -1;
            if (nodeBoxes.Length <= nodeId1) {
                return 0;
            }

            if (nodeBoxes.Length <= nodeId2) {
                return 0;
            }

            const double expandMax = 1.5;
            const double expandMin = 1;
//            double tmax = 0;
//            double tmin = 1E10;

            const double machineAcc = 1.0e-16;
            double dist = (point1 - point2).Length;
            double dx = Math.Abs(point1.X - point2.X);
            double dy = Math.Abs(point1.Y - point2.Y);

            double wx = (nodeBoxes[nodeId1].Width/2 + nodeBoxes[nodeId2].Width/2);
            double wy = (nodeBoxes[nodeId1].Height/2 + nodeBoxes[nodeId2].Height/2);

            double t;
            if (dx < machineAcc*wx) {
                t = wy/dy;
            }
            else if (dy < machineAcc*wy) {
                t = wx/dx;
            }
            else {
                t = Math.Min(wx/dx, wy/dy);
            }

            if (t > 1) {
                t = Math.Max(t, 1.001); // must be done, otherwise the convergence is very slow
            }
            //            tmax = Math.Max(tmax, t);
            //            tmin = Math.Min(tmin, t);
            t = Math.Min(expandMax, t);
            t = Math.Max(expandMin, t);
            tRes = t;
            return t*dist;
        }


        /// <summary>
        ///     Removes the overlap according to the defined settings.
        /// </summary>
        /// <returns></returns>
        public void RemoveOverlaps() {
            if (this._nodes == null || this._nodes.Length == 0) {
                return;
            }
            // init some things
            InitNodePositionsAndBoxes(this.Settings, this._nodes, out this.nodePositions, out this.nodeSizes);
            InitStressWithGraph(this.StressSolver, this._nodes, this.nodePositions);
#if TEST_MSAGL
            //debugging the node movements
            this.trajectories = new List<Polyline>(this._nodes.Length);
            //add starting positions
            for (int i = 0; i < this.nodePositions.Length; i++) {
                var poly = new Polyline();
                poly.AddPoint(this.nodePositions[i]);
                this.trajectories.Add(poly);
            }
#endif

#if !SHARPKIT
            var stopWatch = new Stopwatch();
            stopWatch.Start();
#endif
            bool scanlinePhase = false;
            int iter = 0;
            bool finished = false;
            while (!finished && ((iter++) < this.Settings.IterationsMax || !this.Settings.StopOnMaxIterat)) {
                finished = this.DoSingleIteration(iter, ref scanlinePhase);
            }
#if !SHARPKIT
            stopWatch.Stop();
#endif
            this.LastRunIterations = iter;
#if TEST_MSAGL && !SHARPKIT
            if (DebugMode) {
                this.ShowTrajectoriesOfNodes(this.trajectories);

                //LayoutAlgorithmSettings.ShowGraph(Graph);
            }
#endif


            this.SetPositionsToGraph();
#if !SHARPKIT
            PrintTimeSpan(stopWatch);
#endif
            double nodeBoxArea = this.nodeSizes.Sum(r => r.Width*r.Height);
            var boundingBox = this.GetCommonRectangle(this.nodeSizes, this.nodePositions);
            double boundingBoxArea = boundingBox.Width*boundingBox.Height;
//            nodePositions = null;
//            nodeBoxes = null;
#if TEST_MSAGL && !SHARPKIT
            if (DebugMode) {
                //LayoutAlgorithmSettings.ShowGraph(Graph);
            }
#endif

#if !SHARPKIT
            this.lastCpuTime = stopWatch.Elapsed;
            return;
#else
            return;// new TimeSpan(0);
#endif
        }

        private Rectangle GetCommonRectangle(Size[] sizes, Point[] points) {
            var rect = Rectangle.CreateAnEmptyBox();
            Debug.Assert(sizes.Length==points.Length);
            for (int i = 0; i < sizes.Length; i++) {
                rect.Add(sizes[i], points[i]);
            }

            return rect;
        }

        private bool DoSingleIteration(int currentIteration, ref bool scanlinePhase) {
            List<Tuple<Point, object>> sites =
                this.nodePositions.Select((p, index) => new Tuple<Point, object>(p, index)).ToList();
            var triangulation = new Cdt(sites);
            triangulation.Run();

            List<Tuple<int, int, double, double>> proximityEdgesWithDistance;
            int numCrossings = GetProximityEdgesWithDistance(this._nodes, triangulation, this.nodeSizes,
                                                             out proximityEdgesWithDistance);

            if (scanlinePhase || numCrossings == 0) {
                scanlinePhase = true;
                numCrossings = this.CompleteProximityGraphWithRTree(ref numCrossings, proximityEdgesWithDistance);
            }
#if TEST_MSAGL
            int realCrossings = this.CountCrossingsWithRTree(this.nodeSizes);
            this.crossingsOverTime.Add(realCrossings);
            if (currentIteration%10 == 0) {
                System.Diagnostics.Debug.WriteLine("Scanline: {0}, Crossings: {1}", scanlinePhase, numCrossings);
            }
#endif

            if (numCrossings == 0) {
                return true;
            }

            AddStressFromProximityEdges(this.StressSolver, proximityEdgesWithDistance);
            List<Point> newPositions = this.StressSolver.IterateAll();

            ShowCurrentMovementVectors(currentIteration, this.nodeSizes, this.nodePositions, newPositions,
                                       proximityEdgesWithDistance,
                                       null);

            this.UpdatePointsAndBoxes(newPositions);
            //clear the data structures
            this.StressSolver.ClearVotings();
#if TEST_MSAGL
            for (int i = 0; i < this.nodePositions.Length; i++) {
                this.trajectories[i].AddPoint(newPositions[i]);
            }
#endif
            return false;
        }

        private void UpdatePointsAndBoxes(List<Point> newPositions) {
             for (int i = 0; i < this.nodePositions.Length; i++) {
                this.nodePositions[i] = newPositions[i];
            }
        }

        private void SetPositionsToGraph() {
            for (int i = 0; i < this._nodes.Length; i++) {
                if (this.Settings.WorkInInches) {
                    this._nodes[i].Center = this.nodePositions[i]*72;
                } else {
                    this._nodes[i].Center = this.nodePositions[i];
                }
            }
        }


#if TEST_MSAGL
        private void ShowTrajectoriesOfNodes(List<Polyline> trajectories) {
//            if (trajectories.Count < 1 || trajectories[0].Count < 3) return;
//
//            double[] values = new double[trajectories.Count];
//            for (int i = 1; i <= trajectories.Count; i++) {
//                values[i - 1] = i;
//                    }
//            
//            var colors=ColorInterpolator.LinearInterpolation(values, Color.Tomato, Color.SeaGreen);
//            var cHex = ColorInterpolator.ColorToHex(colors);
//            var list=trajectories.Select((p, i) => new DebugCurve(220, 2,
//                                                         cHex[i], p));
//            LayoutAlgorithmSettings.ShowDebugCurves(list.ToArray());
        }
#endif

        //         double GetAverageOverlap(List<Tuple<int, int, double, double>> proximityEdgesWithDistance,
        //                                         Point[] positions, Rectangle[] rectangles) {
        //            double overlap = 0;
        //            int counter = 0;
        //            foreach (Tuple<int, int, double, double> tuple in proximityEdgesWithDistance) {
        //                int nodeId1 = tuple.Item1;
        //                int nodeId2 = tuple.Item2;
        //                Point point1 = positions[nodeId1];
        //                Point point2 = positions[nodeId2];
        //
        //                if (nodeBoxes == null) throw new ArgumentNullException("nodeBoxes");
        //                if (nodeBoxes.Length <= nodeId1) return 0;
        //                if (nodeBoxes.Length <= nodeId2) return 0;
        //                double box1Width = nodeBoxes[nodeId1].Width;
        //                double box1Height = nodeBoxes[nodeId1].Height;
        //                double box2Width = nodeBoxes[nodeId2].Width;
        //                double box2Height = nodeBoxes[nodeId2].Height;
        //
        //                //Gansner et. al Scaling factor of distance
        //                double tw = (box1Width/2 + box2Width/2)/Math.Abs(point1.X - point2.X);
        //                double th = (box1Height/2 + box2Height/2)/Math.Abs(point1.Y - point2.Y);
        //                double t = Math.Max(Math.Min(tw, th), 1);
        //
        //                if (t == 1) continue; // no overlap between the bounding boxes
        //
        //                double distance = (t - 1)*(point1 - point2).Length;
        //                overlap += distance;
        //                counter++;
        //            }
        //
        //            overlap /= counter;
        //            return overlap;
        //        }

        /// <summary>
        /// For debugging only
        /// </summary>
        /// <param name="currentIteration"></param>
        /// <param name="nodeSizes"></param>
        /// <param name="nodePositions"></param>
        /// <param name="newPositions"></param>
        /// <param name="proximityEdgesWithDistance"></param>
        /// <param name="finalGridVectors"></param>
        private static void ShowCurrentMovementVectors(int currentIteration, Size[] nodeSizes,
                                                       Point[] nodePositions, List<Point> newPositions,
                                                       List<Tuple<int, int, double, double>> proximityEdgesWithDistance,
                                                       Point[] finalGridVectors) {
#if TEST_MSAGL && !SHARPKIT
            if (DebugMode && currentIteration%1 == 0) {
                List<DebugCurve> curveList = new List<DebugCurve>();
                var nodeBoxes = new Rectangle[nodeSizes.Length];
                for(int i=0;i<nodeBoxes.Length;i++) {
                    nodeBoxes[i]=new Rectangle(nodeSizes[i], nodePositions[i]);
                }

                var nodeCurves =
                    nodeBoxes.Select(
                        v =>
                        new DebugCurve(220, 1, "black", Curve.PolylineAroundClosedCurve(CurveFactory.CreateRectangle(v))));
                curveList.AddRange(nodeCurves);
                var vectors = nodePositions.Select(
                    (p, i) =>
                    new DebugCurve(220, 2, "red", new Polyline(p, newPositions[i]))).ToList();

                foreach (Tuple<int, int, double, double> tuple in proximityEdgesWithDistance) {
                    if (tuple.Item3 > 0) {
                        curveList.Add(new DebugCurve(220, 1, "gray",
                                                     new Polyline(nodePositions[tuple.Item1],
                                                                  nodePositions[tuple.Item2])));
                    }
                }
                curveList.AddRange(vectors);
                if (finalGridVectors != null) {
                    var gridFlowVectors = nodePositions.Select((p, i) =>
                                                               new DebugCurve(220, 2, "blue",
                                                                              new Polyline(p, p + finalGridVectors[i])))
                                                       .ToList();
                    curveList.AddRange(gridFlowVectors);
                }

                LayoutAlgorithmSettings.ShowDebugCurves(curveList.ToArray());
            }
#endif
        }

        private int CompleteProximityGraphWithRTree(ref int currentCrossings,
            List<Tuple<int, int, double, double>> proximityEdgesWithDistance) {
            // if no crossings detected, use RTree to be sure there are no crossings 
            int newCrossings = this.CreateProximityEdgesWithRTree(proximityEdgesWithDistance);
            return currentCrossings + newCrossings;
        }


#if !SHARPKIT
        private static void PrintTimeSpan(Stopwatch stopWatch) {
            // Get the elapsed time as a TimeSpan value.
            TimeSpan ts = stopWatch.Elapsed;
            // Format and display the TimeSpan value.
            string elapsedTime = String.Format("{0:00}:{1:00}:{2:00}.{3:00}",
                                               ts.Hours, ts.Minutes, ts.Seconds,
                                               ts.Milliseconds/10);
            System.Diagnostics.Debug.WriteLine(elapsedTime, "RunTime");
        }

#endif

        internal static Point[] InitNodePositionsAndBoxes(OverlapRemovalSettings overlapRemovalSettings,
                                                          Node[] nodes, out Point[] nodePositions,
                                                          out Size[] nodeSizes) {
            nodePositions = nodes.Select(v => v.Center).ToArray();
            //make sure no two points are the same
            RandomizePoints(nodePositions, new Random(overlapRemovalSettings.RandomizationSeed),
                            overlapRemovalSettings.Epsilon,
                            overlapRemovalSettings.RandomizeAllPointsOnStart);
            nodeSizes = GetNodeSizesByPaddingWithHalfSeparation(nodes, overlapRemovalSettings.NodeSeparation);
            return nodePositions;
        }

        private int CreateProximityEdgesWithRTree(List<Tuple<int, int, double, double>> proximityEdges) {
            var edgeSet = new HashSet<Tuple<int, int>>();

            foreach (var proximityEdge in proximityEdges) {
                edgeSet.Add(Tuple.Create(proximityEdge.Item1, proximityEdge.Item2));
            }
            RectangleNode<int,Point> rootNode =
                RectangleNode<int,Point>.CreateRectangleNodeOnEnumeration(
                    this.nodeSizes.Select((size, index) => new RectangleNode<int,Point>(index, new Rectangle(size, this.nodePositions[index]))));
            int numCrossings = 0;
             RectangleNodeUtils.CrossRectangleNodes<int, int, Point>(rootNode, rootNode,
                 (a, b) => {
                     if (a == b) {
                         return;
                     }

                     double t;
                     double dist = GetOverlapFactorBetweenNodes
                         (
                             a, b,
                             this.nodePositions[a
                                 ],
                             this.nodePositions[b
                                 ],
                             this.nodeSizes, out t);
                     int smallId = a;
                     int bigId = b;
                     if (smallId > bigId) {
                         smallId = b;
                         bigId = a;
                     }
                     if (!(t > 1) ||
                         edgeSet.Contains(new Tuple<int, int>(smallId, bigId))) {
                         return;
                     }

                     proximityEdges.Add(Tuple.Create(smallId, bigId,
                         dist, t));
                     edgeSet.Add(new Tuple<int, int>(smallId, bigId));
                     numCrossings++;
                 });

            return numCrossings;
        }

#if TEST_MSAGL
        private int CountCrossingsWithRTree(Size[] nodeSizes) {
            RectangleNode<int,Point> rootNode =
                RectangleNode<int,Point>.CreateRectangleNodeOnEnumeration(
                    nodeSizes.Select((r, index) => new RectangleNode<int,Point>(index, new Rectangle(r, this.nodePositions[index]))));
            int numCrossings = 0;
            RectangleNodeUtils.CrossRectangleNodes<int, int, Point>(rootNode, rootNode,
                                                             (a, b) => {
                                                                 if (a == b) {
                                                                     return;
                                                                 }

                                                                 numCrossings++;
                                                             });

            return numCrossings;
        }
#endif

        private static Size[]? GetNodeSizesByPaddingWithHalfSeparation(Node[] nodes, double nodeSeparation) {
            if (nodes == null) {
                return null;
            }

            var nodeSizes = new Size[nodes.Length];
             var halfSep = nodeSeparation/2;
             for (int i = 0; i < nodes.Length; i++) {
                 nodeSizes[i] = nodes[i].BoundingBox.Size;
                 nodeSizes[i].Pad(halfSep);
             }
             return nodeSizes;
        }

        private static void AddStressFromProximityEdges(StressMajorization stressSolver,
                                                        List<Tuple<int, int, double, double>> proximityEdgesWithDistance) {
            //set to check whether we already added forces between two nodes
            var nodePairs = new HashSet<Tuple<int, int>>();

            // set the corresponding forces/votings
            foreach (var tuple in proximityEdgesWithDistance) {
                int nodeId1 = tuple.Item1;
                int nodeId2 = tuple.Item2;
                if (nodeId1 > nodeId2) {
                    nodeId1 = tuple.Item2;
                    nodeId2 = tuple.Item1;
                }
                var tup = new Tuple<int, int>(nodeId1, nodeId2);
                if (nodePairs.Contains(tup)) {
                    continue;
                }

                nodePairs.Add(tup);

                double distance = tuple.Item3;
                double weight = 1/(distance*distance);
                var voteFromNode1 = new Vote(nodeId1, distance, weight); // vote from node1 for node2
                var voteFromNode2 = new Vote(nodeId2, distance, weight); // vote from node2 for node1

                if (tuple.Item4 <= 1) {
                    //nodes do not overlap
                    // add the votings, which represent forces, to the corresponding block.
                    stressSolver.NodeVotings[nodeId2].VotingBlocks[0].Votings.Add(voteFromNode1);
                    // add vote of node1 to list of node2
                    stressSolver.NodeVotings[nodeId1].VotingBlocks[0].Votings.Add(voteFromNode2);
                }
                else {
                    //edges where nodes do overlap
                    // add the votings, which represent forces, to the corresponding block.
                    stressSolver.NodeVotings[nodeId2].VotingBlocks[1].Votings.Add(voteFromNode1);
                    // add vote of node1 to list of node2
                    stressSolver.NodeVotings[nodeId1].VotingBlocks[1].Votings.Add(voteFromNode2);
                    // add vote of node2 to list of node1
                }
            }
        }

        /// <summary>
        /// This method can directly be called to resolve overlaps on a graph with a given node separationl.
        /// </summary>
        /// <param name="geometryGraph"></param>
        /// <param name="nodeSeparation"></param>
        public static void RemoveOverlaps(GeometryGraph geometryGraph, double nodeSeparation) {
            var prism = new ProximityOverlapRemoval(geometryGraph) {Settings = {NodeSeparation = nodeSeparation}};
            prism.RemoveOverlaps();
            
        }


        void IOverlapRemoval.Settings(OverlapRemovalSettings settingsPar) {
            this.Settings = settingsPar;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="graph"></param>
        public void RemoveOverlaps(GeometryGraph graph) {
            this.Graph = graph;
            this.RemoveOverlaps();
        }

        /// <summary>
        /// Number of iterations of the last run.
        /// </summary>
        /// <returns></returns>
        public int GetLastRunIterations() {
            return this.LastRunIterations;
        }

        /// <summary>
        /// CpuTime of the last run.
        /// </summary>
        /// <returns></returns>
        public TimeSpan GetLastRunCpuTime() {
            return this.lastCpuTime;
        }
    }
}
