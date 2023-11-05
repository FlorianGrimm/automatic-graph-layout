using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.Incremental;
using Microsoft.Msagl.Routing;
using Node = Microsoft.Msagl.Core.Layout.Node;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// Fast incremental layout is a force directed layout strategy with approximate computation of long-range node-node repulsive forces to achieve O(n log n) running time per iteration.
    /// It can be invoked on an existing layout (for example, as computed by MDS) to beautify it.  See docs for CalculateLayout method (below) to see how to use it incrementally.
    /// 
    /// Note that in debug mode lots of numerical checking is applied, which slows things down considerably.  So, run in Release mode unless you're actually debugging!
    /// </summary>
    public class FastIncrementalLayout : AlgorithmBase {
        private readonly BasicGraphOnEdges<FiEdge> basicGraph;
        private readonly List<FiNode[]> components;
        internal readonly Dictionary<int, List<IConstraint>> constraints = new Dictionary<int, List<IConstraint>>();
        private readonly List<FiEdge> edges = new List<FiEdge>();

        /// <summary>
        /// Returns the derivative of the cost function calculated in the most recent iteration.
        /// It's a volatile float so that we can potentially access it from other threads safely,
        /// for example during test.
        /// </summary>
        internal volatile float energy;
        private readonly GeometryGraph graph;
        private readonly AxisSolver horizontalSolver;

        /// <summary>
        /// Construct a graph by adding nodes and edges to these lists
        /// </summary>
        private readonly FiNode[] nodes;
        private int progress;
        private readonly FastIncrementalLayoutSettings settings;
        private double stepSize;
        private readonly AxisSolver verticalSolver;
        private readonly Func<Cluster, LayoutAlgorithmSettings> clusterSettings;
        private List<Edge> clusterEdges = new List<Edge>();

        /// <summary>
        /// Create the graph data structures.
        /// </summary>
        /// <param name="geometryGraph"></param>
        /// <param name="settings">The settings for the algorithm.</param>
        /// <param name="initialConstraintLevel">initialize at this constraint level</param>
        /// <param name="clusterSettings">settings by cluster</param>
        internal FastIncrementalLayout(GeometryGraph geometryGraph, FastIncrementalLayoutSettings settings,
                                       int initialConstraintLevel,
                                       Func<Cluster, LayoutAlgorithmSettings> clusterSettings) {
            this.graph = geometryGraph;
            this.settings = settings;
            this.clusterSettings = clusterSettings;
            int i = 0;
            ICollection<Node> allNodes = this.graph.Nodes;
            this.nodes = new FiNode[allNodes.Count];
            foreach (Node v in allNodes) {
                v.AlgorithmData = this.nodes[i] = new FiNode(i, v);
                i++;
            }

            this.clusterEdges.Clear();
            this.edges.Clear();

            foreach (Edge e in this.graph.Edges) {
                if (e.Source is Cluster || e.Target is Cluster) {
                    this.clusterEdges.Add(e);
                } else {
                    this.edges.Add(new FiEdge(e));
                }

                foreach (var l in e.Labels) {
                    l.InnerPoints = l.OuterPoints = null;
                }
            }
            this.SetLockNodeWeights();
            this.components = new List<FiNode[]>();
            if (!settings.InterComponentForces) {
                this.basicGraph = new BasicGraphOnEdges<FiEdge>(this.edges, this.nodes.Length);
                foreach (var componentNodes in ConnectedComponentCalculator<FiEdge>.GetComponents(this.basicGraph)) {
                    var vs = new FiNode[componentNodes.Count()];
                    int vi = 0;
                    foreach (int v in componentNodes) {
                        vs[vi++] = this.nodes[v];
                    }
                    this.components.Add(vs);
                }
            }
            else // just one big component (regardless of actual edges)
{
                this.components.Add(this.nodes);
            }

            this.horizontalSolver = new AxisSolver(true, this.nodes, new[] {geometryGraph.RootCluster}, settings.AvoidOverlaps,
                                              settings.MinConstraintLevel, clusterSettings) {
                                                  OverlapRemovalParameters =
                                                      new OverlapRemovalParameters {
                                                          AllowDeferToVertical = true,
                   // use "ProportionalOverlap" mode only when iterative apply forces layout is being used.
                   // it is not necessary otherwise.
                                                          ConsiderProportionalOverlap = settings.ApplyForces
                                                      }
                                              };
            this.verticalSolver = new AxisSolver(false, this.nodes, new[] {geometryGraph.RootCluster}, settings.AvoidOverlaps,
                                            settings.MinConstraintLevel, clusterSettings);

            this.SetupConstraints();
            geometryGraph.RootCluster.ComputeWeight();

            foreach (
                Cluster c in geometryGraph.RootCluster.AllClustersDepthFirst().Where(c => c.RectangularBoundary == null)
                ) {
                c.RectangularBoundary = new RectangularClusterBoundary();
            }

            this.SetCurrentConstraintLevel(initialConstraintLevel);
        }

        private void SetupConstraints() {
            this.AddConstraintLevel(0);
            if (this.settings.AvoidOverlaps) {
                this.AddConstraintLevel(2);
            }
            foreach (IConstraint c in this.settings.StructuralConstraints) {
                this.AddConstraintLevel(c.Level);
                if (c is VerticalSeparationConstraint) {
                    this.verticalSolver.AddStructuralConstraint(c);
                }
                else if (c is HorizontalSeparationConstraint) {
                    this.horizontalSolver.AddStructuralConstraint(c);
                }
                else {
                    this.AddConstraint(c);
                }
            }
            EdgeConstraintGenerator.GenerateEdgeConstraints(this.graph.Edges, this.settings.IdealEdgeLength, this.horizontalSolver,
                                                            this.verticalSolver);
        }

        private int currentConstraintLevel;

        /// <summary>
        /// Controls which constraints are applied in CalculateLayout.  Setter enforces feasibility at that level.
        /// </summary>
        internal int GetCurrentConstraintLevel() { return this.currentConstraintLevel; }

        /// <summary>
        /// Controls which constraints are applied in CalculateLayout.  Setter enforces feasibility at that level.
        /// </summary>
        internal void SetCurrentConstraintLevel(int value) {
            this.currentConstraintLevel = value;
            this.horizontalSolver.ConstraintLevel = value;
            this.verticalSolver.ConstraintLevel = value;
            Feasibility.Enforce(this.settings, value, this.nodes, this.horizontalSolver.structuralConstraints,
                                this.verticalSolver.structuralConstraints, new[] { this.graph.RootCluster }, this.clusterSettings);
            this.settings.Unconverge();
        }

        /// <summary>
        /// Add constraint to constraints lists.  Warning, no check that dictionary alread holds a list for the level.
        /// Make sure you call AddConstraintLevel first (perf).
        /// </summary>
        /// <param name="c"></param>
        private void AddConstraint(IConstraint c) {
            if (!this.constraints.ContainsKey(c.Level)) {
                this.constraints[c.Level] = new List<IConstraint>();
            }
            this.constraints[c.Level].Add(c);
        }

        /// <summary>
        /// Check for constraint level in dictionary, if it doesn't exist add the list at that level.
        /// </summary>
        /// <param name="level"></param>
        private void AddConstraintLevel(int level) {
            if (!this.constraints.ContainsKey(level)) {
                this.constraints[level] = new List<IConstraint>();
            }
        }

        internal void SetLockNodeWeights() {
            foreach (LockPosition l in this.settings.locks) {
                l.SetLockNodeWeight();
            }
        }

        internal void ResetNodePositions() {
            foreach (FiNode v in this.nodes) {
                v.ResetBounds();
            }
            foreach (var e in this.edges) {
                foreach (var l in e.mEdge.Labels) {
                    l.InnerPoints = l.OuterPoints = null;
                }
            }
        }

        private void AddRepulsiveForce(FiNode v, Point repulsion) {
            // scale repulsion
            v.force = 10.0* this.settings.RepulsiveForceConstant*repulsion;
        }

        private void AddLogSpringForces(FiEdge e, Point duv, double d) {
            double l = duv.Length,
                   f = 0.0007* this.settings.AttractiveForceConstant*l*Math.Log((l + 0.1)/(d + 0.1));
            e.source.force += f*duv;
            e.target.force -= f*duv;
        }

        private void AddSquaredSpringForces(FiEdge e, Point duv, double d) {
            double l = duv.Length,
                   d2 = d*d + 0.1,
                   f = this.settings.AttractiveForceConstant*(l - d)/d2;
            e.source.force += f*duv;
            e.target.force -= f*duv;
        }

        private void AddSpringForces(FiEdge e) {
            Point duv;
            if (this.settings.RespectEdgePorts) {
                var sourceLocation = e.source.Center;
                var targetLocation = e.target.Center;
                var sourceFloatingPort = e.mEdge.SourcePort as FloatingPort;
                if (sourceFloatingPort != null) {
                    sourceLocation = sourceFloatingPort.Location;
                }
                var targetFloatingPort = e.mEdge.TargetPort as FloatingPort;
                if (targetFloatingPort != null) {
                    targetLocation = targetFloatingPort.Location;
                }
                duv = sourceLocation - targetLocation;
            }
            else {
                duv = e.vector();
            }
            if (this.settings.LogScaleEdgeForces) {
                this.AddLogSpringForces(e, duv, e.mEdge.Length);
            }
            else {
                this.AddSquaredSpringForces(e, duv, e.mEdge.Length);
            }
        }

        private static void AddGravityForce(Point origin, double gravity, FiNode v) {
            // compute and add gravity
            v.force -= 0.0001*gravity*(origin - v.Center);
        }

        private void ComputeRepulsiveForces(FiNode[] vs) {
            int n = vs.Length;
            if (n > 16 && this.settings.ApproximateRepulsion) {
                var ps = new KDTree.Particle[vs.Length];
                // before calculating forces we perturb each center by a small vector in a unique
                // but deterministic direction (by walking around a circle in n steps) - this allows
                // the KD-tree to decompose even when some nodes are at exactly the same position
                double angle = 0, angleDelta = 2.0*Math.PI/n;
                for (int i = 0; i < n; ++i) {
                    ps[i] = new KDTree.Particle(vs[i].Center + 1e-5*new Point(Math.Cos(angle), Math.Sin(angle)));
                    angle += angleDelta;
                }
                var kdTree = new KDTree(ps, 8);
                kdTree.ComputeForces(5);
                for (int i = 0; i < vs.Length; ++i) {
                    this.AddRepulsiveForce(vs[i], ps[i].force);
                }
            }
            else {
                foreach (FiNode u in vs) {
                    var fu = new Point();
                    foreach (FiNode v in vs) {
                        if (u != v) {
                            fu += MultipoleCoefficients.Force(u.Center, v.Center);
                        }
                    }
                    this.AddRepulsiveForce(u, fu);
                }
            }
        }

        private void AddClusterForces(Cluster root) {
            if (root == null) {
                return;
            }
            // SetBarycenter is recursive.
            root.SetBarycenter();
            // The cluster edges draw the barycenters of the connected clusters together
            foreach (var e in this.clusterEdges) {
                // foreach cluster keep a force vector.  Replace ForEachNode calls below with a simple
                // addition to this force vector.  Traverse top down, tallying up force vectors of children
                // to be the sum of their parents.
                var c1 = e.Source as Cluster;
                var c2 = e.Target as Cluster;
                var n1 = e.Source.AlgorithmData as FiNode;
                var n2 = e.Target.AlgorithmData as FiNode;
                Point center1 = c1 != null ? c1.Barycenter : n1.Center;
                Point center2 = c2 != null ? c2.Barycenter : n2.Center;
                Point duv = center1 - center2;
                double l = duv.Length,
                       f = 1e-8* this.settings.AttractiveInterClusterForceConstant*l*Math.Log(l + 0.1);
                if (c1 != null) {
                    c1.ForEachNode(v => {
                        var fv = v.AlgorithmData as FiNode;
                        fv.force += f*duv;
                    });
                }
                else {
                    n1.force += f*duv;
                }
                if (c2 != null) {
                    c2.ForEachNode(v => {
                        var fv = v.AlgorithmData as FiNode;
                        fv.force -= f*duv;
                    });
                }
                else {
                    n2.force -= f*duv;
                }
            }
            foreach (Cluster c in root.AllClustersDepthFirst()) {
                if (c != root) {
                    c.ForEachNode(v => AddGravityForce(c.Barycenter, this.settings.ClusterGravity, (FiNode) v.AlgorithmData));
                }
            }
        }

        /// <summary>
        /// Aggregate all the forces affecting each node
        /// </summary>
        private void ComputeForces() {
            if (this.components != null) {
                this.components.ForEach(this.ComputeRepulsiveForces);
            }
            else {
                this.ComputeRepulsiveForces(this.nodes);
            }
            this.edges.ForEach(this.AddSpringForces);
            foreach (var c in this.components) {
                var origin = new Point();
                for (int i = 0; i < c.Length; ++i) {
                    origin += c[i].Center;
                }
                origin /= (double) c.Length;
                double maxForce = double.NegativeInfinity;
                for (int i = 0; i < c.Length; ++i) {
                    FiNode v = c[i];
                    AddGravityForce(origin, this.settings.GravityConstant, v);
                    if (v.force.Length > maxForce) {
                        maxForce = v.force.Length;
                    }
                }
                if (maxForce > 100.0) {
                    for (int i = 0; i < c.Length; ++i) {
                        c[i].force *= 100.0/maxForce;
                    }
                }
            }
            // This is the only place where ComputeForces (and hence verletIntegration) considers clusters.
            // It's just adding a "gravity" force on nodes inside each cluster towards the barycenter of the cluster.
            this.AddClusterForces(this.graph.RootCluster);
        }

        private void SatisfyConstraints() {
            for (int i = 0; i < this.settings.ProjectionIterations; ++i) {
                foreach (var level in this.constraints.Keys) {
                    if (level > this.GetCurrentConstraintLevel()) {
                        break;
                    }
                    foreach (var c in this.constraints[level]) {
                        c.Project();
                        // c.Project operates only on MSAGL nodes, so need to update the local FiNode.Centers
                        foreach (var v in c.Nodes) {
                            ((FiNode) v.AlgorithmData).Center = v.Center;
                        }
                    }
                }

                foreach (LockPosition l in this.settings.locks) {
                    l.Project();
                    // again, project operates only on MSAGL nodes, we'll also update FiNode.PreviousPosition since we don't want any inertia in this case
                    foreach (var v in l.Nodes) {
                        FiNode fiNode = v.AlgorithmData as FiNode;

                        // the locks should have had their AlgorithmData updated, but if (for some reason)
                        // the locks list is out of date we don't want to null ref here.
                        if (fiNode != null && v.AlgorithmData != null) {
                            fiNode.ResetBounds();
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Checks if solvers need to be applied, i.e. if there are user constraints or 
        /// generated constraints (such as non-overlap) that need satisfying
        /// </summary>
        /// <returns></returns>
        private bool NeedSolve() {
            return this.horizontalSolver.NeedSolve || this.verticalSolver.NeedSolve;
        }

        /// <summary>
        /// Force directed layout is basically an iterative approach to solving a bunch of differential equations.
        /// Different integration schemes are possible for applying the forces iteratively.  Euler is the simplest:
        ///  v_(i+1) = v_i + a dt
        ///  x_(i+1) = x_i + v_(i+1) dt
        /// 
        /// Verlet is much more stable (and not really much more complicated):
        ///  x_(i+1) = x_i + (x_i - x_(i-1)) + a dt dt
        /// </summary>
        private double VerletIntegration() {
            // The following sets the Centers of all nodes to a (not necessarily feasible) configuration that reduces the cost (forces)
            float energy0 = this.energy;
            this.energy = (float)this.ComputeDescentDirection(1.0);
            this.UpdateStepSize(energy0);
            this.SolveSeparationConstraints();

            double displacementSquared = 0;
            for (int i = 0; i < this.nodes.Length; ++i) {
                FiNode v = this.nodes[i];
                displacementSquared += (v.Center - v.previousCenter).LengthSquared;
            }
            return displacementSquared;
        }

        private void SolveSeparationConstraints() {
            if (this.NeedSolve()) {
                // Increasing the padding effectively increases the size of the rectangle, so it will lead to more overlaps,
                // and therefore tighter packing once the overlap is removed and therefore more apparent "columnarity".
                // We don't want to drastically change the shape of the rectangles, just increase them ever so slightly so that
                // there is a bit more space in the horizontal than vertical direction, thus reducing the likelihood that
                // the vertical constraint generation will detect spurious overlaps, which should allow the nodes to slide
                // smoothly around each other.  ConGen padding args are:  First pad is in direction of the constraints being
                // generated, second pad is in the perpendicular direction.
                double dblVpad = this.settings.NodeSeparation;
                double dblHpad = dblVpad + Feasibility.Pad;
                double dblCVpad = this.settings.ClusterMargin;
                double dblCHpad = dblCVpad + Feasibility.Pad;

                // The centers are our desired positions, but we need to find a feasible configuration
                foreach (FiNode v in this.nodes) {
                    v.desiredPosition = v.Center;
                }
                // Set up horizontal non-overlap constraints based on the (feasible) starting configuration
                this.horizontalSolver.Initialize(dblHpad, dblVpad, dblCHpad, dblCVpad, v => v.previousCenter);
                this.horizontalSolver.SetDesiredPositions();
                this.horizontalSolver.Solve();

                // generate y constraints
                this.verticalSolver.Initialize(dblHpad, dblVpad, dblCHpad, dblCVpad, v => v.Center);
                this.verticalSolver.SetDesiredPositions();
                this.verticalSolver.Solve();

                // If we have multiple locks (hence multiple high-weight nodes), there can still be some
                // movement of the locked variables - so update all lock positions.
                foreach (LockPosition l in this.settings.locks.Where(l => !l.Sticky)) {
                    l.Bounds = l.node.BoundingBox;
                }
            }
        }

        private double ComputeDescentDirection(double alpha) {
            this.ResetForceVectors();
            // velocity is the distance travelled last time step
            if (this.settings.ApplyForces) {
                this.ComputeForces();
            }
            double lEnergy = 0;
            foreach (FiNode v in this.nodes) {
                lEnergy += v.force.LengthSquared;
                Point dx = v.Center - v.previousCenter;
                v.previousCenter = v.Center;
                dx *= this.settings.Friction;
                Point a = -this.stepSize *alpha*v.force;
                Debug.Assert(!double.IsNaN(a.X), "!double.IsNaN(a.X)");
                Debug.Assert(!double.IsNaN(a.Y), "!double.IsNaN(a.Y)");
                Debug.Assert(!double.IsInfinity(a.X), "!double.IsInfinity(a.X)");
                Debug.Assert(!double.IsInfinity(a.Y), "!double.IsInfinity(a.Y)");
                dx += a;
                dx /= v.stayWeight;
                v.Center += dx;
            }
            this.SatisfyConstraints();
            return lEnergy;
        }

        private void ResetForceVectors() {
            foreach (var v in this.nodes) {
                v.force = new Point();
            }
        }

        /// <summary>
        /// Adapt StepSize based on change in energy.  
        /// Five sequential improvements in energy mean we increase the stepsize.
        /// Any increase in energy means we reduce the stepsize.
        /// </summary>
        /// <param name="energy0"></param>
        private void UpdateStepSize(float energy0) {
            if (this.energy < energy0) {
                if (++this.progress >= 3) {
                    this.progress = 0;
                    this.stepSize /= this.settings.Decay;
                }
            }
            else {
                this.progress = 0;
                this.stepSize *= this.settings.Decay;
            }
        }

        private double RungeKuttaIntegration() {
            var y0 = new Point[this.nodes.Length];
            var k1 = new Point[this.nodes.Length];
            var k2 = new Point[this.nodes.Length];
            var k3 = new Point[this.nodes.Length];
            var k4 = new Point[this.nodes.Length];
            float energy0 = this.energy;
            this.SatisfyConstraints();
            for (int i = 0; i < this.nodes.Length; ++i) {
                y0[i] = this.nodes[i].previousCenter = this.nodes[i].Center;
            }
            const double alpha = 3;
            this.ComputeDescentDirection(alpha);
            for (int i = 0; i < this.nodes.Length; ++i) {
                k1[i] = this.nodes[i].Center - this.nodes[i].previousCenter;
                this.nodes[i].Center = y0[i] + 0.5*k1[i];
            }
            this.ComputeDescentDirection(alpha);
            for (int i = 0; i < this.nodes.Length; ++i) {
                k2[i] = this.nodes[i].Center - this.nodes[i].previousCenter;
                this.nodes[i].previousCenter = y0[i];
                this.nodes[i].Center = y0[i] + 0.5*k2[i];
            }
            this.ComputeDescentDirection(alpha);
            for (int i = 0; i < this.nodes.Length; ++i) {
                k3[i] = this.nodes[i].Center - this.nodes[i].previousCenter;
                this.nodes[i].previousCenter = y0[i];
                this.nodes[i].Center = y0[i] + k3[i];
            }
            this.energy = (float)this.ComputeDescentDirection(alpha);
            for (int i = 0; i < this.nodes.Length; ++i) {
                k4[i] = this.nodes[i].Center - this.nodes[i].previousCenter;
                this.nodes[i].previousCenter = y0[i];
                Point dx = (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i])/6.0;
                this.nodes[i].Center = y0[i] + dx;
            }
            this.UpdateStepSize(energy0);
            this.SolveSeparationConstraints();

            return this.nodes.Sum(v => (v.Center - v.previousCenter).LengthSquared);
        }

        /// <summary>
        /// Apply a small number of iterations of the layout.  
        /// The idea of incremental layout is that settings.minorIterations should be a small number (e.g. 3) and 
        /// CalculateLayout should be invoked in a loop, e.g.:
        /// 
        /// while(settings.RemainingIterations > 0) {
        ///    fastIncrementalLayout.CalculateLayout();
        ///    InvokeYourProcedureToRedrawTheGraphOrHandleInteractionEtc();
        /// }
        /// 
        /// In the verletIntegration step above, the RemainingIterations is used to control damping.
        /// </summary>
        protected override void RunInternal() {
            this.settings.Converged = false;
            this.settings.EdgeRoutesUpToDate = false;
            if (this.settings.Iterations++ == 0) {
                this.stepSize = this.settings.InitialStepSize;
                this.energy = float.MaxValue;
                this.progress = 0;
            }
            this.StartListenToLocalProgress(this.settings.MinorIterations);
            for (int i = 0; i < this.settings.MinorIterations; ++i) {                
                double d2 = this.settings.RungeKuttaIntegration ? this.RungeKuttaIntegration() : this.VerletIntegration();

                if (d2 < this.settings.DisplacementThreshold || this.settings.Iterations > this.settings.MaxIterations) {
                    this.settings.Converged = true;
                    this.ProgressComplete();
                    break;
                }

                this.ProgressStep();
            }
            this.FinalizeClusterBoundaries();
        }

        /// <summary>
        /// Simply does a depth first traversal of the cluster hierarchies fitting Rectangles to the contents of the cluster
        /// or updating the cluster BoundingBox to the already calculated RectangularBoundary
        /// </summary>
        private void FinalizeClusterBoundaries() {
            foreach (var c in this.graph.RootCluster.AllClustersDepthFirst()) {
                if (c == this.graph.RootCluster) {
                    continue;
                }

                if (!this.NeedSolve() && this.settings.UpdateClusterBoundariesFromChildren) {
                    // if we are not using the solver (e.g. when constraintLevel == 0) then we need to get the cluster bounds manually
                    c.CalculateBoundsFromChildren(this.settings.ClusterMargin);
                }
                else {
                    c.BoundingBox = c.RectangularBoundary.Rect;
                }
                c.RaiseLayoutDoneEvent();
            }
        }
    }
}
