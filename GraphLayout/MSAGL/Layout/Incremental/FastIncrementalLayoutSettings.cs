using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics.CodeAnalysis;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// Fast incremental layout settings
    /// </summary>
#if PROPERTY_GRID_SUPPORT
    [DisplayName("Fast Incremental layout settings")]
    [Description("Settings for Fast Incremental Layout algorithm"),
    TypeConverter(typeof(ExpandableObjectConverter))]
#endif
    public class FastIncrementalLayoutSettings : LayoutAlgorithmSettings {

        /// <summary>
        /// Stop after maxIterations completed
        /// </summary>
        private int maxIterations = 100;

        /// <summary>
        /// Stop after maxIterations completed
        /// </summary>
        public int MaxIterations {
            get { return this.maxIterations; }
            set { this.maxIterations = value; }
        }

        private int minorIterations = 3;
        /// <summary>
        /// Number of iterations in inner loop.
        /// </summary>
        public int MinorIterations {
            get { return this.minorIterations; }
            set { this.minorIterations = value; }
        }

        private int iterations;
        /// <summary>
        /// Number of iterations completed
        /// </summary>
        public int Iterations {
            get { return this.iterations; }
            set { this.iterations = value; }
        }

        private int projectionIterations = 5;
        /// <summary>
        /// number of times to project over all constraints at each layout iteration
        /// </summary>
        public int ProjectionIterations {
            get { return this.projectionIterations; }
            set { this.projectionIterations = value; }
        }

        private bool approximateRepulsion = true;
        /// <summary>
        /// Rather than computing the exact repulsive force between all pairs of nodes (which would take O(n^2) time for n nodes)
        /// use a fast inexact technique (that takes O(n log n) time)
        /// </summary>
        public bool ApproximateRepulsion {
            get { return this.approximateRepulsion; }
            set { this.approximateRepulsion = value; }
        }

        /// <summary>
        /// RungaKutta integration potentially gives smoother increments, but is more expensive
        /// </summary>
        public bool RungeKuttaIntegration {
            get;
            set;
        }

        private double initialStepSize = 1.4;
        /// <summary>
        /// StepSize taken at each iteration (a coefficient of the force on each node) adapts depending on change in
        /// potential energy at each step.  With this scheme changing the InitialStepSize doesn't have much effect
        /// because if it is too large or too small it will be quickly updated by the algorithm anyway.
        /// </summary>
        public double InitialStepSize {
            get { return this.initialStepSize; }
            set {
                if (value <= 0 || value > 2) {
                    throw new ArgumentException("ForceScalar should be greater than 0 and less than 2 (if we let you set it to 0 nothing would happen, greater than 2 would most likely be very unstable!)");
                }
                this.initialStepSize = value;
            }
        }

        private double decay = 0.9;
        /// <summary>
        /// FrictionalDecay isn't really friction so much as a scaling of velocity to improve convergence.  0.8 seems to work well.
        /// </summary>
        public double Decay {
            get { return this.decay; }
            set {
                if (value < 0.1 || value > 1) {
                    throw new ArgumentException("Setting decay too small gives no progress.  1==no decay, 0.1==minimum allowed value");
                }
                this.decay = value;
            }
        }

        private double friction = 0.8;
        /// <summary>
        /// Friction isn't really friction so much as a scaling of velocity to improve convergence.  0.8 seems to work well.
        /// </summary>
        public double Friction {
            get { return this.friction; }
            set {
                if (value < 0 || value > 1) {
                    throw new ArgumentException("Setting friction less than 0 or greater than 1 would just be strange.  1==no friction, 0==no conservation of velocity");
                }
                this.friction = value;
            }
        }

        private double repulsiveForceConstant = 1.0;
        /// <summary>
        /// strength of repulsive force between each pair of nodes.  A setting of 1.0 should work OK.
        /// </summary>
        public double RepulsiveForceConstant {
            get { return this.repulsiveForceConstant; }
            set { this.repulsiveForceConstant = value; }
        }

        private double attractiveForceConstant = 1.0;
        /// <summary>
        /// strength of attractive force between pairs of nodes joined by an edge.  A setting of 1.0 should work OK.
        /// </summary>
        public double AttractiveForceConstant {
            get { return this.attractiveForceConstant; }
            set { this.attractiveForceConstant = value; }
        }

        private double gravity = 1.0;
        /// <summary>
        /// gravity is a constant force applied to all nodes attracting them to the Origin
        /// and keeping disconnected components from flying apart.  A setting of 1.0 should work OK.
        /// </summary>
        public double GravityConstant {
            get { return this.gravity; }
            set { this.gravity = value; }
        }

        private bool interComponentForces = true;
        /// <summary>
        /// If the following is false forces will not be considered between each component and each component will have its own gravity origin.
        /// </summary>
        public bool InterComponentForces
        {
            get { return this.interComponentForces; }
            set { this.interComponentForces = value; }
        }

        private bool applyForces = true;
        /// <summary>
        /// If the following is false forces will not be applied, but constraints will still be satisfied.
        /// </summary>
        public bool ApplyForces
        {
            get { return this.applyForces; }
            set { this.applyForces = value; }
        }

        internal FastIncrementalLayout algorithm;
        internal LinkedList<LockPosition> locks = new LinkedList<LockPosition>();

        /// <summary>
        /// Add a LockPosition for each node whose position you want to keep fixed.  LockPosition allows you to,
        /// for example, do interactive mouse
        ///  dragging.
        /// We return the LinkedListNode which you can store together with your local Node object so that a RemoveLock operation can be performed in
        /// constant time.
        /// </summary>
        /// <param name="node"></param>
        /// <param name="bounds"></param>
        /// <returns>LinkedListNode which you should hang on to if you want to call RemoveLock later on.</returns>
        public LockPosition CreateLock(Node node, Rectangle bounds) {
            LockPosition lp = new LockPosition(node, bounds);
            lp.listNode = this.locks.AddLast(lp);
            return lp;
        }

        /// <summary>
        /// Add a LockPosition for each node whose position you want to keep fixed.  LockPosition allows you to,
        /// for example, do interactive mouse dragging.
        /// We return the LinkedListNode which you can store together with your local Node object so that a RemoveLock operation can be performed in
        /// constant time.
        /// </summary>
        /// <param name="node"></param>
        /// <param name="bounds"></param>
        /// <param name="weight">stay weight of lock</param>
        /// <returns>LinkedListNode which you should hang on to if you want to call RemoveLock later on.</returns>
        public LockPosition CreateLock(Node node, Rectangle bounds, double weight)
        {
            LockPosition lp = new LockPosition(node, bounds, weight);
            lp.listNode = this.locks.AddLast(lp);
            return lp;
        }

        
        /// <summary>
        /// restart layout, use e.g. after a mouse drag or non-structural change to the graph
        /// </summary>
        public void ResetLayout()
        {
            this.Unconverge();
            if (this.algorithm != null)
            {
                this.algorithm.ResetNodePositions();
                this.algorithm.SetLockNodeWeights();
            }
        }

        /// <summary>
        /// reset iterations and convergence status
        /// </summary>
        internal void Unconverge()
        {

            this.iterations = 0;
            //EdgeRoutesUpToDate = false;
            this.converged = false;
        }

		/// <summary>
		/// 
		/// </summary>
		public void InitializeLayout(GeometryGraph graph, int initialConstraintLevel)
        {
            this.InitializeLayout(graph, initialConstraintLevel, anyCluster => this);
        }

        /// <summary>
        /// Initialize the layout algorithm
        /// </summary>
        /// <param name="graph">The graph upon which layout is performed</param>
        /// <param name="initialConstraintLevel"></param>
        /// <param name="clusterSettings"></param>
        public void InitializeLayout(GeometryGraph graph, int initialConstraintLevel, Func<Cluster, LayoutAlgorithmSettings> clusterSettings) 
        {
            ValidateArg.IsNotNull(graph, "graph");
            this.algorithm = new FastIncrementalLayout(graph, this, initialConstraintLevel, clusterSettings);
            this.ResetLayout();
        }

        /// <summary>
        /// 
        /// </summary>
        public void Uninitialize()
        {
            this.algorithm = null;
        }

        /// <summary>
        /// 
        /// </summary>
        public bool IsInitialized
        {
            get { return this.algorithm != null; }
        }

		/// <summary>
		/// 
		/// </summary>
		public void IncrementalRun(GeometryGraph graph)
        {
            this.IncrementalRun(graph, anyCluster => this);
        }

        private void SetupIncrementalRun(GeometryGraph graph, Func<Cluster, LayoutAlgorithmSettings> clusterSettings)
        {
            ValidateArg.IsNotNull(graph, "graph");
            if (!this.IsInitialized)
            {
                this.InitializeLayout(graph, this.MaxConstraintLevel, clusterSettings);
            }
            else if (this.IsDone)
            {
                // If we were already done from last time but we are doing more work then something has changed.
                this.ResetLayout();
            }
        }

        /// <summary>
        /// Run the FastIncrementalLayout instance incrementally
        /// </summary>
        public void IncrementalRun(GeometryGraph graph, Func<Cluster, LayoutAlgorithmSettings> clusterSettings)
        {
            this.SetupIncrementalRun(graph, clusterSettings);
            this.algorithm.Run();
            graph.UpdateBoundingBox();
        }

		/// <summary>
		/// 
		/// </summary>
		public void IncrementalRun(CancelToken cancelToken, GeometryGraph graph, Func<Cluster, LayoutAlgorithmSettings> clusterSettings)
        {
            if (cancelToken != null)
            {
                cancelToken.ThrowIfCanceled();
            }
            this.SetupIncrementalRun(graph, clusterSettings);
            this.algorithm.Run(cancelToken);
            graph.UpdateBoundingBox();
        }

        /// <summary>
        /// Clones the object
        /// </summary>
        /// <returns></returns>
        public override LayoutAlgorithmSettings Clone() {
            return this.MemberwiseClone() as LayoutAlgorithmSettings;
        }

        /// <summary>
        /// 
        /// </summary>
        public IEnumerable<IConstraint> StructuralConstraints {
            get { return this.structuralConstraints; }
        }

        /// <summary>
        /// 
        /// </summary>
        public void AddStructuralConstraint(IConstraint cc) {
            this.structuralConstraints.Add(cc);
        }

        internal List<IConstraint> structuralConstraints = new List<IConstraint>();
        /// <summary>
        /// Clear all constraints over the graph
        /// </summary>
        public void ClearConstraints() {
            this.locks.Clear();
            this.structuralConstraints.Clear();
           // clusterHierarchies.Clear();
        }

        /// <summary>
        /// 
        /// </summary>
        public void ClearStructuralConstraints()
        {
            this.structuralConstraints.Clear();
        }

        /// <summary>
        /// Avoid overlaps between nodes boundaries, and if there are any
        /// clusters, then between each cluster boundary and nodes that are not
        /// part of that cluster.
        /// </summary>
        public bool AvoidOverlaps { get; set; }

        /// <summary>
        /// If edges have FloatingPorts then the layout will optimize edge lengths based on the port locations.
        /// If MultiLocationFloatingPorts are specified then the layout will choose the nearest pair of locations for each such edge.
        /// </summary>
        public bool RespectEdgePorts { get; set; }

        /// <summary>
        /// Apply nice but expensive routing of edges once layout converges
        /// </summary>
        public bool RouteEdges { get; set; }

        private bool approximateRouting = true;
        /// <summary>
        /// If RouteEdges is true then the following is checked to see whether to do optimal shortest path routing
        /// or use a sparse visibility graph spanner to do approximate---but much faster---shortest path routing
        /// </summary>
        public bool ApproximateRouting {
            get { return this.approximateRouting; }
            set { this.approximateRouting = value; }
        }

        private bool logScaleEdgeForces = true;
        /// <summary>
        /// If true then attractive forces across edges are computed as:
        /// AttractiveForceConstant * actualLength * Math.Log((actualLength + epsilon) / (idealLength + epsilon))
        /// where epsilon is a small positive constant to avoid divide by zero or taking the log of zero.
        /// Note that LogScaleEdges can lead to ghost forces in highly constrained scenarios.
        /// If false then a the edge force is based on (actualLength - idealLength)^2, which works better with
        /// lots of constraints.
        /// </summary>
        public bool LogScaleEdgeForces {
            get { return this.logScaleEdgeForces; }
            set { this.logScaleEdgeForces = value; }
        }

        private double displacementThreshold = 0.1;
        /// <summary>
        /// If the amount of total squared displacement after a particular iteration falls below DisplacementThreshold then Converged is set to true.
        /// Make DisplacementThreshold larger if you want layout to finish sooner - but not necessarily make as much progress towards a good layout.
        /// </summary>
        public double DisplacementThreshold {
            get { return this.displacementThreshold; }
            set { this.displacementThreshold = value; }
        }

        private bool converged;
        /// <summary>
        /// Set to true if displacement from the last iteration was less than DisplacementThreshold.        
        /// The caller should invoke FastIncrementalLayout.CalculateLayout() in a loop, e.g.:
        /// 
        ///  while(!settings.Converged) 
        ///  {
        ///    layout.CalculateLayout();
        ///    redrawGraphOrHandleInteractionOrWhatever();
        ///  }
        ///  
        /// RemainingIterations affects damping.
        /// </summary>
        public bool Converged { 
            get { return this.converged; }
            set { this.converged = value; }
        }

        /// <summary>
        /// Return iterations as a percentage of MaxIterations.  Useful for reporting progress, e.g. in a progress bar.
        /// </summary>
        public int PercentDone {
            get {
                if (this.Converged) {
                    return 100;
                } else {
                    return (int)((100.0 * (double)this.iterations) / (double)this.MaxIterations);
                }
            }
        }

        /// <summary>
        /// Not quite the same as Converged:
        /// </summary>
        public bool IsDone {
            get {
                return this.Converged || this.iterations >= this.MaxIterations;
            }
        }

        /// <summary>
        /// Returns an estimate of the cost function calculated in the most recent iteration.
        /// It's a float because FastIncrementalLayout.Energy is a volatile float so it
        /// can be safely read from other threads
        /// </summary>
        public float Energy
        {
            get
            {
                if (this.algorithm != null)
                {
                    return this.algorithm.energy;
                }
                return 0;
            }
        }

        /// <summary>
        /// When layout is in progress the following is false.  
        /// When layout has converged, routes are populated and this is set to true to tell the UI that the routes can be drawn.
        /// </summary>
        public bool EdgeRoutesUpToDate { get; set; }

        private int maxConstraintLevel = 2;
        /// <summary>
        /// 
        /// </summary>
        public int MaxConstraintLevel { 
            get { 
                return this.maxConstraintLevel; 
            } 
            set {
                if (this.maxConstraintLevel != value)
                {
                    this.maxConstraintLevel = value;
                    if (this.IsInitialized)
                    {
                        this.Uninitialize();
                    }
                }
            } 
        }

        private int minConstraintLevel = 0;
        /// <summary>
        /// 
        /// </summary>
        public int MinConstraintLevel { 
            get 
            { 
                return this.minConstraintLevel; 
            } 
            set 
            {
                this.minConstraintLevel = value;
            } 
        }

        /// <summary>
        /// Constraint level ranges from Min to MaxConstraintLevel.
        /// 0 = no constraints
        /// 1 = only structural constraints
        /// 2 = all constraints including non-overlap constraints
        /// 
        /// A typical run of FastIncrementalLayout will apply it at each constraint level, starting at 0 to
        /// obtain an untangled unconstrained layout, then 1 to introduce structural constraints and finally 2 to beautify.
        /// Running only at level 2 will most likely leave the graph stuck in a tangled local minimum.
        /// </summary>
        public int GetCurrentConstraintLevel() {
            if (this.algorithm == null) {
                return 0;
            }

            return this.algorithm.GetCurrentConstraintLevel();
        }

        /// <summary>
        /// Constraint level ranges from Min to MaxConstraintLevel.
        /// 0 = no constraints
        /// 1 = only structural constraints
        /// 2 = all constraints including non-overlap constraints
        /// 
        /// A typical run of FastIncrementalLayout will apply it at each constraint level, starting at 0 to
        /// obtain an untangled unconstrained layout, then 1 to introduce structural constraints and finally 2 to beautify.
        /// Running only at level 2 will most likely leave the graph stuck in a tangled local minimum.
        /// </summary>
        public void SetCurrentConstraintLevel(int value) {
            this.algorithm.SetCurrentConstraintLevel(value);
        }

        private double attractiveInterClusterForceConstant = 1.0;
        /// <summary>
        /// Attractive strength of edges connected to clusters
        /// </summary>
        public double AttractiveInterClusterForceConstant {
            get
            {
                return this.attractiveInterClusterForceConstant;
            }
            set
            {
                this.attractiveInterClusterForceConstant = value;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public FastIncrementalLayoutSettings()
        {
        }

        /// <summary>
        /// Shallow copy the settings
        /// </summary>
        /// <param name="previousSettings"></param>
        public FastIncrementalLayoutSettings(FastIncrementalLayoutSettings previousSettings)
        {
            ValidateArg.IsNotNull(previousSettings, "previousSettings");
            this.maxIterations = previousSettings.maxIterations;
            this.minorIterations = previousSettings.minorIterations;
            this.projectionIterations = previousSettings.projectionIterations;
            this.approximateRepulsion = previousSettings.approximateRepulsion;
            this.initialStepSize = previousSettings.initialStepSize;
            this.RungeKuttaIntegration = previousSettings.RungeKuttaIntegration;
            this.decay = previousSettings.decay;
            this.friction = previousSettings.friction;
            this.repulsiveForceConstant = previousSettings.repulsiveForceConstant;
            this.attractiveForceConstant = previousSettings.attractiveForceConstant;
            this.gravity = previousSettings.gravity;
            this.interComponentForces = previousSettings.interComponentForces;
            this.applyForces = previousSettings.applyForces;
            this.IdealEdgeLength = previousSettings.IdealEdgeLength;
            this.AvoidOverlaps = previousSettings.AvoidOverlaps;
            this.RespectEdgePorts = previousSettings.RespectEdgePorts;
            this.RouteEdges = previousSettings.RouteEdges;
            this.approximateRouting = previousSettings.approximateRouting;
            this.logScaleEdgeForces = previousSettings.logScaleEdgeForces;
            this.displacementThreshold = previousSettings.displacementThreshold;
            this.minConstraintLevel = previousSettings.minConstraintLevel;
            this.maxConstraintLevel = previousSettings.maxConstraintLevel;
            this.attractiveInterClusterForceConstant = previousSettings.attractiveInterClusterForceConstant;
            this.clusterGravity = previousSettings.clusterGravity;
            this.PackingAspectRatio = previousSettings.PackingAspectRatio;
            this.NodeSeparation = previousSettings.NodeSeparation;
            this.ClusterMargin = previousSettings.ClusterMargin;
        }

        private double clusterGravity = 1.0;

        /// <summary>
        /// Controls how tightly members of clusters are pulled together
        /// </summary>
        public double ClusterGravity
        {
            get
            {
                return this.clusterGravity;
            }
            set
            {
                this.clusterGravity = value;
            }
        }

        /// <summary>
        /// Settings for calculation of ideal edge length
        /// </summary>
        public EdgeConstraints IdealEdgeLength { get; set; }

        private bool updateClusterBoundaries = true;

        /// <summary>
        /// Force groups to follow their constituent nodes, 
        /// true by default.
        /// </summary>
        public bool UpdateClusterBoundariesFromChildren
        {
            get { return this.updateClusterBoundaries; }
            set { this.updateClusterBoundaries = value; }
        }

        /// <summary>
        ///     creates the settings that seems working
        /// </summary>
        /// <returns></returns>
        public static FastIncrementalLayoutSettings CreateFastIncrementalLayoutSettings() {
            return new FastIncrementalLayoutSettings {
                                                         ApplyForces = false,
                                                         ApproximateRepulsion = true,
                                                         ApproximateRouting = true,
                                                         AttractiveForceConstant = 1.0,
                                                         AttractiveInterClusterForceConstant = 1.0,
                                                         AvoidOverlaps = true,
                                                         ClusterGravity = 1.0,
                                                         Decay = 0.9,
                                                         DisplacementThreshold = 0.00000005,
                                                         Friction = 0.8,
                                                         GravityConstant = 1.0,
                                                         InitialStepSize = 2.0,
                                                         InterComponentForces = false,
                                                         Iterations = 0,
                                                         LogScaleEdgeForces = false,
                                                         MaxConstraintLevel = 2,
                                                         MaxIterations = 20,
                                                         MinConstraintLevel = 0,
                                                         MinorIterations = 1,
                                                         ProjectionIterations = 5,
                                                         RepulsiveForceConstant = 2.0,
                                                         RespectEdgePorts = false,
                                                         RouteEdges = false,
                                                         RungeKuttaIntegration = true,
                                                         UpdateClusterBoundariesFromChildren = true,
                                                         NodeSeparation = 20
                                                     };
        }
    }
}
