using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    ///     A cluster has a list of nodes and a list of nested clusters
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class Cluster : Node {
        private bool isCollapsed;

        /// <summary>
        ///     this flag should be respected by layout algorithms
        /// </summary>
        public bool IsCollapsed {
            get { return this.isCollapsed; }
            set { this.isCollapsed = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        public override ICurve BoundaryCurve {
            get { return this.IsCollapsed ? this.CollapsedBoundary : base.BoundaryCurve; }
            set { base.BoundaryCurve = value; }
        }

        private event EventHandler<LayoutChangeEventArgs> layoutDoneEvent;

        /// <summary>
        ///     event signalling that the layout is done
        /// </summary>
        public event EventHandler<LayoutChangeEventArgs> LayoutDoneEvent {
            add { layoutDoneEvent += value; }
            remove { layoutDoneEvent -= value; }
        }

        internal Point Barycenter; // Filled in by SetBarycenter
        private ICurve collapsedBoundary;

        /// <summary>
        ///     the boundary curve when the cluster is collapsed
        /// </summary>
        public ICurve CollapsedBoundary {
            get { return this.collapsedBoundary; }
            set { this.collapsedBoundary = value; }
        }

        internal List<Cluster> clusters = new List<Cluster>();
        internal List<Node> nodes = new List<Node>();

        /// <summary>
        /// </summary>
        public Cluster() : this(new Point()) {}

        /// <summary>
        ///     Bottom-most ctor.
        /// </summary>
        /// <param name="origin"></param>
        public Cluster(Point origin) {
            this.Weight = 0;
            this.Barycenter = origin;
        }

        /// <summary>
        ///     Construct a cluster with the specified nodes as members
        /// </summary>
        /// <param name="nodes"></param>
        [SuppressMessage("Microsoft.Design", "CA1002:DoNotExposeGenericLists")]
        public Cluster(IEnumerable<Node> nodes)
            : this() {
            ValidateArg.IsNotNull(nodes, "nodes");
            foreach (Node v in nodes) {
                this.AddNode(v);
            }
        }

        /// <summary>
        ///     Construct a cluster with the specified nodes and clusters as child members
        /// </summary>
        /// <param name="nodes"></param>
        /// <param name="clusters"></param>
        public Cluster(IEnumerable<Node> nodes, IEnumerable<Cluster> clusters)
            : this(nodes) {
            ValidateArg.IsNotNull(clusters, "clusters");
            foreach (Cluster c in clusters) {
                this.AddCluster(c);
            }
        }

        /// <summary>
        ///     Clusters can (optionally) have a rectangular border which is respected by overlap avoidance.
        ///     Currently, this is controlled by FastIncrementalLayoutSettings.RectangularClusters.
        ///     If FastIncrementalLayoutSettings.RectangularClusters is true, then the
        ///     FastIncrementalLayout constructor will create a RectangularBoundary in each cluster.
        ///     Otherwise it will be null.
        /// </summary>
        public RectangularClusterBoundary RectangularBoundary { get; set; }

        /// <summary>
        ///     List of member nodes
        /// </summary>
        public IEnumerable<Node> Nodes {
            get { return this.nodes; }
        }

        /// <summary>
        ///     List of child clusters
        /// </summary>
        public IEnumerable<Cluster> Clusters {
            get { return this.clusters; }
        }

        /// <summary>
        ///     number of nodes in cluster
        /// </summary>
        public double Weight { get; private set; }

        /// <summary>
        ///     BoundingBox_get uses the RectangularBoundary.rectangle if available, otherwise uses the cluster's content bounds
        ///     BoundingBox_set scales the old bounds to fit the desired bounds
        /// </summary>
        public override Rectangle BoundingBox {
            get {
                if (!this.IsCollapsed || this.CollapsedBoundary ==null) {
                    if (this.RectangularBoundary != null) {
                        return this.RectangularBoundary.Rect;
                    }

                    // Default to the cluster's content bounds
                    return new Rectangle(this.nodes.Concat(this.clusters).Select(n => n.BoundingBox));
                }

                return this.CollapsedBoundary.BoundingBox;

            }
            set { this.FitBoundaryCurveToTarget(value); }
        }

        /// <summary>
        ///     Add a child node or cluster.  It is added to the correct list (nodes or clusters) based on type
        /// </summary>
        /// <param name="child"></param>
        public void AddChild(Node child) {
            ValidateArg.IsNotNull(child, "child");
            Debug.Assert(child != this);
            var childCluster = child as Cluster;
            if (childCluster != null) {
                this.clusters.Add(childCluster);
            }
            else {
                this.nodes.Add(child);
            }
            child.AddClusterParent(this);
        }

        /// <summary>
        ///     Cleares the child clusters.
        /// </summary>
        public void ClearClusters() {
            this.clusters.Clear();
        }

        /// <summary>
        ///     Compute the total weight of all nodes and clusters in this cluster.
        /// </summary>
        /// <returns></returns>
        public double ComputeWeight() {
            this.Weight = this.nodes.Count;
            foreach (Cluster c in this.clusters) {
                this.Weight += c.ComputeWeight();
            }
            return this.Weight;
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Barycenter"),
         SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "SetBarycenter")]
        public Point SetBarycenter() {
            this.Barycenter = new Point();

            // If these are empty then Weight is 0 and barycenter becomes NaN.
            // If there are no child clusters with nodes, then Weight stays 0.
            if ((0 != this.nodes.Count) || (0 != this.clusters.Count)) {
                if (0 == this.Weight) {
                    this.ComputeWeight();
                }
                if (0 != this.Weight) {
                    foreach (Node v in this.nodes) {
                        //double wv = ((FastIncrementalLayout.Node)v.AlgorithmData).stayWeight;
                        //p+=wv*v.Center;
                        //w += wv;
                        this.Barycenter += v.Center;
                    }
                    foreach (Cluster c in this.clusters) {
                        // SetBarycenter ensures Weight is calculated so call it first.
                        this.Barycenter += c.SetBarycenter()*c.Weight;
                    }
                    this.Barycenter /= this.Weight;
                }
            }
            Debug.Assert(!Double.IsNaN(this.Barycenter.X) && !Double.IsNaN(this.Barycenter.Y));
            return this.Barycenter;
        }


        /// <summary>
        ///     TODO: Check all the places we use this and make sure we don't have O(n^2) complexity
        /// </summary>
        /// <returns>This cluster and all clusters beneath this one, in depth first order</returns>
        public IEnumerable<Cluster> AllClustersDepthFirst() {
            foreach (Cluster c in this.clusters) {
                foreach (Cluster d in c.AllClustersDepthFirst()) {
                    yield return d;
                }
            }
            yield return this;
        }


        /// <summary>
        /// </summary>
        /// <returns>This cluster and all clusters beneath this one, in width first order</returns>
        public IEnumerable<Node> AllSuccessorsWidthFirst() {
            foreach (Node n in this.Nodes) {
                yield return n;
            }

            foreach (Cluster c in this.clusters) {
                yield return c;
                foreach (Node n in c.AllSuccessorsWidthFirst()) {
                    yield return n;
                }
            }
        }

       

        /// <summary>
        /// </summary>
        /// <returns>This cluster and all clusters beneath this one, in depth first order</returns>
        public IEnumerable<Cluster> AllClustersDepthFirstExcludingSelf() {
            foreach (Cluster c in this.clusters) {
                foreach (Cluster d in c.AllClustersDepthFirst()) {
                    yield return d;
                }
            }
        }

        /// <summary>
        ///     TODO: Check all the places we use this and make sure we don't have O(n^2) complexity
        /// </summary>
        /// <param name="f"></param>
        internal void ForEachNode(Action<Node> f) {
            foreach (Node v in this.nodes) {
                f(v);
            }
            foreach (Cluster c in this.clusters) {
                c.ForEachNode(f);
            }
        }

        /// <summary>
        ///     Remove the specified cluster from the list of children of this cluster
        /// </summary>
        /// <param name="cluster"></param>
        public void RemoveCluster(Cluster cluster) {
            this.clusters.Remove(cluster);
        }

        /// <summary>
        ///     Translates the cluster's contents by the delta.
        /// </summary>
        public void DeepContentsTranslation(Point delta, bool translateEdges) {
            foreach (Cluster c in this.AllClustersDepthFirst()) {
                foreach (Node v in c.Nodes.Concat(c.Clusters.Cast<Node>())) {
                    var cluster = v as Cluster;
                    if (cluster != null) {
                        cluster.RectangularBoundary.TranslateRectangle(delta);
                    }
                    v.Center += delta;
                    if (translateEdges) {
                        foreach (Edge e in this.EdgesIncomingToNodeWithDescendantSource(v)) {
                            e.Translate(delta);
                        }
                    }
                }
            }
        }

        /// <summary>
        ///     Get edges both of whose end-points are immediate children of this cluster
        /// </summary>
        /// <returns></returns>
        public IEnumerable<Edge> ChildEdges() {
            return this.Nodes.Concat(this.Clusters).SelectMany(this.EdgesIncomingToNodeWithChildSource);
        }

        /// <summary>
        ///     get the edges incoming to the specified node where the source of the edge is a descendant of this cluster.
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        private IEnumerable<Edge> EdgesIncomingToNodeWithDescendantSource(Node node) {
            return node.InEdges.Concat(node.SelfEdges).Where(e => e.Source.IsDescendantOf(this));
        }

        /// <summary>
        ///     get the edges incoming to the specified node where the source of an edge is an immediate child of this cluster.
        /// </summary>
        /// <param name="node"></param>
        /// <returns></returns>
        private IEnumerable<Edge> EdgesIncomingToNodeWithChildSource(Node node) {
            return node.InEdges.Concat(node.SelfEdges).Where(e => e.Source.ClusterParent==this);
        }

        /// <summary>
        ///     Translates the cluster and all of it's contents by the delta.
        /// </summary>
        internal void DeepTranslation(Point delta, bool translateEdges) {
            this.DeepContentsTranslation(delta, translateEdges);
            this.RectangularBoundary.TranslateRectangle(delta);
            this.Center += delta;
        }

        /// <summary>
        /// </summary>
        /// <param name="padding"></param>
        public void CalculateBoundsFromChildren(double padding) {
            var r = new Rectangle((from v in this.Nodes select v.BoundingBox).Concat(
                from d in this.Clusters select d.BoundingBox));
            r.Pad(padding);
            this.UpdateBoundary(r);
        }

        internal void UpdateBoundary(Rectangle bounds) {
            Rectangle r = bounds;
            if (this.RectangularBoundary != null) {
                r = new Rectangle(
                    r.Left - this.RectangularBoundary.LeftMargin,
                    r.Bottom - this.RectangularBoundary.BottomMargin,
                    r.Right + this.RectangularBoundary.RightMargin,
                    r.Top + this.RectangularBoundary.TopMargin);
                double widthPad = (this.RectangularBoundary.MinWidth - r.Width)/2;
                if (widthPad > 0) {
                    r.PadWidth(widthPad);
                }
                double heightPad = (this.RectangularBoundary.MinHeight - r.Height)/2;
                if (heightPad > 0) {
                    r.PadHeight(heightPad);
                }
                this.RectangularBoundary.Rect = r;
            }
            this.BoundingBox = r;
        }

      

        
        


        /// <summary>
        ///     Calculate cluster's RectangularBoundary to preserve the offsets calculated in initial layout, for example,
        ///     to allow for extra space required for non-shortest path edge routes or for labels.
        /// </summary>
        /// <param name="padding">amount of padding between child node bounding box and expected inner bounds</param>
        public void SetInitialLayoutState(double padding) {
            if (this.RectangularBoundary != null) {
                this.RectangularBoundary.StoreDefaultMargin();
                var childBounds =
                    new Rectangle(from v in this.Nodes.Concat(this.Clusters) select v.BoundingBox);
                childBounds.Pad(padding);
                this.RectangularBoundary.LeftMargin = childBounds.Left - this.RectangularBoundary.Rect.Left;
                this.RectangularBoundary.RightMargin = this.RectangularBoundary.Rect.Right - childBounds.Right;
                this.RectangularBoundary.BottomMargin = childBounds.Bottom - this.RectangularBoundary.Rect.Bottom;
                this.RectangularBoundary.TopMargin = this.RectangularBoundary.Rect.Top - childBounds.Top;
            }
        }

        /// <summary>
        /// Set the initial layout state such that our current margin is stored and the new margin is taken from the given rb
        /// </summary>
        public void SetInitialLayoutState(RectangularClusterBoundary bounds) {
            if (this.RectangularBoundary != null && bounds != null) {
                this.RectangularBoundary.StoreDefaultMargin();
                this.RectangularBoundary.LeftMargin = bounds.LeftMargin;
                this.RectangularBoundary.RightMargin = bounds.RightMargin;
                this.RectangularBoundary.BottomMargin = bounds.BottomMargin;
                this.RectangularBoundary.TopMargin = bounds.TopMargin;
            }
        }

        /// <summary>
        ///     sets IsInitialLayoutState to false and restores the default margins if we have a RectangularBoundary
        /// </summary>
        public void UnsetInitialLayoutState() {
            RectangularClusterBoundary rb = this.RectangularBoundary;
            if (rb != null) {
                rb.RestoreDefaultMargin();
            }
        }

        /// <summary>
        ///     Unset the initial layout state of this cluster and also all of its ancestors
        /// </summary>
        public void UnsetInitialLayoutStateIncludingAncestors() {
            this.UnsetInitialLayoutState();
            foreach (Cluster c in this.AllClusterAncestors) {
                c.UnsetInitialLayoutState();
            }
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        public IEnumerable<Cluster> AllClustersWideFirstExcludingSelf() {
            var q = new Queue<Cluster>();
            foreach (Cluster cluster in this.Clusters) {
                q.Enqueue(cluster);
            }

            while (q.Count > 0) {
                Cluster c = q.Dequeue();
                yield return c;
                foreach (Cluster cluster in c.Clusters) {
                    q.Enqueue(cluster);
                }
            }
        }

        /// <summary>
        /// </summary>
        /// <returns></returns>
        public IEnumerable<Cluster> AllClustersWidthFirstExcludingSelfAvoidingChildrenOfCollapsed() {
            var q = new Queue<Cluster>();
            foreach (Cluster cluster in this.Clusters) {
                q.Enqueue(cluster);
            }

            while (q.Count > 0) {
                Cluster c = q.Dequeue();
                yield return c;
                if (c.IsCollapsed) {
                    continue;
                }

                foreach (Cluster cluster in c.Clusters) {
                    q.Enqueue(cluster);
                }
            }
        }

        /// <summary>
        ///     adding a node without checking that it is a cluster
        /// </summary>
        /// <param name="node"></param>
        internal void AddNode(Node node) {
            node.AddClusterParent(this);
            this.nodes.Add(node);
        }

        internal void AddCluster(Cluster cluster) {
            cluster.AddClusterParent(this);
            this.clusters.Add(cluster);
        }

        internal void AddRangeOfCluster(IEnumerable<Cluster> clustersToAdd) {
            foreach (Cluster cluster in clustersToAdd) {
                cluster.AddClusterParent(this);
                this.clusters.Add(cluster);
            }
        }

        /// <summary>
        ///     to string!
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return this.UserData != null ? this.UserData.ToString() : base.ToString();
        }

        internal void RaiseLayoutDoneEvent() {
            if (layoutDoneEvent != null) {
                layoutDoneEvent(this, null);
            }
        }
    }
}