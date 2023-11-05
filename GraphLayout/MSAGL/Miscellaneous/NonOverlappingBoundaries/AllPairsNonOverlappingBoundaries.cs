using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Layout.Incremental;

namespace Microsoft.Msagl.Prototype.NonOverlappingBoundaries {
    /// <summary>
    /// A CvxHull is Convex hull
    /// </summary>
    public abstract class CvxHull : IHull {
        /// <summary>
        /// We have a hierarchy of membership
        /// </summary>
        public ClusterConvexHull Parent { get; protected set; }
        /// <summary>
        /// see IHull
        /// </summary>
        public abstract Point Center { get; }
        /// <summary>
        /// move the center by delta
        /// </summary>
        /// <param name="delta"></param>
        public abstract void MoveCenter(Point delta);
        /// <summary>
        /// see IHull
        /// </summary>
        public abstract RectangleNode<IHull,Point> RectangleNode { get; }
        /// <summary>
        /// 
        /// </summary>
        public abstract double Weight { get; }
        /// <summary>
        /// Gets the boundary translated to the current Center
        /// </summary>
        /// <returns></returns>
        public abstract Polyline TranslatedBoundary();
        /// <summary>
        /// Resolves overlap between this and another CHull by moving on the minimum penetration depth vector
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public double Project(IHull other) {
            var v = other as CvxHull;
            var vc = v as ClusterConvexHull;
            var c = this as ClusterConvexHull;
            if (c!=null && c.Contains(v)) {
                return 0;
            }
            if (vc != null && vc.Contains(this)) {
                return 0;
            }
            Debug.Assert(v != null);
            Point pd = PenetrationDepth.PenetrationDepthForPolylines(this.TranslatedBoundary(), v.TranslatedBoundary());
            if (pd.Length > 0) {
                Point wpd = pd / (this.Weight + v.Weight);
                this.MoveCenter(v.Weight * wpd);
                v.MoveCenter(-this.Weight * wpd);
            }
            return pd.Length;
        }
    }
    /// <summary>
    /// A CvxHull for rectangles
    /// </summary>
    public class RCHull : CvxHull {
        internal Node mNode;
        private Polyline boundary;
        private Polyline translatedBoundary;
        private double w2, h2;
        /// <summary>
        /// Center of the node
        /// </summary>
        public override Point Center {
            get { return this.mNode.Center; }
        }
        /// <summary>
        /// Move by delta
        /// </summary>
        /// <param name="delta"></param>
        public override void MoveCenter(Point delta) {
            this.mNode.Center += delta;
        }
        /// <summary>
        /// RectangleNode is used in region queries
        /// </summary>
        public override RectangleNode<IHull,Point> RectangleNode {
            get {
                var r = new Rectangle(this.Center.X - this.w2, this.Center.Y - this.h2, this.Center.X + this.w2, this.Center.Y + this.h2);
                return new RectangleNode<IHull,Point>(this, r);
            }
        }
        /// <summary>
        /// 
        /// </summary>
        public override double Weight {
            get { return ((FiNode)this.mNode.AlgorithmData).stayWeight; }
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="parent"></param>
        /// <param name="padding"></param>
        /// <param name="mNode"></param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "m")]
        public RCHull(ClusterConvexHull parent, Node mNode, double padding) {
            //var node = (FastIncrementalLayout.Node)mNode.AlgorithmData;
            //this.w2 = node.width / 2.0 + padding;
            //this.h2 = node.height / 2.0 + padding;
            this.w2 = mNode.Width / 2.0 + padding;
            this.h2 = mNode.Height / 2.0 + padding;
            this.Parent = parent;

            this.boundary = new Polyline(new Point[]{
                new Point(-this.w2, -this.h2),
                new Point(-this.w2, this.h2),
                new Point(this.w2, this.h2),
                new Point(this.w2, -this.h2)
            });
            this.boundary.Closed = true;
            this.translatedBoundary = new Polyline(this.boundary);
            this.translatedBoundary.Closed = true;
            this.mNode = mNode;
        }
        /// <summary>
        /// Gets the boundary translated to the current Center
        /// </summary>
        /// <returns></returns>
        public override Polyline TranslatedBoundary() {
            PolylinePoint qq = this.translatedBoundary.StartPoint;
            for (PolylinePoint pp = this.boundary.StartPoint; pp != null; pp = pp.Next) {
                qq.Point = pp.Point + this.Center;
                qq = qq.Next;
            }
            return this.translatedBoundary;
        }
    }
    /// <summary>
    /// The convex hull of the constituents of a Cluster
    /// </summary>
    public class ClusterConvexHull : CvxHull {
        internal Cluster cluster;
        /// <summary>
        /// The Barycenter of the cluster
        /// </summary>
        public override Point Center {
            get { return this.cluster.SetBarycenter(); }
        }
        /// <summary>
        /// Move contents by delta
        /// </summary>
        /// <param name="delta"></param>
        public override void MoveCenter(Point delta)
        {
            this.cluster.ForEachNode(v => v.Center += delta);
        }
        /// <summary>
        /// 
        /// </summary>
        public override double Weight {
            get { return this.cluster.Weight; }
        }
        /// <summary>
        /// Bounding box used in region queries
        /// </summary>
        public override RectangleNode<IHull,Point> RectangleNode {
            get {
                var r = new Rectangle(this.Center);
                foreach (var node in this.cluster.Nodes) {
                    r.Add(node.BoundingBox);
                }

                return new RectangleNode<IHull,Point>(this, r);
            }
        }
        /// <summary>
        /// The convex hull of the constituents of a Cluster
        /// </summary>
        /// <param name="cluster"></param>
        /// <param name="parent"></param>
        public ClusterConvexHull(Cluster cluster, ClusterConvexHull parent) {
            this.cluster = cluster;
            this.Parent = parent;
        }
        /// <summary>
        /// Gets the boundary translated to the current Center
        /// </summary>
        /// <returns></returns>
        public override Polyline TranslatedBoundary() {
            return this.ComputeConvexHull();
        }

        /// <summary>
        /// The convex hull of all the points of all the nodes in the cluster
        /// </summary>
        private Polyline ComputeConvexHull()
        {
            var points = new List<Point>();
            foreach (Node v in this.cluster.Nodes) 
            {
                CvxHull r = new RCHull(null, v, 0);
                foreach (PolylinePoint p in r.TranslatedBoundary().PolylinePoints) {
                    points.Add(p.Point);
                }
            }

            foreach (Cluster c in this.cluster.Clusters)
            {
                points.AddRange(new ClusterConvexHull(c, this).TranslatedBoundary());
            }

            return new Polyline(ConvexHull.CalculateConvexHull(points)) {Closed = true};
        }

        /// <summary>
        /// Search hierarchy to check if child is a descendent of this.
        /// </summary>
        /// <param name="child"></param>
        /// <returns>true if child is a descendent of this cluster</returns>
        public bool Contains(CvxHull child) {
            if (child.Parent == null) {
                return false;
            }
            if (child.Parent == this) {
                return true;
            }
            return this.Contains(child.Parent);
        }
    }
    /// <summary>
    /// Prevents the boundaries of nodes and clusters from overlapping
    /// </summary>
    public class AllPairsNonOverlappingBoundaries : IConstraint {
        private List<IHull> hulls = new List<IHull>();
        private void traverseClusters(ClusterConvexHull parent, Cluster cluster, double padding) {
            ClusterConvexHull hull = new ClusterConvexHull(cluster, parent);
            this.hulls.Add(hull);
            foreach (var v in cluster.nodes) {
                this.hulls.Add(new RCHull(hull, v, padding));
            }
            foreach (var c in cluster.clusters) {
                this.traverseClusters(hull, c, padding);
            }
        }
        /// <summary>
        /// 
        /// Non-overlap between nodes in the same cluster (or the root cluster), between the convex hulls
        /// of clusters and nodes that do belong to those clusters and between clusters and clusters.
        /// </summary>
        /// <param name="cluster"></param>
        /// <param name="settings">for padding extra space around nodes</param>
        public AllPairsNonOverlappingBoundaries(Cluster cluster, FastIncrementalLayoutSettings settings) {
            foreach (var v in cluster.nodes) {
                this.hulls.Add(new RCHull(null,v, settings.NodeSeparation));
            }
            foreach (var c in cluster.clusters) {
                this.traverseClusters(null, c, settings.NodeSeparation);
            }
        }
        #region IConstraint Members
        private static int AllPairsComputationLimit = 20;
        /// <summary>
        /// Uses Lev's fast proximity query to find pairs of nodes/clusters with overlapping bounding boxes.
        /// When such are found, they are projected apart.
        /// </summary>
        public double Project() {
            double displacement = 0;
            if (this.hulls.Count < AllPairsComputationLimit) {
                // if there are only a few nodes then do it the most straightforward n^2 way
                for (int i = 0; i < this.hulls.Count - 1; ++i) {
                    IHull u = this.hulls[i];
                    for (int j = i + 1; j < this.hulls.Count; ++j) {
                        displacement += u.Project(this.hulls[j]);
                    }
                }
            } else {
                var pq = new ProximityQuery(this.hulls);
                List<Tuple<IHull, IHull>> closePairs = pq.GetAllIntersections();
                //shuffle(ref closePairs);
                foreach (var k in closePairs) {
                    displacement += k.Item1.Project(k.Item2);
                }
            }
            return displacement;
        }
        
        //void shuffle<T>(ref List<T> l) {
        //    List<T> tmpList = new List<T>();
        //    while (l.Count > 0) {
        //        int i = rand.Next(l.Count);
        //        tmpList.Add(l[i]);
        //        l.RemoveAt(i);
        //    }
        //    l = tmpList;
        //}
        /// <summary>
        /// NonOverlap constraints are a beautification thing, and therefore higher level than others
        /// </summary>
        /// <returns>2</returns>
        public int Level { get { return 2; } }
        #endregion

        #region IConstraint Members


        /// <summary>
        /// 
        /// </summary>
        public IEnumerable<Node> Nodes {
            get { return new List<Node>(); }
        }

        #endregion
    }
}
