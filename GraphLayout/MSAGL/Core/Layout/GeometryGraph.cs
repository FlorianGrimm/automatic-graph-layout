using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Layout.LargeGraphLayout;
#if TEST_MSAGL
using Microsoft.Msagl.DebugHelpers;
using System.Diagnostics;
#endif

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// This class keeps the graph nodes, edges, and clusters, together with their geometries
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class GeometryGraph : GeometryObject
    {
        private IList<Node> nodes;
        private EdgeCollection edges;
#if TEST_MSAGL
    [NonSerialized]
        private
#endif
         Cluster rootCluster;

        /// <summary>
        /// Creates a new GeometryGraph.
        /// </summary>
        public GeometryGraph()
        {
            this.nodes = new NodeCollection(this);
            this.edges = new EdgeCollection(this);
            this.rootCluster = new Cluster();
        }

        /// <summary>
        /// The root cluster for this graph. Will never be null.
        /// </summary>
        public Cluster RootCluster 
        { 
            get
            {
                return this.rootCluster;
            }
            set
            {
                ValidateArg.IsNotNull(value, "value");
                this.rootCluster = value;
            }
        }

        internal Rectangle boundingBox;

        /// <summary>
        /// Bounding box of the graph
        /// </summary>
        public override Rectangle BoundingBox {
            get { return this.boundingBox; }
            set { this.boundingBox = value; }
        }

        private double margins;
#if TEST_MSAGL
        /// <summary>
        /// curves to show debug stuff
        /// </summary>
        public DebugCurve[] DebugCurves;
#endif

        /// <summary>
        /// margins width are equal from the left and from the right; they are given in percents
        /// </summary>
        public double Margins
        {
            get { return this.margins; }
            set { this.margins = value; }
        }

        /// <summary>
        /// Width of the graph
        /// </summary>
        public double Width {
            get { return this.BoundingBox.RightBottom.X - this.BoundingBox.LeftTop.X; }
        }

        /// <summary>
        /// Height of the graph
        /// </summary>
        public double Height {
            get { return this.BoundingBox.Height; }
        }

        /// <summary>
        /// Left bound of the graph
        /// </summary>
        public double Left {
            get { return this.BoundingBox.Left; }
        }

        /// <summary>
        /// Right bound of the graph
        /// </summary>
        public double Right {
            get { return this.BoundingBox.Right; }
        }

        /// <summary>
        /// Left bottom corner of the graph
        /// </summary>
        internal Point LeftBottom {
            get { return new Point(this.BoundingBox.Left, this.BoundingBox.Bottom); }
        }

        /// <summary>
        /// Right top corner of the graph
        /// </summary>
        internal Point RightTop {
            get { return new Point(this.Right, this.Top); }
        }

        /// <summary>
        /// Bottom bound of the graph
        /// </summary>
        public double Bottom {
            get { return this.BoundingBox.Bottom; }
        }

        /// <summary>
        /// Top bound of the graph
        /// </summary>
        public double Top {
            get { return this.BoundingBox.Bottom + this.BoundingBox.Height; }
        }

        /// <summary>
        /// The nodes in the graph.
        /// </summary>
        public IList<Node> Nodes {
            get { return this.nodes; }
            set { this.nodes = value; }
        }

        /// <summary>
        /// Edges of the graph
        /// </summary>
        public EdgeCollection Edges {
            get { return this.edges; }
            set { this.edges =value; }
        }

        /// <summary>
        /// Returns a collection of all the labels in the graph.
        /// </summary>
        /// <returns></returns>
        public ICollection<Label> CollectAllLabels()
        {
            return this.Edges.SelectMany(e => e.Labels).ToList();
        }

        /// <summary>
        /// transforms the graph by the given matrix
        /// </summary>
        /// <param name="matrix">the matrix</param>
        public void Transform(PlaneTransformation matrix) {
            foreach (var node in this.Nodes) {
                node.Transform(matrix);
            }

            foreach (var edge in this.Edges) {
                edge.Transform(matrix);
            }
#if TEST_MSAGL
            if (this.DebugCurves != null) {
                foreach (var dc in this.DebugCurves) {
                    dc.Curve = dc.Curve.Transform(matrix);
                }
            }
#endif
            this.UpdateBoundingBox();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Rectangle PumpTheBoxToTheGraphWithMargins() {
            var b = Rectangle.CreateAnEmptyBox();
            this.PumpTheBoxToTheGraph(ref b);
            var del=new Point(this.Margins, -this.Margins);
            b.RightBottom += del;
            b.LeftTop -= del;
            b.Width = Math.Max(b.Width, this.MinimalWidth);
            b.Height = Math.Max(b.Height, this.MinimalHeight);

            return b;
        }

        ///<summary>
        ///the minimal width of the graph
        ///</summary>
        public double MinimalWidth { get; set; }
        ///<summary>
        ///the minimal height of the graph
        ///</summary>
        public double MinimalHeight { get; set; }

        /// <summary>
        /// enlarge the rectangle to contain the graph
        /// </summary>
        /// <param name="b"></param>
        private void PumpTheBoxToTheGraph(ref Rectangle b) {
            foreach (Edge e in this.Edges) {
                if (e.UnderCollapsedCluster()) {
                    continue;
                }

                if (e.Curve != null) {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=369 there are no structs in js
                    var cb = e.Curve.BoundingBox.Clone();
#else
                    var cb = e.Curve.BoundingBox;
#endif
                    cb.Pad(e.LineWidth);
                    b.Add(cb);
                }

                foreach (var l in e.Labels.Where(lbl => lbl != null)) {
                    b.Add(l.BoundingBox);
                }
            }

            foreach (Node n in this.Nodes) {
                if (n.UnderCollapsedCluster()) {
                    continue;
                }

                b.Add(n.BoundingBox);
            }


            foreach (var c in this.RootCluster.Clusters) {
                if (c.BoundaryCurve == null) {
                    if (c.RectangularBoundary != null) {
                        c.BoundaryCurve = c.RectangularBoundary.RectangularHull();
                    }
                }
                if (c.BoundaryCurve != null) {
                    b.Add(c.BoundaryCurve.BoundingBox);
                }
            }
#if TEST_MSAGL
            if(this.DebugCurves !=null) {
                foreach (var debugCurve in this.DebugCurves.Where(d => d.Curve != null)) {
                    b.Add(debugCurve.Curve.BoundingBox);
                }
            }
#endif
        }

        /// <summary>
        /// Translates the graph by delta.
        /// Assumes bounding box is already up to date.
        /// </summary>
        public void Translate(Point delta)
        {
            var nodeSet = new Set<Node>(this.Nodes);
            foreach (var v in this.Nodes) {
                v.Center += delta;
            }

            foreach (var cluster in this.RootCluster.AllClustersDepthFirstExcludingSelf()) {
                foreach (var node in cluster.Nodes.Where(n => !nodeSet.Contains(n))) {
                    node.Center += delta;
                }

                cluster.Center += delta;
                cluster.RectangularBoundary.TranslateRectangle(delta);                
            }

            foreach (var e in this.edges) {
                e.Translate(delta);
            }

            this.BoundingBox = new Rectangle(this.BoundingBox.Left + delta.X, this.BoundingBox.Bottom + delta.Y, new Point(this.BoundingBox.Width, this.BoundingBox.Height));
        }

        /// <summary>
        /// Updates the bounding box to fit the contents.
        /// </summary>
        public void UpdateBoundingBox() {
            this.BoundingBox = this.PumpTheBoxToTheGraphWithMargins();
        }

        /// <summary>
        /// Flatten the list of nodes and clusters
        /// </summary>
        /// <returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1024:UsePropertiesWhereAppropriate")]
        public IEnumerable<Node> GetFlattenedNodesAndClusters()
        {
            foreach (Node v in this.Nodes)
            {
                yield return v;
            }
            
            foreach(Cluster cluster in this.RootCluster.AllClustersDepthFirst())
            {
                if (cluster != this.RootCluster)
                {
                    yield return cluster;
                }
            }
        }

        /// <summary>
        /// Finds the first node with the corresponding user data.
        /// </summary>
        /// <returns>The first node with the given user data. Null if no such node exists.</returns>
        public Node FindNodeByUserData(object userData)
        {
            return this.Nodes.FirstOrDefault(n => n.UserData.Equals(userData));
        }
#if TEST_MSAGL
        ///<summary>
        ///</summary>

        public void SetDebugIds()
        {
            int id = 0;
            foreach (var node in this.RootCluster.AllClustersDepthFirst()) {
                node.DebugId = id++;
            }

            foreach (var node in this.Nodes) {
                if (node.DebugId == null) {
                    node.DebugId = id++;
                }
            }
        }

        internal void CheckClusterConsistency() {
            foreach (var cluster in this.RootCluster.AllClustersDepthFirst()) {
                CheckClusterConsistency(cluster);
            }
        }

        private static void CheckClusterConsistency(Cluster cluster) {
            if (cluster.BoundaryCurve == null) {
                return;
            }

            foreach (var child in cluster.Clusters.Concat(cluster.Nodes)) {
                var inside=Curve.CurveIsInsideOther(child.BoundaryCurve, cluster.BoundaryCurve);
#if TEST_MSAGL
//                if (!inside)
//                    LayoutAlgorithmSettings.ShowDebugCurves(new DebugCurve("green", cluster.BoundaryCurve), new DebugCurve("red", child.BoundaryCurve));
#endif
                Debug.Assert(inside,
                             "A child of a cluster has to have the BoundaryCurve inside of the BoundaryCurve of the cluster");
            }

        }
#endif
        /// <summary>
        /// info of layers for large graph browsing
        /// </summary>
        public LgData LgData { get; set; }
    }
}
