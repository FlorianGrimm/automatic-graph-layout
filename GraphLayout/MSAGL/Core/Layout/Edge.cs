using System;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// Edge of the graph
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class Edge : GeometryObject, ILabeledObject {
        /// <summary>
        /// Defines the way the edge connects to the source.
        /// The member is used at the moment only when adding an edge to the graph.
        /// </summary>
        public Port SourcePort {
            get { return this.EdgeGeometry.SourcePort; }
            set { this.EdgeGeometry.SourcePort = value; }
        }

        /// <summary>
        /// defines the way the edge connects to the target
        /// The member is used at the moment only when adding an edge to the graph.
        /// </summary>
        public Port TargetPort {
            get { return this.EdgeGeometry.TargetPort; }
            set { this.EdgeGeometry.TargetPort = value; }
        }

        private readonly List<Label> labels = new List<Label>();

        /// <summary>
        /// gets the default (first) label of the edge
        /// </summary>
        public Label Label {
            get{
                if (this.labels.Count == 0) {
                    return null;
                }

                return this.labels[0];
            }
            set {
                if (this.labels.Count == 0) {
                    this.labels.Add(value);
                } else {
                    this.labels[0] = value;
                }
            }
        }
        /// <summary>
        /// Returns the full enumeration of labels associated with this edge
        /// </summary>
        public IList<Label> Labels {
            get { return this.labels; }
        }

        private Node source;

        /// <summary>
        /// id of the source node
        /// </summary>
        public Node Source {
            get { return this.source; }
            set { this.source = value; }
        }

        private Node target;

        /// <summary>
        /// id of the target node
        /// </summary>
        public Node Target {
            get { return this.target; }
            set { this.target = value; }
        }


        /// <summary>
        /// Label width, need to backup it for transformation purposes
        /// </summary>
        internal double OriginalLabelWidth { get; set; }

        /// <summary>
        /// Original label height
        /// </summary>
        internal double OriginalLabelHeight { get; set; }

        /// <summary>
        /// Edge constructor
        /// </summary>
        /// <param name="source"></param>
        /// <param name="target"></param>
        /// <param name="labelWidth"></param>
        /// <param name="labelHeight"></param>
        /// <param name="edgeThickness"></param>
        public Edge(Node source, Node target, double labelWidth, double labelHeight, double edgeThickness) {
            this.source = source;
            this.target = target;
            if (labelWidth > 0) {
                this.Label = new Label(labelWidth, labelHeight, this);
            }

            this.LineWidth = edgeThickness;
        }

        /// <summary>
        /// Constructs an edge without a label or arrowheads and with edge thickness 1.
        /// </summary>
        /// <param name="source">souce node</param>
        /// <param name="target">target node</param>
        public Edge(Node source, Node target)
            : this(source, target, 0, 0, 1) {
        }

        /// <summary>
        /// The default constructor
        /// </summary>
        public Edge() : this(null, null) {
        }
        
        /// <summary>
        /// The label bounding box
        /// </summary>
        internal Rectangle LabelBBox {
            get { return this.Label.BoundingBox; }
        }

        private double length = 1;

        /// <summary>
        /// applicable for MDS layouts
        /// </summary>
        public double Length {
            get { return this.length; }
            set { this.length = value; }
        }

        private int weight = 1;

        /// <summary>
        /// The greater is the weight the more important is keeping the edge short. It is 1 by default.
        /// Other values are not tested yet.
        /// </summary>
        public int Weight {
            get { return this.weight; }
            set { this.weight = value; }
        }

        private int separation = 1;

        /// <summary>
        /// The minimum number of levels dividing source from target: 1 means that the edge goes down at least one level.
        /// Separation is 1 by default. Other values are not tested yet.
        /// </summary>
        public int Separation {
            get { return this.separation; }
            set { this.separation = value; }
        }


        /// <summary>
        /// overrides ToString
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return this.source + "->" + this.target;
        }

        /// <summary>
        /// edge thickness
        /// </summary>
        public double LineWidth {
            get { return this.EdgeGeometry.LineWidth; }
            set { this.EdgeGeometry.LineWidth = value; }
        }

        /// <summary>
        /// The bounding box of the edge curve
        /// </summary>
        public override Rectangle BoundingBox {
            get {

                var rect = Rectangle.CreateAnEmptyBox();
                if (this.UnderlyingPolyline != null) {
                    foreach (Point p in this.UnderlyingPolyline) {
                        rect.Add(p);
                    }
                }

                if (this.Curve != null) {
                    rect.Add(this.Curve.BoundingBox);
                }

                if (this.EdgeGeometry != null) {
                    if (this.EdgeGeometry.SourceArrowhead != null) {
                        rect.Add(this.EdgeGeometry.SourceArrowhead.TipPosition);
                    }

                    if (this.EdgeGeometry.TargetArrowhead != null) {
                        rect.Add(this.EdgeGeometry.TargetArrowhead.TipPosition);
                    }
                }

                double del = this.LineWidth;
                rect.Left -= del;
                rect.Top += del;
                rect.Right += del;
                rect.Bottom -= del;
                return rect;
            }
            set { throw new NotImplementedException(); }
        }

        private EdgeGeometry edgeGeometry = new EdgeGeometry();
        public object Color;

        /// <summary>
        /// Gets or sets the edge geometry: the curve, the arrowhead positions and the underlying polyline
        /// </summary>
        public EdgeGeometry EdgeGeometry {
            get { return this.edgeGeometry; }
            set { this.edgeGeometry = value; }
        }

        /// <summary>
        /// the polyline of the untrimmed spline
        /// </summary>
        public SmoothedPolyline UnderlyingPolyline {
            get { return this.edgeGeometry.SmoothedPolyline; }
            set { this.edgeGeometry.SmoothedPolyline = value; }
        }

        /// <summary>
        /// A curve representing the edge
        /// </summary>
        public ICurve? Curve {
            get { return this.edgeGeometry?.Curve; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.edgeGeometry.Curve = value;
            }
        }

        /// <summary>
        /// Transform the curve, arrowheads and label according to the given matrix
        /// </summary>
        /// <param name="matrix">affine transform matrix</param>
        internal void Transform(PlaneTransformation matrix)
        {
            if (this.Curve == null) {
                return;
            }

            this.Curve = this.Curve.Transform(matrix);
            if (this.UnderlyingPolyline != null) {
                for (CornerSite s = this.UnderlyingPolyline.HeadSite, s0 = this.UnderlyingPolyline.HeadSite;
                     s != null;
                     s = s.Next, s0 = s0.Next) {
                    s.Point = matrix * s.Point;
                }
            }

            var sourceArrow = this.edgeGeometry.SourceArrowhead;
            if (sourceArrow != null) {
                sourceArrow.TipPosition = matrix * sourceArrow.TipPosition;
            }

            var targetArrow = this.edgeGeometry.TargetArrowhead;
            if (targetArrow != null) {
                targetArrow.TipPosition = matrix * targetArrow.TipPosition;
            }

            if (this.Label != null) {
                this.Label.Center = matrix * this.LabelBBox.Center;
            }
        }

        /// <summary>
        /// Translate the edge curve arrowheads and label by the specified delta
        /// </summary>
        /// <param name="delta">amount to shift geometry</param>
        public void Translate(Point delta)
        {
            if (this.EdgeGeometry != null)
            {
                this.EdgeGeometry.Translate(delta);
            }
            foreach (var l in this.Labels)
            {
                l.Translate(delta);
            }
        }

		/// <summary>
		/// transforms relative to given rectangles
		/// </summary>
		public void TransformRelativeTo(Rectangle oldBounds, Rectangle newBounds)
        {
            if (this.EdgeGeometry != null) {
                var toOrigin = new PlaneTransformation(1, 0, -oldBounds.Left, 0, 1, -oldBounds.Bottom);
                var scale = new PlaneTransformation(newBounds.Width/oldBounds.Width, 0, 0,
                                                    0,newBounds.Height/oldBounds.Height, 0);
                var toNewBounds = new PlaneTransformation(1, 0, newBounds.Left, 0, 1, newBounds.Bottom);
                this.Transform(toNewBounds*scale*toOrigin);
            }
            foreach (var l in this.Labels)
            {
                l.Translate(newBounds.LeftBottom - oldBounds.LeftBottom);
            }
        }

        /// <summary>
        /// Checks if an arrowhead is needed at the source
        /// </summary>
        public bool ArrowheadAtSource
        {
            get
            {
                return this.EdgeGeometry != null && this.EdgeGeometry.SourceArrowhead != null;
            }
        }

        /// <summary>
        /// Checks if an arrowhead is needed at the target
        /// </summary>
        public bool ArrowheadAtTarget
        {
            get
            {
                return this.EdgeGeometry != null && this.EdgeGeometry.TargetArrowhead != null;
            }
        }
        
        /// <summary>
        /// Routes a self edge inside the given "howMuchToStickOut" parameter
        /// </summary>
        /// <param name="boundaryCurve"></param>
        /// <param name="howMuchToStickOut"></param>
        /// <param name="smoothedPolyline"> the underlying polyline used later for editing</param>
        /// <returns></returns>
        static internal ICurve RouteSelfEdge(ICurve boundaryCurve, double howMuchToStickOut, out SmoothedPolyline smoothedPolyline)
        {
            //we just need to find the box of the corresponding node
            var w = boundaryCurve.BoundingBox.Width;
            var h = boundaryCurve.BoundingBox.Height;
            var center = boundaryCurve.BoundingBox.Center;

            var p0 = new Point(center.X - w / 4, center.Y);
            var p1 = new Point(center.X - w / 4, center.Y - h / 2 - howMuchToStickOut);
            var p2 = new Point(center.X + w / 4, center.Y - h / 2 - howMuchToStickOut);
            var p3 = new Point(center.X + w / 4, center.Y);

            smoothedPolyline = SmoothedPolyline.FromPoints(new[] { p0, p1, p2, p3 });

            return smoothedPolyline.CreateCurve();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="newValue"></param>
        public override void RaiseLayoutChangeEvent(object? newValue) {
            this.edgeGeometry.RaiseLayoutChangeEvent(newValue);
        }

        
        /// <summary>
        /// 
        /// </summary>
        public override event EventHandler<LayoutChangeEventArgs> BeforeLayoutChangeEvent {
            add { this.edgeGeometry.LayoutChangeEvent+=value; }
            remove { this.edgeGeometry.LayoutChangeEvent-=value; }
        }

        internal bool UnderCollapsedCluster() {
            return this.Source.UnderCollapsedCluster() || this.Target.UnderCollapsedCluster();
        }
    }
}
