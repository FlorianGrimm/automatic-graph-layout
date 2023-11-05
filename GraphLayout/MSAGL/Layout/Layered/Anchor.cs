using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Defines the anchors for a node; anchors can be not symmetrical in general
    /// 
    ///          |TopAnchor
    ///Left anchor|
    /// ======Origin==================RightAnchor
    ///          |
    ///          |
    ///          |BottomAnchor
    /// </summary>
#if TEST_MSAGL
    public
#else
    internal
#endif
        class Anchor {
        /// <summary>
        /// ToString
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return "la:ra " +
              this.la.ToString("#.##", CultureInfo.InvariantCulture) + " " + this.ra.ToString("#.##", CultureInfo.InvariantCulture) + " ta:ba " + this.ta.ToString("#.##", CultureInfo.InvariantCulture) + " " + this.ba.ToString("#.##", CultureInfo.InvariantCulture) + " x:y " + this.x.ToString("#.##", CultureInfo.InvariantCulture) + " " + this.y.ToString("#.##", CultureInfo.InvariantCulture);
        }

        private double la;
        private double ra;
        private double ta;
        private double ba;
        private double labelCornersPreserveCoefficient;

        /// <summary>
        /// distance for the center of the node to its left boundary
        /// </summary>
        public double LeftAnchor {
            get {
                return this.la;
            }
            set {
                //the absence of this check allows a situation when an edge crosses its label or 
                // a label which does not belong to the edge
                //       if(value<-Curve.DistEps)
                //       throw new Exception("assigning negative value to a anchor");
                this.la = Math.Max(value, 0); ;
            }
        }

        /// <summary>
        /// distance from the center of the node to its right boundary
        /// </summary>
        public double RightAnchor {
            get {
                return this.ra;
            }
            set {
                //   if(value<-Curve.DistEps)
                //   throw new Exception("assigning negative value to a anchor: "+value );
                this.ra = Math.Max(value, 0);
            }
        }

        /// <summary>
        /// distance from the center of the node to its top boundary
        /// </summary>
        public double TopAnchor {
            get {
                return this.ta;
            }
            set {
                //if(value<-Curve.DistEps)
                //throw new Exception("assigning negative value to a anchor");
                this.ta = Math.Max(value, 0);
            }
        }

        /// <summary>
        /// distance from the center of the node to it bottom boundary
        /// </summary>
        public double BottomAnchor {
            get {
                return this.ba;
            }
            set {

                //if(value<-Curve.DistEps)
                //throw new InvalidOperationException();//"assigning negative value to a anchor");
                this.ba = Math.Max(value, 0);
            }
        }


        /// <summary>
        /// Left boundary of the node
        /// </summary>
        public double Left {
            get { return this.x - this.la; }
        }

        /// <summary>
        /// right boundary of the node
        /// </summary>
        public double Right {
            get { return this.x + this.ra; }
        }

        /// <summary>
        /// top boundary of the node
        /// </summary>
        public double Top {
            get { return this.y + this.ta; }
            set {
                this.y += value - this.Top;
            }
        }

        /// <summary>
        /// bottom of the node
        /// </summary>
        public double Bottom {
            get { return this.y - this.ba; }
            set { this.y += value - this.Bottom; }
        }

        /// <summary>
        /// Left top corner
        /// </summary>
        public Point LeftTop {
            get { return new Point(this.Left, this.Top); }
        }
        /// <summary>
        /// Left bottom of the node
        /// </summary>
        public Point LeftBottom {
            get { return new Point(this.Left, this.Bottom); }
        }
        /// <summary>
        /// Right bottom of the node
        /// </summary>
        public Point RightBottom {
            get { return new Point(this.Right, this.Bottom); }
        }

        private Node node;

        internal Node Node {
            get { return this.node; }
            set {
                this.node = value;
                this.polygonalBoundary = null;
            }
        }

        /// <summary>
        /// Right top of the node
        /// </summary>
        public Point RightTop {
            get { return new Point(this.Right, this.Top); }
        }

        /// <summary>
        /// an empty constructor
        /// </summary>
        public Anchor(double labelCornersPreserveCoefficient) {
            this.labelCornersPreserveCoefficient = labelCornersPreserveCoefficient;
        }
        /// <summary>
        /// constructor
        /// </summary>
        public Anchor(double leftAnchor, double rightAnchor,
            double topAnchor, double bottomAnchor, Node node, double labelCornersPreserveCoefficient) {
            this.la = leftAnchor;
            this.ra = rightAnchor;
            this.ta = topAnchor;
            this.ba = bottomAnchor;
            this.Node = node;
            this.labelCornersPreserveCoefficient = labelCornersPreserveCoefficient;
        }

        private double x;
        /// <summary>
        /// the x position
        /// </summary>
        internal double X {
            get {
                return this.x;
            }

            set {
                this.polygonalBoundary = null;
                this.x = value;
            }
        }

        private double y;
        /// <summary>
        /// the y position
        /// </summary>
        internal double Y {
            get {
                return this.y;
            }

            set {
                this.polygonalBoundary = null;
                this.y = value;
            }
        }

        /// <summary>
        /// Center of the node
        /// </summary>
        public Point Origin {
            get {
                return new Point(this.x, this.y);
            }
        }

        private bool alreadySitsOnASpline;
        /// <summary>
        /// signals if the spline has been routed already through the node
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811")]
        public bool AlreadySitsOnASpline {
            get { return this.alreadySitsOnASpline; }
            set { this.alreadySitsOnASpline = value; }
        }

        /// <summary>
        /// node widths
        /// </summary>
        public double Width {
            get { return this.la + this.ra; }
        }

        /// <summary>
        /// node height
        /// </summary>
        public double Height {
            get { return this.ta + this.ba; }
        }


        /// <summary>
        /// set to true if the anchor has been introduced for a label
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811")]
        public bool RepresentsLabel {
            get { return this.LabelToTheRightOfAnchorCenter || this.LabelToTheLeftOfAnchorCenter; }
        }

        private bool labelIsToTheLeftOfTheSpline;

        /// <summary>
        /// An anchor for an edge label with the label to the right of the spline has its height equal to the one of the label
        /// Its leftAnchor is a reserved space for the spline and the rightAnchor is equal to the label width.
        /// </summary>
        internal bool LabelToTheLeftOfAnchorCenter {
            get { return this.labelIsToTheLeftOfTheSpline; }
            set { this.labelIsToTheLeftOfTheSpline = value; }
        }

        private bool labelIsToTheRightOfTheSpline;

        /// <summary>
        /// An anchor for an edge label with the label to the left of the spline has its height equal to the one of the label
        /// Its rightAnchor is a reserved space for the spline and the leftAnchor is equal to the label width.
        /// </summary>
        internal bool LabelToTheRightOfAnchorCenter {
            get { return this.labelIsToTheRightOfTheSpline; }
            set { this.labelIsToTheRightOfTheSpline = value; }
        }

        internal bool HasLabel {
            get { return this.LabelToTheRightOfAnchorCenter || this.LabelToTheLeftOfAnchorCenter; }
        }

        internal double LabelWidth {
            get {
                if (this.LabelToTheLeftOfAnchorCenter) {
                    return this.LeftAnchor;
                }

                if (this.LabelToTheRightOfAnchorCenter) {
                    return this.RightAnchor;
                }

                throw new InvalidOperationException();
            }
        }

        private Polyline polygonalBoundary;
        /// <summary>
        /// the polygon representing the boundary of a node
        /// </summary>
#if TEST_MSAGL
        public
#else
        internal
#endif
 Polyline PolygonalBoundary {
            get {
                if (this.polygonalBoundary != null) {
                    return this.polygonalBoundary;
                }

                return this.polygonalBoundary = Pad(this.CreatPolygonalBoundaryWithoutPadding(), this.Padding);
                }
        }

        private static Polyline Pad(Polyline curve, double padding) {
            if (padding == 0) {
                return curve;
            }

            if (CurveIsConvex(curve)) {
                return PadConvexCurve(curve, padding);
            } else {
                return PadConvexCurve(Curve.StandardRectBoundary(curve), padding);
            }
        }

        private static void PadCorner(Polyline poly, PolylinePoint p0, PolylinePoint p1, PolylinePoint p2, double padding) {
            Point a, b;
            int numberOfPoints=GetPaddedCorner(p0, p1, p2, out a, out b, padding);
            poly.AddPoint(a);
            if (numberOfPoints==2) {
                poly.AddPoint(b);
            }
        }

        private static  Polyline PadConvexCurve(Polyline poly, double padding) { 
            Polyline ret = new Polyline();

            PadCorner(ret, poly.EndPoint.Prev, poly.EndPoint, poly.StartPoint, padding);
            PadCorner(ret, poly.EndPoint, poly.StartPoint, poly.StartPoint.Next, padding);

            for (PolylinePoint pp = poly.StartPoint; pp.Next.Next != null; pp = pp.Next) {
                PadCorner(ret, pp, pp.Next, pp.Next.Next, padding);
            }

            ret.Closed = true;
            return ret;

        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        /// <param name="third"></param>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="padding"></param>
        /// <returns>number of new points</returns>
        private static int GetPaddedCorner(PolylinePoint first, PolylinePoint second, PolylinePoint third, out Point a, out Point b,
            double padding) {
            Point u = first.Point;
            Point v = second.Point;
            Point w = third.Point;
            bool ccw = Point.GetTriangleOrientation(u, v, w) == TriangleOrientation.Counterclockwise;

            //uvPerp has to look outside of the curve
            var uvPerp = (v - u).Rotate((ccw? - Math.PI:Math.PI) / 2).Normalize();



            //l is bisector of the corner (u,v,w) pointing out of the corner - outside of the polyline
            Point l = (v - u).Normalize() + (v - w).Normalize();
            Debug.Assert(l * uvPerp >= 0);
            if (l.Length < ApproximateComparer.IntersectionEpsilon) {
                a = b = v + padding * uvPerp;
                return 1;
            }
// flip uvPerp if it points inside of the polyline
            Point d = l.Normalize() * padding;
            Point dp = d.Rotate(Math.PI / 2);

            //look for a in the form d+x*dp
            //we have:  Padding=(d+x*dp)*uvPerp
            double xp = (padding - d * uvPerp) / (dp * uvPerp);
            a = d + xp * dp + v;
            b = d - xp * dp + v;
            return 2; //number of points to add 
        }

        private static IEnumerable<TriangleOrientation> Orientations(Polyline poly) {
            yield return Point.GetTriangleOrientation(poly.EndPoint.Point, poly.StartPoint.Point, poly.StartPoint.Next.Point);
            yield return Point.GetTriangleOrientation(poly.EndPoint.Prev.Point, poly.EndPoint.Point, poly.StartPoint.Point);
              
            var pp = poly.StartPoint;
            while (pp.Next.Next != null ) {
                yield return Point.GetTriangleOrientation(pp.Point, pp.Next.Point, pp.Next.Next.Point);
                pp = pp.Next;
            }
        }

        private static bool CurveIsConvex(Polyline poly) {
            var orientation = TriangleOrientation.Collinear;
            foreach (var or in Orientations(poly)) {
                if (or == TriangleOrientation.Collinear) {
                    continue;
                }

                if (orientation == TriangleOrientation.Collinear) {
                    orientation = or;
                } else if (or != orientation) {
                    return false;
                }
            }
            return true;
        }


        //private static double TurnAfterSeg(Curve curve, int i) {
        //    return Point.SignedDoubledTriangleArea(curve.Segments[i].Start, curve.Segments[i].End, curve.Segments[(i + 1) / curve.Segments.Count].End);
        //}

        private Polyline CreatPolygonalBoundaryWithoutPadding() {
            Polyline ret;
            if (this.HasLabel) {
                ret = this.LabelToTheLeftOfAnchorCenter ? this.PolygonOnLeftLabel() : this.PolygonOnRightLabel();
            } else if (this.NodeBoundary == null) {
                ret = this.StandardRectBoundary();
            } else {
                ret = Curve.PolylineAroundClosedCurve(this.NodeBoundary);
            }

            return ret;
        }

        private Polyline StandardRectBoundary() {
            Polyline poly = new Polyline();
            poly.AddPoint(this.LeftTop);
            poly.AddPoint(this.RightTop);
            poly.AddPoint(this.RightBottom);
            poly.AddPoint(this.LeftBottom);
            poly.Closed = true;
            return poly;
        }

        private Polyline PolygonOnLeftLabel() {
            Polyline poly = new Polyline();
            double t = this.Left + (1 - this.labelCornersPreserveCoefficient) * this.LabelWidth;
            poly.AddPoint(new Point(t, this.Top));
            poly.AddPoint(this.RightTop);
            poly.AddPoint(this.RightBottom);
            poly.AddPoint(new Point(t, this.Bottom));
            poly.AddPoint(new Point(this.Left, this.Y));
            poly.Closed = true;
            return poly;
        }

        private Polyline PolygonOnRightLabel() {
            Polyline poly = new Polyline();
            double t = this.Right - (1 - this.labelCornersPreserveCoefficient) * this.LabelWidth;
            poly.AddPoint(t, this.Top);
            poly.AddPoint(this.Right, this.Y);
            poly.AddPoint(t, this.Bottom);
            poly.AddPoint(this.Left, this.Bottom);
            poly.AddPoint(this.Left, this.Top);
            poly.Closed = true;
            return poly;

        }

      
        internal ICurve NodeBoundary {
            get { return this.Node !=null? this.Node.BoundaryCurve:null; }
        }

        private double padding;
        /// <summary>
        /// node padding
        /// </summary>
        public double Padding {
            get { return this.padding; }
            set { this.padding = value; }
        }

        internal void Move(Point p){
            this.X += p.X;
            this.Y += p.Y;
        }

#if TEST_MSAGL

        /// <summary>
        /// UserData
        /// </summary>
        public object UserData { get; set; }

#endif

    }
}
