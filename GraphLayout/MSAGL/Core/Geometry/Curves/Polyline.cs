using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Text;

namespace Microsoft.Msagl.Core.Geometry.Curves {
    /// <summary>
    /// class representing a polyline
    /// </summary>
    [SuppressMessage("Microsoft.Naming", "CA1710:IdentifiersShouldHaveCorrectSuffix"),
     SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
#if TEST_MSAGL
    [Serializable]
#endif
    public class Polyline : ICurve, IEnumerable<Point> {
        private bool needToInit = true;

        /// <summary>
        /// 
        /// </summary>
        internal void RequireInit() {
            this.needToInit = true;
        }

        private bool NeedToInit {
            get {
                return this.needToInit;
            }
            set {
                this.needToInit = value;
            }
        }

		/// <summary>
		/// 
		/// </summary>
		public IEnumerable<PolylinePoint> PolylinePoints {
            get {
                PolylinePoint p = this.StartPoint;
                while (p != null) {
                    yield return p;
                    p = p.Next;
                }
            }
        }
#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.String.Format(System.String,System.Object,System.Object,System.Object)")]
        public override string ToString() {
            return String.Format("{0},{1},count={2}", this.Start, this.End, this.Count);
        }
#endif 
        internal Curve ToCurve() {
            var c = new Curve();
            Curve.AddLineSegment(c, this.StartPoint.Point, this.StartPoint.Next.Point);
            PolylinePoint p = this.StartPoint.Next;
            while ((p = p.Next) != null) {
                Curve.ContinueWithLineSegment(c, p.Point);
            }

            if (this.Closed) {
                Curve.ContinueWithLineSegment(c, this.StartPoint.Point);
            }

            return c;
        }

        private ParallelogramInternalTreeNode pBNode;
        private PolylinePoint startPoint;

		/// <summary>
		/// 
		/// </summary>
		public PolylinePoint StartPoint {
            get { return this.startPoint; }
            set {
                this.RequireInit();
                this.startPoint = value;
            }
        }

        private PolylinePoint endPoint;

		/// <summary>
		/// 
		/// </summary>
		public PolylinePoint EndPoint {
            get { return this.endPoint; }
            set {
                this.RequireInit();
                this.endPoint = value;
            }
        }

        private int count;

        internal int Count {
            get {
                if (this.needToInit) {
                    this.Init();
                }

                return this.count;
            }            
        }

        private bool closed;

        /// <summary>
        /// 
        /// </summary>
        public bool Closed {
            get { return this.closed; }
            set {
                if (this.closed != value) {
                    this.closed = value;
                    this.RequireInit();
                }
            }
        }

        #region ICurve Members

        /// <summary>
        /// the value of the curve at the parameter
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public Point this[double t] {
            get {
                Point a, b;
                if (this.NeedToInit) {
                    this.Init();
                }

                this.GetAdjustedParamAndStartEndPoints(ref t, out a, out b);
                return (1 - t)*a + t*b;
            }
        }

        private void GetAdjustedParamAndStartEndPoints(ref double t, out Point a, out Point b) {
            Debug.Assert(t >= -ApproximateComparer.Tolerance);
            Debug.Assert(this.StartPoint != null);
            PolylinePoint s = this.StartPoint;

            while (s.Next != null) {
                if (t <= 1) {
                    a = s.Point;
                    b = s.Next.Point;
                    return;
                }
                s = s.Next;
                t -= 1;
            }

            if (this.Closed) {
                if (t <= 1) {
                    a = this.EndPoint.Point;
                    b = this.StartPoint.Point;
                    return;
                }
            }

            throw new InvalidOperationException(); //"out of the parameter domain");
        }


        /// <summary>
        /// first derivative at t
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point Derivative(double t) {
            Point a, b;
            if (this.NeedToInit) {
                this.Init();
            }

            this.GetAdjustedParamAndStartEndPoints(ref t, out a, out b);
            return b - a;
        }


        /// <summary>
        /// left derivative at t
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point LeftDerivative(double t) {
            if(this.NeedToInit) {
                this.Init();
            }

            PolylinePoint pp = this.TryToGetPolylinePointCorrespondingToT(t);
            if (pp == null) {
                return this.Derivative(t);
            }

            PolylinePoint prev = this.TryToGetPrevPointToPolylinePoint(pp);
            if (prev != null) {
                return pp.Point - prev.Point;
            }

            return pp.Next.Point - pp.Point;
        }

        /// <summary>
        /// right derivative at t
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point RightDerivative(double t) {
            if(this.NeedToInit) {
                this.Init();
            }

            var pp = this.TryToGetPolylinePointCorrespondingToT(t);
            if (pp == null) {
                return this.Derivative(t);
            }

            PolylinePoint next = this.TryToGetNextPointToPolylinePoint(pp);
            if (next != null) {
                return next.Point - pp.Point;
            }

            return pp.Point - pp.Prev.Point;
        }

        private PolylinePoint? TryToGetPolylinePointCorrespondingToT(double t) {
            for (PolylinePoint p = this.StartPoint; p != null; p = p.Next, t--) {
                if (Math.Abs(t) < ApproximateComparer.Tolerance) {
                    return p;
                }
            }

            return null;
        }

        private PolylinePoint? TryToGetPrevPointToPolylinePoint(PolylinePoint p) {
            if (p != this.StartPoint) {
                return p.Prev;
            }

            if (!this.Closed) {
                return null;
            }

            return this.EndPoint;
        }

        private PolylinePoint? TryToGetNextPointToPolylinePoint(PolylinePoint p) {
            if (p != this.EndPoint) {
                return p.Next;
            }

            if (!this.Closed) {
                return null;
            }

            return this.StartPoint;
        }


        /// <summary>
        /// second derivative
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public Point SecondDerivative(double t) {
            return new Point();
        }

        /// <summary>
        /// third derivative
        /// </summary>
        /// <param name="t">the parameter of the derivative</param>
        /// <returns></returns>
        public Point ThirdDerivative(double t)
        {
            return new Point();
        }


        /// <summary>
        /// A tree of ParallelogramNodes covering the curve. 
        /// This tree is used in curve intersections routines.
        /// </summary>
        /// <value></value>
        public ParallelogramNodeOverICurve ParallelogramNodeOverICurve {
            get {
#if PPC
                lock(this){
#endif
                if(this.NeedToInit) {
                    this.Init();
                }

                return this.pBNode;

                
                
#if PPC
                }
#endif

            }
        }

        private static Parallelogram ParallelogramOfLineSeg(Point a, Point b) {
            Point side = 0.5*(b - a);
            return new Parallelogram(a, side, side);
        }

        private Rectangle boundingBox = Rectangle.CreateAnEmptyBox();

        /// <summary>
        /// bounding box of the polyline
        /// </summary>
        public Rectangle BoundingBox {
            get {
#if PPC
                lock(this){
#endif
                if (this.NeedToInit) {
                    this.Init();
                }

                return this.boundingBox;
#if PPC
                }
#endif
            }
        }

        private void Init() {

            this.boundingBox = new Rectangle(this.StartPoint.Point);
            this.count = 1;
            foreach (Point p in this.Skip(1)) {
                this.boundingBox.Add(p);
                this.count++;
            }

            this.CalculatePbNode();

            this.NeedToInit = false;
        }

        private void CalculatePbNode() {
            this.pBNode = new ParallelogramInternalTreeNode(this, ParallelogramNodeOverICurve.DefaultLeafBoxesOffset);
            var parallelograms = new List<Parallelogram>();
            PolylinePoint pp = this.StartPoint;
            int offset = 0;
            while (pp.Next != null) {
                Parallelogram parallelogram = ParallelogramOfLineSeg(pp.Point, pp.Next.Point);
                parallelograms.Add(parallelogram);
                this.pBNode.AddChild(new ParallelogramLeaf(offset, offset + 1, parallelogram, this, 0));
                pp = pp.Next;
                offset++;
            }

            if (this.Closed) {
                Parallelogram parallelogram = ParallelogramOfLineSeg(this.EndPoint.Point, this.StartPoint.Point);
                parallelograms.Add(parallelogram);
                this.pBNode.AddChild(new ParallelogramLeaf(offset, offset + 1, parallelogram, this, 0));
            }

            this.pBNode.Parallelogram = Parallelogram.GetParallelogramOfAGroup(parallelograms);
        }

        /// <summary>
        /// the start of the parameter domain
        /// </summary>
        public double ParStart {
            get { return 0; }
        }

        /// <summary>
        /// the end of the parameter domain
        /// </summary>
        public double ParEnd {
            get { return this.Closed ? this.Count : this.Count - 1; }
        }

        /// <summary>
        /// Returns the trimmed polyline. Does not change this polyline. Reversed start and end if start is less than end.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public ICurve Trim(double start, double end) {
            //this is a very lazy version!
            Curve curve = this.ToCurve();
            curve = (Curve) curve.Trim(start, end);

            return PolylineFromCurve(curve);
        }

        /// <summary>
        /// Returns the trimmed polyline, wrapping around the end if start is greater than end.
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public ICurve TrimWithWrap(double start, double end) {
            Debug.Assert((start < end) || this.Closed, "Polyline must be closed to wrap");

            //this is a very lazy version!
            var curve = (Curve)this.ToCurve().TrimWithWrap(start, end);
            return PolylineFromCurve(curve);
        }

        internal static Polyline PolylineFromCurve(Curve curve) {
            var ret = new Polyline();
            ret.AddPoint(curve.Start);
            foreach (var ls in curve.Segments) {
                ret.AddPoint(ls.End);
            }

            ret.Closed = curve.Start == curve.End;
            return ret;
        }

        /// <summary>
        /// Returns the curved moved by delta
        /// </summary>
        public void Translate(Point delta)
        {
            PolylinePoint polyPoint = this.StartPoint;
            while (polyPoint != null)
            {
                polyPoint.Point += delta;
                polyPoint = polyPoint.Next;
            }

            this.RequireInit();
        }

        /// <summary>
        /// Returns the curved with all points scaled from the original by x and y
        /// </summary>
        /// <param name="xScale"></param>
        /// <param name="yScale"></param>
        /// <returns></returns>
        public ICurve ScaleFromOrigin(double xScale, double yScale)
        {
            var ret = new Polyline();
            PolylinePoint polyPoint = this.StartPoint;
            while (polyPoint != null)
            {
                ret.AddPoint(Point.Scale(xScale, yScale, polyPoint.Point));
                polyPoint = polyPoint.Next;
            }
            ret.Closed = this.Closed;
            return ret;
        }

        internal void AddPoint(double x, double y) {
            this.AddPoint(new Point(x, y));
        }

        internal void PrependPoint(Point p) {
            Debug.Assert(this.EndPoint == null || !ApproximateComparer.Close(p, this.EndPoint.Point));           
            var pp = new PolylinePoint(p) {Polyline = this};
            if (this.StartPoint != null) {
                if (!ApproximateComparer.Close(p, this.StartPoint.Point))
                {
                    this.StartPoint.Prev = pp;
                    pp.Next = this.StartPoint;
                    this.StartPoint = pp;
                }
            } else {
                this.StartPoint = this.EndPoint = pp;
            }
            this.RequireInit();
        }

        ///<summary>
        ///adds a point to the polyline
        ///</summary>
        ///<param name="point"></param>
        public void AddPoint(Point point) {
            var pp = new PolylinePoint(point) {Polyline = this};
            if (this.EndPoint != null) {
                // if (!ApproximateComparer.Close(point, EndPoint.Point)) {
                this.EndPoint.Next = pp;
                    pp.Prev = this.EndPoint;
                this.EndPoint = pp;
               // }
            } else {
                this.StartPoint = this.EndPoint = pp;
            }
            this.RequireInit();
        }

        /// <summary>
        /// this[ParStart]
        /// </summary>
        public Point Start
        {
            get { return this.StartPoint.Point; }
        }

        /// <summary>
        /// this[ParEnd]
        /// </summary>
        public Point End
        {
            get { return this.EndPoint.Point; }
        }

        /// <summary>
        /// this[Reverse[t]]=this[ParEnd+ParStart-t]
        /// </summary>
        /// <returns></returns>
        public ICurve? Reverse()
        {
            return this.ReversePolyline();
        }

        /// <summary>
        /// Offsets the curve in the direction of dir
        /// </summary>
        /// <param name="offset"></param>
        /// <param name="dir"></param>
        /// <returns></returns>
        public ICurve? OffsetCurve(double offset, Point dir) => null;

        /// <summary>
        /// return length of the curve segment [start,end] 
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public double LengthPartial(double start, double end)
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// Get the length of the curve
        /// </summary>
        public double Length
        {
            get {
                double ret = 0;
                if (this.StartPoint != null && this.StartPoint.Next != null) {
                    PolylinePoint p = this.StartPoint.Next;
                    do {
                        ret += (p.Point - p.Prev.Point).Length;
                        p = p.Next;
                    } while (p != null);
                }
                return ret;
            }
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="length"></param>
        /// <returns></returns>
        /// <exception cref="NotImplementedException"></exception>
        public double GetParameterAtLength(double length) {
            throw new NotImplementedException();
        }

        /// <summary>
        /// returns the transformed polyline
        /// </summary>
        /// <param name="transformation"></param>
        /// <returns></returns>
      public ICurve Transform(PlaneTransformation transformation) {
            if (transformation == null) {
                return this;
            }

            var poly = new Polyline {Closed = this.Closed };
            foreach (var p in this)
            {
                poly.AddPoint(transformation*p);
            }
            return poly;
        }

        /// <summary>
        /// returns a parameter t such that the distance between curve[t] and targetPoint is minimal 
        /// and t belongs to the closed segment [low,high]
        /// </summary>
        /// <param name="targetPoint">the point to find the closest point</param>
        /// <param name="high">the upper bound of the parameter</param>
        /// <param name="low">the low bound of the parameter</param>
        /// <returns></returns>
      public double ClosestParameterWithinBounds(Point targetPoint, double low, double high) {
            double ret = 0;
            double dist = Double.MaxValue;
            int offset = 0;
            PolylinePoint pp = this.StartPoint;
            while (pp.Next != null) {
                if (offset <= high && offset + 1 >= low) {
                    var lowLocal = Math.Max(0, low - offset);
                    var highLocal = Math.Min(1, high - offset);
                    var ls = new LineSegment(pp.Point, pp.Next.Point);
                    double t = ls.ClosestParameterWithinBounds(targetPoint, lowLocal, highLocal);
                    Point delta = ls[t] - targetPoint;
                    double newDist = delta*delta;
                    if (newDist < dist) {
                        dist = newDist;
                        ret = t + offset;
                    }
                }
                pp = pp.Next;
                offset++;
            }

            if (this.Closed) {
                if (offset <= high && offset + 1 >= low) {
                    var lowLocal = Math.Max(0, low - offset);
                    var highLocal = Math.Min(1, high - offset);
                    var ls = new LineSegment(this.EndPoint.Point, this.StartPoint.Point);
                    double t = ls.ClosestParameterWithinBounds(targetPoint, lowLocal, highLocal);
                    Point delta = ls[t] - targetPoint;
                    double newDist = delta*delta;
                    if (newDist < dist) {
                        ret = t + offset;
                    }
                }
            }
            return ret;
        }

        /// <summary>
        /// gets the parameter of the closest point
        /// </summary>
        /// <param name="targetPoint"></param>
        /// <returns></returns>
        public double ClosestParameter(Point targetPoint) {
            double ret = 0;
            double dist = Double.MaxValue;
            int offset = 0;
            PolylinePoint pp = this.StartPoint;
            while (pp.Next != null) {
                var ls = new LineSegment(pp.Point, pp.Next.Point);
                double t = ls.ClosestParameter(targetPoint);
                Point delta = ls[t] - targetPoint;
                double newDist = delta*delta;
                if (newDist < dist) {
                    dist = newDist;
                    ret = t + offset;
                }
                pp = pp.Next;
                offset++;
            }

            if (this.Closed) {
                var ls = new LineSegment(this.EndPoint.Point, this.StartPoint.Point);
                double t = ls.ClosestParameter(targetPoint);
                Point delta = ls[t] - targetPoint;
                double newDist = delta*delta;
                if (newDist < dist) {
                    ret = t + offset;
                }
            }
            return ret;
        }

        /// <summary>
        /// clones the curve. 
        /// </summary>
        /// <returns>the cloned curve</returns>
        ICurve ICurve.Clone() {
            var ret = new Polyline();
            foreach (Point p in this) {
                ret.AddPoint(p);
            }

            ret.Closed = this.Closed;
            return ret;
        }

        #endregion

        #region IEnumerable<Point> Members

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=332
        public IEnumerator<Point> GetEnumerator()
        {
#else
        IEnumerator<Point> IEnumerable<Point>.GetEnumerator() {
#endif
            return new PolylineIterator(this);
        }

        #endregion

        #region IEnumerable Members

        IEnumerator IEnumerable.GetEnumerator() {
            return new PolylineIterator(this);
        }

        #endregion

        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Polyline ReversePolyline() {
            var ret = new Polyline();
            PolylinePoint pp = this.EndPoint;
            while (pp.Prev != null) {
                ret.AddPoint(pp.Point);
                pp = pp.Prev;
            }
            ret.AddPoint(this.StartPoint.Point);
            ret.Closed = this.Closed;
            return ret;
        }

        internal PolylinePoint? Next(PolylinePoint a) {
            return a.Next ?? (this.Closed ? this.StartPoint : null);
        }

        internal PolylinePoint? Prev(PolylinePoint a) {
            return a.Prev ?? (this.Closed ? this.EndPoint : null);
        }

        /// <summary>
        /// creates a polyline from a point enumeration
        /// </summary>
        /// <param name="points"></param>
        public Polyline(IEnumerable<Point> points) {
            ValidateArg.IsNotNull(points, "points");
            foreach (var p in points) {
                this.AddPoint(p);
            }
        }

        /// <summary>
        /// creating a polyline from two points
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "b"),
         SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "a")]
        public Polyline(Point a, Point b) {
            this.AddPoint(a);
            this.AddPoint(b);
        }


        /// <summary>
        /// an empty constructor
        /// </summary>
        public Polyline() {}

        ///<summary>
        ///</summary>
        ///<param name="points"></param>
#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=339
        [SharpKit.JavaScript.JsMethod(NativeParams = false)]
#endif
        public Polyline(params Point[] points) : this((IEnumerable<Point>)points) { }

        /// <summary>
        /// true in general for convex polylines
        /// </summary>
        /// <returns></returns>
        internal bool IsClockwise() {
            return Point.GetTriangleOrientation(this.StartPoint.Point, this.StartPoint.Next.Point, this.StartPoint.Next.Next.Point) ==
                   TriangleOrientation.Clockwise;
        }

        internal void RemoveStartPoint() {
            PolylinePoint p = this.StartPoint.Next;
            p.Prev = null;
            this.StartPoint = p;
            this.RequireInit();
        }

        internal void RemoveEndPoint() {
            PolylinePoint p = this.EndPoint.Prev;
            p.Next = null;
            this.EndPoint = p;
            this.RequireInit();
        }

        /// <summary>
        /// Returns the point location value. The assumption is that the polyline goes clockwise and is closed and convex.
        /// </summary>
        /// <param name="point">Point to find.</param>
        /// <param name="witness">if the point belongs to the boundary then witness is
        ///         the first point of the boundary segment containing p </param>
        /// <returns></returns>
        internal PointLocation GetPointLocation(Point point, out PolylinePoint witness) {
            Debug.Assert(this.Closed && this.IsClockwise());
            witness = null;

            foreach (PolylinePoint polyPoint in this.PolylinePoints) {
                PolylinePoint secondPoint = this.Next(polyPoint);
                TriangleOrientation triangleOrientation = Point.GetTriangleOrientation(point, polyPoint.Point,
                                                                                       secondPoint.Point);
                if (triangleOrientation == TriangleOrientation.Counterclockwise) {
                    return PointLocation.Outside;
                }

                if (triangleOrientation == TriangleOrientation.Collinear) {
                    if ((point - polyPoint.Point)*(secondPoint.Point - point) >= 0) {
                        witness = polyPoint;
                        return PointLocation.Boundary;
                    }
                }
            }

            return PointLocation.Inside;
        }

        /// <summary>
        /// Returns the point location value and the edge containing it if it belongs to a boundary. 
        /// The assumption is that the polyline goes clockwise and is closed and convex.
        /// </summary>
        /// <param name="point">Point to find</param>
        /// <param name="edgeStart">The starting point of the boundary hit, if any</param>
        /// <param name="edgeEnd">The ending point of the boundary hit, if any</param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1021:AvoidOutParameters", MessageId = "2#")]
        [SuppressMessage("Microsoft.Design", "CA1021:AvoidOutParameters", MessageId = "1#")]
        public PointLocation GetPointLocation(Point point, out Point edgeStart, out Point edgeEnd) {
            PolylinePoint start;
            edgeStart = new Point();
            edgeEnd = new Point();
            PointLocation loc = this.GetPointLocation(point, out start);
            if (PointLocation.Boundary == loc) {
                edgeStart = start.Point;
                edgeEnd = start.NextOnPolyline.Point;
            }
            return loc;
        }

        /// <summary>
        /// shift the given polyline by delta
        /// </summary>
        /// <param name="delta"></param>
        public void Shift(Point delta) {
            for (PolylinePoint pp = this.StartPoint; pp != null; pp = pp.Next) {
                pp.Point += delta;
            }
        }

        #region ICurve Members

        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double Curvature(double t) {
            throw new NotImplementedException();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double CurvatureDerivative(double t) {
            throw new NotImplementedException();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double CurvatureSecondDerivative(double t) {
            throw new NotImplementedException();
        }

        #endregion

        internal void AddRangeOfPoints(IEnumerable<Point> points){
            foreach (var point in points) {
                this.AddPoint(point);
            }
        }
    }
}
