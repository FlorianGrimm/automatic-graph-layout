using System;
using System.Diagnostics;

namespace Microsoft.Msagl.Core.Geometry.Curves {
    /// <summary>
    /// A rectanglular curve with rounded corners
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class RoundedRect : ICurve {
        private Curve _Curve;

        ///<summary>
        /// underlying curve
        ///</summary>
        public Curve Curve => this._Curve;

        /// <summary>
        /// The horizontal radius of the corner ellipse segments
        /// </summary>
        public double RadiusX { get; set; }

        /// <summary>
        /// The vertical radius of the corner ellipse segments
        /// </summary>
        public double RadiusY { get; set; }

        private RoundedRect(Curve curve, double radiusX, double radiusY) {
            this._Curve = curve;
            this.RadiusX = radiusX;
            this.RadiusY = radiusY;
        }

        /// <summary>
        /// Create a rounded rectangle geometry
        /// </summary>
        /// <param name="bounds">rounded rectangle will fit these bounds</param>
        /// <param name="radiusX">horizontal radius of the corner ellipse segments</param>
        /// <param name="radiusY">vertical radius of the corner ellipse segments</param>
        public RoundedRect(Rectangle bounds, double radiusX, double radiusY) {
            this.RadiusX = radiusX;
            this.RadiusY = radiusY;
            this._Curve = new Curve(8);
            CurveFactory.CreateRectangleWithRoundedCorners(this._Curve,
                bounds.Width, bounds.Height, radiusX, radiusY, bounds.Center);
        }

        /// <summary>
        /// Create a new RoundedRect with the same CornerRadius inside the target bounds
        /// </summary>
        /// <param name="target">target bounds</param>
        /// <returns>new RoundedRect with the same CornerRadius inside the target bounds</returns>
        public ICurve FitTo(Rectangle target) {
            if (ApproximateComparer.Close(target, this.BoundingBox, ApproximateComparer.UserDefinedTolerance)) {
                return this.Clone();
            }

            return new RoundedRect(target, this.RadiusX, this.RadiusY);
        }

        /// <summary>
        /// Returns the point on the curve corresponding to parameter t
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public Point this[double t] {
            get { return this._Curve[t]; }
        }

        /// <summary>
        /// first derivative at t
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point Derivative(double t) {
            return this._Curve.Derivative(t);
        }

        /// <summary>
        /// second derivative
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public Point SecondDerivative(double t) {
            return this._Curve.SecondDerivative(t);
        }

        /// <summary>
        /// third derivative
        /// </summary>
        /// <param name="t">the parameter of the derivative</param>
        /// <returns></returns>
        public Point ThirdDerivative(double t) {
            return this._Curve.ThirdDerivative(t);
        }

        /// <summary>
        /// A tree of ParallelogramNodes covering the curve. 
        /// This tree is used in curve intersections routines.
        /// </summary>
        /// <value></value>
        public ParallelogramNodeOverICurve ParallelogramNodeOverICurve {
            get { return this._Curve.ParallelogramNodeOverICurve; }
        }

        private bool dirtyBounds = true;
        private Rectangle cachedBounds;

        /// <summary>
        /// XY bounding box of the curve
        /// </summary>
        public Rectangle BoundingBox {
            get {
                if (this.dirtyBounds) {
                    this.cachedBounds = this._Curve.BoundingBox;
                    this.dirtyBounds = false;
                }
                return this.cachedBounds;
            }
        }

        /// <summary>
        /// the start of the parameter domain
        /// </summary>
        public double ParStart {
            get { return this._Curve.ParStart; }
        }

        /// <summary>
        /// the end of the parameter domain
        /// </summary>
        public double ParEnd {
            get { return this._Curve.ParEnd; }
        }

        /// <summary>
        /// returns the trim curve
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public ICurve Trim(double start, double end) {
            return this._Curve.Trim(start, end);
        }

        /// <summary>
        /// Returns the trimmed curve, wrapping around the end if start is greater than end.
        /// </summary>
        /// <param name="start">The starting parameter</param>
        /// <param name="end">The ending parameter</param>
        /// <returns>The trimmed curve</returns>
        public ICurve TrimWithWrap(double start, double end) {
            return this._Curve.TrimWithWrap(start, end);
        }

        /// <summary>
        /// Returns the curved moved by delta
        /// </summary>
        public void Translate(Point delta) {
            this._Curve.Translate(delta);
            this.dirtyBounds = true;
        }

        /// <summary>
        /// Returns the curved with all points scaled from the original by x and y
        /// </summary>
        /// <returns></returns>
        public ICurve ScaleFromOrigin(double xScale, double yScale) {
            var bounds = this._Curve.BoundingBox;
            var lt = Point.Scale(xScale, yScale, bounds.LeftTop);
            var rb = Point.Scale(xScale, yScale, bounds.RightBottom);
            return new RoundedRect(new Rectangle(lt, rb), this.RadiusX, this.RadiusY);
        }

        /// <summary>
        /// this[ParStart]
        /// </summary>
        public Point Start {
            get { return this._Curve.Start; }
        }

        /// <summary>
        /// this[ParEnd]
        /// </summary>
        public Point End {
            get { return this._Curve.End; }
        }

        /// <summary>
        /// this[Reverse[t]]=this[ParEnd+ParStart-t] - not implemented
        /// </summary>
        /// <returns></returns>
        public ICurve? Reverse() => default;

        /// <summary>
        /// Offsets the curve in the direction of dir
        /// </summary>
        /// <param name="offset"></param>
        /// <param name="dir"></param>
        /// <returns></returns>
        public ICurve? OffsetCurve(double offset, Point dir) {
            return this._Curve.OffsetCurve(offset, dir);
        }

        /// <summary>
        /// return length of the curve segment [start,end] 
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        /// <returns></returns>
        public double LengthPartial(double start, double end) {
            return this._Curve.LengthPartial(start, end);
        }

        /// <summary>
        /// Get the length of the curve
        /// </summary>
        public double Length {
            get { return this._Curve.Length; }
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="length"></param>
        /// <returns></returns>
        public double GetParameterAtLength(double length) {
            return this._Curve.GetParameterAtLength(length);
        }

        /// <summary>
        /// Return the transformed curve
        /// </summary>
        /// <param name="transformation"></param>
        /// <returns>the transformed curve</returns>
        public ICurve Transform(PlaneTransformation transformation) {
            var bounds = this._Curve.BoundingBox;
            var lt = transformation * bounds.LeftTop;
            var rb = transformation * bounds.RightBottom;
            var transBounds = new Rectangle(lt, rb);
            return new RoundedRect(transBounds, this.RadiusX, this.RadiusY);
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
            return this.Curve.ClosestParameterWithinBounds(targetPoint, low, high);
        }

        /// <summary>
        /// returns a parameter t such that the distance between curve[t] and a is minimal
        /// </summary>
        /// <param name="targetPoint"></param>
        /// <returns></returns>
        public double ClosestParameter(Point targetPoint) {
            return this._Curve.ClosestParameter(targetPoint);
        }

        /// <summary>
        /// clones the curve. 
        /// </summary>
        /// <returns>the cloned curve</returns>
        public ICurve Clone()
            => new RoundedRect(
                (Curve)this._Curve.Clone(),
                this.RadiusX,
                this.RadiusY
                );

        /// <summary>
        /// The left derivative at t. 
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point LeftDerivative(double t) {
            return this._Curve.LeftDerivative(t);
        }

        /// <summary>
        /// the right derivative at t
        /// </summary>
        /// <param name="t">the parameter where the derivative is calculated</param>
        /// <returns></returns>
        public Point RightDerivative(double t) {
            return this._Curve.RightDerivative(t);
        }

        /// <summary>
        /// the signed curvature of the segment at t
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double Curvature(double t) {
            return this._Curve.Curvature(t);
        }

        /// <summary>
        /// the derivative of the curvature at t
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double CurvatureDerivative(double t) {
            return this._Curve.CurvatureDerivative(t);
        }

        /// <summary>
        /// the derivative of CurvatureDerivative
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double CurvatureSecondDerivative(double t) {
            return this._Curve.CurvatureSecondDerivative(t);
        }
    }
}