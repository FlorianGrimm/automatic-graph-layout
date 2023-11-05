using System;

namespace Microsoft.Msagl.Core.Geometry {
    /// <summary>
    /// A class for keeping polyline points in a double linked list
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class CornerSite{
        /// <summary>
        /// the coeffiecient used to calculate the first and the second control points of the 
        /// Bezier segment for the fillet at the site
        /// </summary>
        private double previouisBezierCoefficient = 0.5;
        /// <summary>
        /// used to calculate the first control points: the formula is kPrev * a + (1 - kPrev) * b
        /// </summary>
        public double PreviousBezierSegmentFitCoefficient {
            get { return this.previouisBezierCoefficient; }
            set { this.previouisBezierCoefficient = value; }
        }

        /// <summary>
        /// the coeffiecient used to calculate the third and the fourth control points of the 
        /// Bezier segment for the fillet at the site
        /// </summary>
        private double nextBezierCoefficient = 0.5;
        /// <summary>
        /// the coefficient tells how tight the segment fits to the segment after the site; the formula is kNext * c + (1 - kNext) * b
        /// </summary>
        public double NextBezierSegmentFitCoefficient {
            get { return this.nextBezierCoefficient; }
            set { this.nextBezierCoefficient = value; }
        }

        private double previousTangentCoefficient=1.0/3;

        ///<summary>
        ///used to calculate the second control point
        ///</summary>
        public double PreviousTangentCoefficient {
            get { return this.previousTangentCoefficient; }
            set { this.previousTangentCoefficient = value; }
        }

        private double nextTangentCoefficient = 1.0 / 3;

        ///<summary>
        ///used to calculate the third control point
        ///</summary>
        public double NextTangentCoefficient
        {
            get { return this.nextTangentCoefficient; }
            set { this.nextTangentCoefficient = value; }
        }

        //   internal double par;
        private Point point;

        /// <summary>
        /// gets the site point
        /// </summary>
        public Point Point {
            get { return this.point; }
            set { this.point = value; }
        }

        private CornerSite prev;
/// <summary>
/// gets the previous site
/// </summary>
		public CornerSite Previous
		{
            get { return this.prev; }
            set { this.prev = value; }
        }

        private CornerSite next;
/// <summary>
/// gets the next site
/// </summary>
		public CornerSite Next
		{
            get { return this.next; }
            set { this.next = value; }
        }
        internal CornerSite() { }
        /// <summary>
        /// the constructor
        /// </summary>
        /// <param name="sitePoint"></param>
        public CornerSite(Point sitePoint) {
            this.point = sitePoint;
        }
        /// <summary>
        /// a constructor
        /// </summary>
        /// <param name="previousSite"></param>
        /// <param name="sitePoint"></param>
		public CornerSite(CornerSite previousSite, Point sitePoint )
		{
            ValidateArg.IsNotNull(previousSite, "pr");
            this.point = sitePoint;
            this.prev = previousSite;
            previousSite.next = this;
        }
        /// <summary>
        /// a constructor
        /// </summary>
        /// <param name="previousSite"></param>
        /// <param name="sitePoint"></param>
        /// <param name="nextSite"></param>
		public CornerSite(CornerSite previousSite, Point sitePoint, CornerSite nextSite )
		{
            ValidateArg.IsNotNull(previousSite, "pr");
            ValidateArg.IsNotNull(nextSite, "ne");
            this.prev = previousSite;
            this.point = sitePoint;
            this.next = nextSite;

            previousSite.next = this;
            this.next.prev = this;
        }
        
        internal double Turn {
            get {
                if (this.Next == null || this.Previous == null) {
                    return 0;
                }

                return Point.SignedDoubledTriangleArea(this.Previous.Point, this.Point, this.Next.Point);
            }
        }

        internal CornerSite Clone() {
            CornerSite s = new CornerSite();
            s.PreviousBezierSegmentFitCoefficient = this.PreviousBezierSegmentFitCoefficient;
            s.Point = this.Point;
            return s;
        }
/// <summary>
/// 
/// </summary>
/// <returns></returns>
        public override string ToString() {
            return this.Point.ToString();
        }

    }
}
