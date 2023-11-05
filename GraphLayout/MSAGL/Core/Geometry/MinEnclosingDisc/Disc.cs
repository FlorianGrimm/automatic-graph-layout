using System;
using System.Linq;
using System.Diagnostics;

namespace Microsoft.Msagl.Core.Geometry
{
    /// <summary>
    /// Disc for use in Minimum Enclosing Disc computation
    /// </summary>
    public class Disc
    {
        /// <summary>
        /// disc centre
        /// </summary>
        private Point c;
        /// <summary>
        /// disc centre
        /// </summary>
        public Point Center
        {
            get { return this.c; }
        }

        /// <summary>
        /// radius
        /// </summary>
        private double r;
        /// <summary>
        /// Radius of disc
        /// </summary>
        public double Radius
        {
            get { return this.r; }
        }

        private double r2;
        /// <summary>
        /// squared distance from the centre of this disc to point
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public double Distance2(Point point)
        {
            double dx = this.c.X - point.X, dy = this.c.Y - point.Y;
            return dx * dx + dy * dy;
        }
        /// <summary>
        /// Test if point is contained in this disc
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool Contains(Point point)
        {
            return this.Distance2(point) - 1e-7 <= this.r2;
        }
        /// <summary>
        /// test if all specified points (apart from the except list) are contained in this disc
        /// </summary>
        /// <param name="points">points to test for containment</param>
        /// <param name="except">short list of exclusions</param>
        /// <returns>true if all points are contained in the disc</returns>
        public bool Contains(Point[] points, int[] except)
        {
            ValidateArg.IsNotNull(points, "points");
            for (int i = 0; i < points.Length; ++i)
            {
                if (!except.Contains(i) && !this.Contains(points[i]))
                {
                    return false;
                }
            }
            return true;
        }
        /// <summary>
        /// create a zero radius disc centred at center
        /// </summary>
        /// <param name="center">center of disc</param>
        public Disc(Point center)
        {
            this.c = center;
            this.r2 = this.r = 0;
        }

        /// <summary>
        /// find the point mid-way between two points
        /// </summary>
        /// <param name="startPoint"></param>
        /// <param name="endPoint"></param>
        private static Point midPoint(Point startPoint, Point endPoint)
        {
            return new Point((endPoint.X + startPoint.X) / 2.0, (endPoint.Y + startPoint.Y) / 2.0);
        }
        /// <summary>
        /// Create the smallest possible disc with the specified points on the boundary
        /// </summary>
        /// <param name="firstBoundaryPoint"></param>
        /// <param name="secondBoundaryPoint"></param>
        public Disc(Point firstBoundaryPoint, Point secondBoundaryPoint)
        {
            this.c = midPoint(firstBoundaryPoint, secondBoundaryPoint);
            this.r2 = this.Distance2(firstBoundaryPoint);
            this.r = Math.Sqrt(this.r2);
            Debug.Assert(this.OnBoundary(firstBoundaryPoint));
            Debug.Assert(this.OnBoundary(secondBoundaryPoint));
        }
        /// <summary>
        /// test if a point lies on (within a small delta of) the boundary of this disc
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool OnBoundary(Point point)
        {
            double d = this.Distance2(point);
            return Math.Abs(d - this.r2) / (d + this.r2) < 1e-5;
        }

        /// <summary>
        /// computes the centre of the disc with the 3 specified points on the boundary
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <returns></returns>
        private static Point centre(Point p1, Point p2, Point p3)
        {
            Debug.Assert(p2.X != p1.X);
            Debug.Assert(p3.X != p2.X);
            double ma, mb;
            ma = (p2.Y - p1.Y) / (p2.X - p1.X);
            mb = (p3.Y - p2.Y) / (p3.X - p2.X);
            Debug.Assert(mb != ma); // collinear points not allowed
            Point c = new Point();
            c.X = ma * mb * (p1.Y - p3.Y) + mb * (p1.X + p2.X) - ma * (p2.X + p3.X);
            c.X /= 2.0 * (mb - ma);
            if (Math.Abs(ma) > Math.Abs(mb))
            {
                c.Y = (p1.Y + p2.Y) / 2.0 - (c.X - (p1.X + p2.X) / 2.0) / ma;
            }
            else
            {
                c.Y = (p2.Y + p3.Y) / 2.0 - (c.X - (p2.X + p3.X) / 2.0) / mb;
            }
            return c;
        }
        /// <summary>
        /// if the area of the triangle formed by the 3 points is 0 then the points are collinear
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "p")]
        public static bool Collinear(Point p1, Point p2, Point p3)
        {
            return p1.X * (p2.Y - p3.Y) + p2.X * (p3.Y - p1.Y) + p3.X * (p1.Y - p2.Y) == 0;
        }
        /// <summary>
        /// Create a disc with the specified points on the boundary
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "p")]
        public Disc(Point p1, Point p2, Point p3)
        {
            if (Collinear(p1, p2, p3))
            {
                Point LL = new Point(
                        Math.Min(p1.X, Math.Min(p2.X, p3.X)),
                        Math.Min(p1.Y, Math.Max(p2.Y, p3.Y))),
                      UR = new Point(
                        Math.Max(p1.X, Math.Max(p2.X, p3.X)),
                        Math.Max(p1.Y, Math.Max(p2.Y, p3.Y)));
                this.c = midPoint(LL, UR);
                this.r2 = this.Distance2(UR);
                this.r = Math.Sqrt(this.r2);
            }
            else
            {
                double dx12 = p2.X - p1.X, dx23 = p3.X - p2.X, dx13 = p3.X - p1.X;
                if (dx12 != 0)
                {
                    if (dx23 != 0)
                    {
                        this.c = centre(p1, p2, p3);
                    }
                    else
                    {
                        Debug.Assert(dx13 != 0);
                        this.c = centre(p2, p1, p3);
                    }
                }
                else
                {
                    Debug.Assert(dx23 != 0); // because points are not collinear
                    this.c = centre(p2, p3, p1);
                }
                this.r2 = this.Distance2(p1);
                this.r = Math.Sqrt(this.r2);
                Debug.Assert(this.OnBoundary(p1));
                Debug.Assert(this.OnBoundary(p2));
                Debug.Assert(this.OnBoundary(p3));
            }
        }
    }
}
