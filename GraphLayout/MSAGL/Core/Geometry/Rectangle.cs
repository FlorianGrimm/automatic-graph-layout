using System;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Geometry{
    /// <summary>
    /// Just a rectangle
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
#pragma warning disable CS0660 // Type defines operator == or operator != but does not override Object.Equals(object o)
#pragma warning disable CS0661 // Type defines operator == or operator != but does not override Object.GetHashCode()
    public struct Rectangle : IRectangle<Point> {
#pragma warning restore CS0661 // Type defines operator == or operator != but does not override Object.GetHashCode()
#pragma warning restore CS0660 // Type defines operator == or operator != but does not override Object.Equals(object o)
        /// <summary>
        /// shows min and max coordinates of corners
        /// </summary>
        /// <returns>string leftbottom, righttop</returns>
        public override string ToString() {
            return "(" + this.LeftBottom + " " + this.RightTop + ")";
        }

        private double left;
        private double right;
        private double top;
        private double bottom;

        /// <summary>
        /// returns true if r intersect this rectangle
        /// </summary>
        /// <param name="rectangle"></param>
        /// <returns></returns>
        public bool Intersects(Rectangle rectangle) {
            return this.IntersectsOnX(rectangle) && this.IntersectsOnY(rectangle);
        }

        /// <summary>
        /// intersection (possibly empty) of rectangles
        /// </summary>
        /// <param name="rectangle"></param>
        /// <returns></returns>
        public Rectangle Intersection(Rectangle rectangle) {
            Rectangle intersection = new Rectangle();
            if (!this.Intersects(rectangle)) {
                intersection.SetToEmpty();
                return intersection;
            }
            double l = Math.Max(this.Left, rectangle.Left);
            double r = Math.Min(this.Right, rectangle.Right);
            double b = Math.Max(this.Bottom, rectangle.Bottom);
            double t = Math.Min(this.Top, rectangle.Top);
            return new Rectangle(l, b, r, t);
        }

        /// <summary>
        /// the center of the bounding box
        /// </summary>
        public Point Center {
            get { return 0.5 * (this.LeftTop + this.RightBottom); }
            set {
                Point shift = value - this.Center;
                this.LeftTop += shift;
                this.RightBottom += shift;
            }
        }

        internal bool IntersectsOnY(Rectangle r) {
            if (r.Bottom > this.top + ApproximateComparer.DistanceEpsilon) {
                return false;
            }

            if (r.Top < this.bottom - ApproximateComparer.DistanceEpsilon) {
                return false;
            }

            return true;
        }

        internal bool IntersectsOnX(Rectangle r) {
            if (r.Left > this.right + ApproximateComparer.DistanceEpsilon) {
                return false;
            }

            if (r.Right < this.left - ApproximateComparer.DistanceEpsilon) {
                return false;
            }

            return true;
        }


        /// <summary>
        /// creates an empty rectangle
        /// </summary>
        /// <returns></returns>
        public static Rectangle CreateAnEmptyBox() {
            return new Rectangle(0, 0, new Point(-1, -1));
        }

        /// <summary>
        /// the left of the rectangle
        /// </summary>
        public double Left {
            get { return this.left; }
            set { this.left = value; }
        }

        /// <summary>
        /// the right of the rectangle
        /// </summary>
        public double Right {
            get { return this.right; }
            set { this.right = value; }
        }

        /// <summary>
        /// the top of the rectangle
        /// </summary>
        public double Top {
            get { return this.top; }
            set { this.top = value; }
        }

        /// <summary>
        /// the bottom of the rectangle
        /// </summary>
        public double Bottom {
            get { return this.bottom; }
            set { this.bottom = value; }
        }

        /// <summary>
        /// the left bottom corner
        /// </summary>
        public Point LeftBottom {
            get { return new Point(this.Left, this.Bottom); }
        }

        /// <summary>
        /// the right top corner
        /// </summary>
        public Point RightTop {
            get { return new Point(this.Right, this.Top); }
        }

        /// <summary>
        /// the left top corner
        /// </summary>
        public Point LeftTop {
            get { return new Point(this.left, this.top); }
            set {
                this.left = value.X;
                this.top = value.Y;
            }
        }

        /// <summary>
        /// the right bottom corner
        /// </summary>
        public Point RightBottom {
            get { return new Point(this.right, this.bottom); }
            set {
                this.right = value.X;
                this.Bottom = value.Y;
            }
        }

        /// <summary>
        /// create a box of two points
        /// </summary>
        /// <param name="point0"></param>
        /// <param name="point1"></param>
        public Rectangle(Point point0, Point point1) {
            this.left = this.right = point0.X;
            this.top = this.bottom = point0.Y;
            this.Add(point1);
        }

        /// <summary>
        /// create rectangle from a point
        /// </summary>
        /// <param name="point"></param>
        public Rectangle(Point point) {
            this.left = this.right = point.X;
            this.top = this.bottom = point.Y;
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="left">left</param>
        /// <param name="bottom">bottom</param>
        /// <param name="sizeF">size</param>
        public Rectangle(double left, double bottom, Point sizeF) {
            this.left = left;
            this.bottom = bottom;
            this.right = left + sizeF.X;
            this.top = bottom + sizeF.Y;
        }

        /// <summary>
        /// create a box on points (x0,y0), (x1,y1)
        /// </summary>
        /// <param name="x0"></param>
        /// <param name="y0"></param>
        /// <param name="x1"></param>
        /// <param name="y1"></param>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "y"),
         SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "x")]
        public Rectangle(double x0, double y0, double x1, double y1) {
            this.left = this.right = x0;
            this.top = this.bottom = y0;
            this.Add(new Point(x1, y1));
        }

        /// <summary>
        /// Create rectangle that is the bounding box of the given points
        /// </summary>
        public Rectangle(IEnumerable<Point> points)
            : this(0, 0, new Point(-1, -1)) {
            ValidateArg.IsNotNull(points, "points");
            foreach (var p in points) {
                this.Add(p);
            }
        }

        /// <summary>
        /// Create rectangle that is the bounding box of the given Rectangles
        /// </summary>
        public Rectangle(IEnumerable<Rectangle> rectangles)
            : this(0, 0, new Point(-1, -1)) {
            ValidateArg.IsNotNull(rectangles, "rectangles");
            foreach (var r in rectangles) {
                this.Add(r);
            }
        }

        /// <summary>
        /// the width of the rectangle
        /// </summary>
        public double Width {
            get { return this.right - this.left; }
            set {
                double hw = value / 2.0f;
                double cx = (this.left + this.right) / 2.0f;
                this.left = cx - hw;
                this.right = cx + hw;
            }
        }

        /// <summary>
        /// returns true if the rectangle has negative width
        /// </summary>
        public bool IsEmpty {
            get { return this.Width < 0; }
        }

        /// <summary>
        /// makes the rectangle empty
        /// </summary>
        public void SetToEmpty() {
            this.Left = 0;
            this.Right = -1;
        }

        /// <summary>
        /// Height of the rectangle
        /// </summary>
        public double Height {
            get { return this.top - this.bottom; }
            set {
                double hw = value / 2.0f;
                double cx = (this.top + this.bottom) / 2.0f;
                this.top = cx + hw;
                this.bottom = cx - hw;
            }
        }

        /// <summary>
        /// rectangle containing both a and side1
        /// </summary>
        /// <param name="rectangle0"></param>
        /// <param name="rectangle1"></param>
        public Rectangle(Rectangle rectangle0, Rectangle rectangle1) {
            this.left = rectangle0.left;
            this.right = rectangle0.right;
            this.top = rectangle0.top;
            this.bottom = rectangle0.bottom;

            this.Add(rectangle1);
        }

        /// <summary>
        /// contains with padding
        /// </summary>
        /// <param name="point"></param>
        /// <param name="padding"></param>
        /// <returns></returns>
        public bool Contains(Point point, double padding) {
            return this.left - padding - ApproximateComparer.DistanceEpsilon <= point.X && point.X <= this.right + padding + ApproximateComparer.DistanceEpsilon &&
                this.bottom - padding - ApproximateComparer.DistanceEpsilon <= point.Y && point.Y <= this.top + padding + ApproximateComparer.DistanceEpsilon;
        }

        /// <summary>
        /// Rectangle area
        /// </summary>
        public double Area {
            get { return (this.right - this.left) * (this.top - this.bottom); }
        }

        /// <summary>
        /// adding a point to the rectangle
        /// </summary>
        /// <param name="point"></param>
        public void Add(Point point) {
            if (!this.IsEmpty) {
                if (this.left > point.X) {
                    this.left = point.X;
                }

                if (this.top < point.Y) {
                    this.top = point.Y;
                }

                if (this.right < point.X) {
                    this.right = point.X;
                }

                if (this.bottom > point.Y) {
                    this.bottom = point.Y;
                }
            }
            else {
                this.left = this.right = point.X;
                this.top = this.bottom = point.Y;
            }
        }

        /// <summary>
        /// extend the box to keep the point.
        /// Assume here that the box is initialized correctly
        /// </summary>
        /// <param name="point"></param>
        /// <returns>true if the box has been extended</returns>
        public bool AddWithCheck(Point point) {
            bool wider;
            if (wider = (point.X < this.left)) {
                this.left = point.X;
            } else if (wider = (this.right < point.X)) {
                this.right = point.X;
            }

            bool higher;

            if (higher = (point.Y > this.top)) {
                this.top = point.Y;
            } else if (higher = (this.bottom > point.Y)) {
                this.bottom = point.Y;
            }

            return wider || higher;
        }

        /// <summary>
        /// adding rectangle
        /// </summary>
        /// <param name="rectangle"></param>
        public void Add(Rectangle rectangle) {
            this.Add(rectangle.LeftTop);
            this.Add(rectangle.RightBottom);
        }
        public void Add(IRectangle<Point> r) {
            var rectangle = (Rectangle)r;
            this.Add(rectangle.LeftTop);
            this.Add(rectangle.RightBottom);
        }
        /// <summary>
        /// override ==
        /// </summary>
        /// <param name="rectangle0"></param>
        /// <param name="rectangle1"></param>
        /// <returns></returns>
        public static bool operator ==(Rectangle rectangle0, Rectangle rectangle1) {
            return rectangle0.Equals(rectangle1);
        }

        /// <summary>
        /// overrides !=
        /// </summary>
        /// <param name="rectangle0"></param>
        /// <param name="rectangle1"></param>
        /// <returns></returns>
        public static bool operator !=(Rectangle rectangle0, Rectangle rectangle1) {
            return !rectangle0.Equals(rectangle1);
        }

        /// <summary>
        /// Return copy of specified rectangle translated by the specified delta
        /// </summary>
        /// <param name="rectangle">source to copy and translate</param>
        /// <param name="delta">translation vector</param>
        /// <returns>copy of specified rectangle translated by the specified delta</returns>
        public static Rectangle Translate(Rectangle rectangle, Point delta) {
            rectangle.Center += delta;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=369 there are no structs in js
            return rectangle.Clone();
#else
            return rectangle;
#endif
        }

        /// <summary>
        /// returns true if the rectangle contains the point
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool Contains(Point point) {
            return (ApproximateComparer.Compare(this.left, point.X) <= 0) && (ApproximateComparer.Compare(this.right, point.X) >= 0)
                   && (ApproximateComparer.Compare(this.top, point.Y) >= 0) && (ApproximateComparer.Compare(this.bottom, point.Y) <= 0);
        }

        /// <summary>
        /// returns true if this rectangle completely contains the specified rectangle
        /// </summary>
        /// <param name="rect"></param>
        /// <returns></returns>
        public bool Contains(Rectangle rect) {
            return this.Contains(rect.LeftTop) && this.Contains(rect.RightBottom);
        }


        /// <summary>
        /// return the length of the diagonal 
        /// </summary>
        /// <returns></returns>
        public double Diagonal {
            get { return Math.Sqrt(this.Width * this.Width + this.Height * this.Height); }
        }

        /// <summary>
        /// pad the rectangle horizontally by the given padding
        /// </summary>
        /// <param name="padding"></param>
        public void PadWidth(double padding) {
            this.Left -= padding;
            this.Right += padding;
        }

        /// <summary>
        /// pad the rectangle vertically by the given padding
        /// </summary>
        /// <param name="padding"></param>
        public void PadHeight(double padding) {
            this.Top += padding;
            this.Bottom -= padding;
        }

        /// <summary>
        /// pad the rectangle by the given padding
        /// </summary>
        /// <param name="padding"></param>
        public Rectangle Pad(double padding) {
            if (padding < -this.Width / 2) {
                padding = -this.Width / 2;
            }

            if (padding < -this.Height / 2) {
                padding = -this.Height / 2;
            }

            this.PadWidth(padding);
            this.PadHeight(padding);
            return this;
        }

        /// <summary>
        /// Pad the rectangle by the given amount on each side
        /// </summary>
        /// <param name="left"></param>
        /// <param name="bottom"></param>
        /// <param name="right"></param>
        /// <param name="top"></param>
        public void Pad(double left, double bottom, double right, double top) {
            this.Left -= left;
            this.Right += right;
            this.Bottom -= bottom;
            this.Top += top;
        }

        /// <summary>
        /// Returns the intersection of two rectangles.
        /// </summary>
        /// <param name="rect1"></param>
        /// <param name="rect2"></param>
        /// <returns></returns>
        public static Rectangle Intersect(Rectangle rect1, Rectangle rect2) {
            if (rect1.Intersects(rect2)) {
                return new Rectangle(new Point(Math.Max(rect1.Left, rect2.Left), Math.Max(rect1.Bottom, rect2.Bottom)),
                    new Point(Math.Min(rect1.Right, rect2.Right), Math.Min(rect1.Top, rect2.Top)));
            }

            return Rectangle.CreateAnEmptyBox();
        }

        ///<summary>
        ///</summary>
        ///<returns></returns>
        public Polyline Perimeter() {
            var poly = new Polyline();
            poly.AddPoint(this.LeftTop);
            poly.AddPoint(this.RightTop);
            poly.AddPoint(this.RightBottom);
            poly.AddPoint(this.LeftBottom);
            poly.Closed = true;
            return poly;
        }

        ///<summary>
        ///</summary>
        ///<param name="scale"></param>
        public void ScaleAroundCenter(double scale) {
            this.Width = this.Width * scale;
            this.Height = this.Height * scale;
        }

        internal Rectangle Clone() {
            return new Rectangle(this.LeftTop, this.RightBottom);
        }



        /// <summary>
        /// gets or sets the Size
        /// </summary>
        public Size Size {
            get { return new Size(this.Width, this.Height); }
            set {
                this.Width = value.Width;
                this.Height = value.Height;
            }
        }
        /// <summary>
        /// constructor with Size and center
        /// </summary>
        /// <param name="size"></param>
        /// <param name="center"></param>
        public Rectangle(Size size, Point center) {
            var w = size.Width / 2;
            this.left = center.X - w;
            this.right = center.X + w;
            var h = size.Height / 2;
            this.bottom = center.Y - h;
            this.top = center.Y + h;
        }

        /// <summary>
        /// adding a point with a Size
        /// </summary>
        /// <param name="size"></param>
        /// <param name="point"></param>
        public void Add(Size size, Point point) {
            var w = size.Width / 2;
            var h = size.Height / 2;

            this.Add(new Point(point.X - w, point.Y - h));
            this.Add(new Point(point.X + w, point.Y - h));
            this.Add(new Point(point.X - w, point.Y + h));
            this.Add(new Point(point.X + w, point.Y + h));
        }


        bool IRectangle<Point>.Contains(IRectangle<Point> rect) {
            return this.Contains((Rectangle)rect);
        }

        IRectangle<Point> IRectangle<Point>.Intersection(IRectangle<Point> rectangle) {
            return this.Intersection((Rectangle)rectangle);
        }

        bool IRectangle<Point>.Intersects(IRectangle<Point> rectangle) {
            return this.Intersects((Rectangle)rectangle);
        }

        public IRectangle<Point> Unite(IRectangle<Point> rectangle) {
            return new Rectangle(this, (Rectangle)rectangle);
        }

        double IRectangle<Point>.Area { get { return this.Area; } }
    }
}