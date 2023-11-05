using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Geometry {
    /// <summary>
    /// Clusters can (optionally) have a rectangular border which is respected by overlap avoidance.
    /// Currently, this is controlled by FastIncrementalLayoutSettings.RectangularClusters.
    /// If FastIncrementalLayoutSettings.RectangularClusters is true, then the 
    /// FastIncrementalLayout constructor will create a RectangularBoundary in each cluster.
    /// Otherwise it will be null.
    /// </summary>
    public class RectangularClusterBoundary {
        /// <summary>
        /// Set margins to zero which also initializes other members.
        /// </summary>
        public RectangularClusterBoundary() {
            this.LeftBorderInfo = new BorderInfo(0.0);
            this.RightBorderInfo = new BorderInfo(0.0);
            this.TopBorderInfo = new BorderInfo(0.0);
            this.BottomBorderInfo = new BorderInfo(0.0);
        }
        internal Rectangle rectangle;                   // Used only for RectangularHull
        internal OverlapRemovalCluster olapCluster;    // For use with RectangularHull only, and only valid during verletIntegration()
        /// <summary>
        /// Left margin of this cluster (additional space inside the cluster border).
        /// </summary>
        public double LeftMargin {
            get { return this.LeftBorderInfo.InnerMargin; }
            set { this.LeftBorderInfo = new BorderInfo(value, this.LeftBorderInfo.FixedPosition, this.LeftBorderInfo.Weight); }
        }
        /// <summary>
        /// Right margin of this cluster (additional space inside the cluster border).
        /// </summary>
        public double RightMargin {
            get { return this.RightBorderInfo.InnerMargin; }
            set { this.RightBorderInfo = new BorderInfo(value, this.RightBorderInfo.FixedPosition, this.RightBorderInfo.Weight); }
        }
        /// <summary>
        /// Top margin of this cluster (additional space inside the cluster border).
        /// </summary>
        public double TopMargin {
            get { return this.TopBorderInfo.InnerMargin; }
            set { this.TopBorderInfo = new BorderInfo(value, this.TopBorderInfo.FixedPosition, this.TopBorderInfo.Weight); }
        }
        /// <summary>
        /// Bottom margin of this cluster (additional space inside the cluster border).
        /// </summary>
        public double BottomMargin {
            get { return this.BottomBorderInfo.InnerMargin; }
            set { this.BottomBorderInfo = new BorderInfo(value, this.BottomBorderInfo.FixedPosition, this.BottomBorderInfo.Weight); }
        }

        /// <summary>
        /// Information for the Left border of the cluster.
        /// </summary>
        public BorderInfo LeftBorderInfo { get; set; }
        /// <summary>
        /// Information for the Right border of the cluster.
        /// </summary>
        public BorderInfo RightBorderInfo { get; set; }
        /// <summary>
        /// Information for the Top border of the cluster.
        /// </summary>
        public BorderInfo TopBorderInfo { get; set; }
        /// <summary>
        /// Information for the Bottom border of the cluster.
        /// </summary>
        public BorderInfo BottomBorderInfo { get; set; }

        /// <summary>
        /// When this is set, the OverlapRemovalCluster will generate equality constraints rather than inequalities
        /// to keep its children within its bounds.
        /// </summary>
        public bool GenerateFixedConstraints
        {
            get;
            set;
        }

        private bool generateFixedConstraintsDefault;

        /// <summary>
        /// The default value that GenerateFixedConstraints will be reverted to when a lock is released
        /// </summary>
        public bool GenerateFixedConstraintsDefault
        {
            get
            {
                return this.generateFixedConstraintsDefault;
            }
            set
            {
                this.GenerateFixedConstraints = this.generateFixedConstraintsDefault = value;
            }
        }

        /// <summary>
        /// The rectangular hull of all the points of all the nodes in the cluster, as set by
        /// ProjectionSolver.Solve().
        /// Note: This rectangle may not originate at the barycenter.  Drawing uses only the results
        /// of this function; the barycenter is used only for gravity computations.
        /// </summary>
        /// <returns></returns>
        public ICurve RectangularHull() {
            Debug.Assert(this.rectangle.Bottom <= this.rectangle.Top);
            if (this.RadiusX > 0 || this.RadiusY > 0)
            {
                return CurveFactory.CreateRectangleWithRoundedCorners(this.rectangle.Width, this.rectangle.Height, this.RadiusX, this.RadiusY, this.rectangle.Center);
            }
            else
            {
                return CurveFactory.CreateRectangle(this.rectangle.Width, this.rectangle.Height, this.rectangle.Center);
            }
        }
        /// <summary>
        /// Will only return something useful if FastIncrementalLayoutSettings.AvoidOverlaps is true.
        /// </summary>
        public Rectangle Rect {
            get {
                return this.rectangle;
            }
            set
            {
                this.rectangle = value;
            }
        }

        private class Margin
        {
            public double Left;
            public double Right;
            public double Top;
            public double Bottom;
        }

        private Margin defaultMargin;
        internal bool DefaultMarginIsSet {
            get { return this.defaultMargin !=null; }
        }

        /// <summary>
        /// The default margin stored by StoreDefaultMargin
        /// </summary>
        public double DefaultLeftMargin
        {
            get
            {
                return this.defaultMargin.Left;
            }
        }
        /// <summary>
        /// The default margin stored by StoreDefaultMargin
        /// </summary>
        public double DefaultTopMargin
        {
            get
            {
                return this.defaultMargin.Top;
            }
        }

        /// <summary>
        /// The default margin stored by StoreDefaultMargin
        /// </summary>
        public double DefaultRightMargin
        {
            get
            {
                return this.defaultMargin.Right;
            }
        }
        /// <summary>
        /// The default margin stored by StoreDefaultMargin
        /// </summary>
        public double DefaultBottomMargin
        {
            get
            {
                return this.defaultMargin.Bottom;
            }
        }

        /// <summary>
        /// store a the current margin as the default which we can revert to later with the RestoreDefaultMargin
        /// </summary>
        public void StoreDefaultMargin()
        {
            this.defaultMargin = new Margin { Left = this.LeftMargin, Right = this.RightMargin, Bottom = this.BottomMargin, Top = this.TopMargin };
        }

        /// <summary>
        /// store a default margin which we can revert to later with the RestoreDefaultMargin
        /// </summary>
        public void StoreDefaultMargin(double left, double right, double bottom, double top)
        {
            this.defaultMargin = new Margin { Left = left, Right = right, Bottom = bottom, Top = top };
        }

        /// <summary>
        /// revert to a previously stored default margin
        /// </summary>
        public void RestoreDefaultMargin()
        {
            if (this.defaultMargin != null)
            {
                this.LeftMargin = this.defaultMargin.Left;
                this.RightMargin = this.defaultMargin.Right;
                this.TopMargin = this.defaultMargin.Top;
                this.BottomMargin = this.defaultMargin.Bottom;
            }
        }

        
        /// <summary>
        /// Move the bounding box by delta
        /// </summary>
        /// <param name="delta"></param>
        public void TranslateRectangle(Point delta)
        {
            this.rectangle = new Rectangle(this.rectangle.Left + delta.X, this.rectangle.Bottom + delta.Y, new Point(this.rectangle.Width, this.rectangle.Height));
        }
        
        /// <summary>
        /// Radius on the X axis
        /// </summary>
        public double RadiusX { get; set; }
        /// <summary>
        /// Radius on the Y axis
        /// </summary>
        public double RadiusY { get; set; }
        /// <summary>
        /// Creates a lock on all four borders
        /// </summary>
        /// <param name="left"></param>
        /// <param name="right"></param>
        /// <param name="top"></param>
        /// <param name="bottom"></param>
        public void Lock(double left, double right, double top, double bottom) {
            double weight = 1e4;
            this.LeftBorderInfo = new BorderInfo(this.LeftBorderInfo.InnerMargin, left, weight);
            this.RightBorderInfo = new BorderInfo(this.RightBorderInfo.InnerMargin, right, weight);
            this.TopBorderInfo = new BorderInfo(this.TopBorderInfo.InnerMargin, top, weight);
            this.BottomBorderInfo = new BorderInfo(this.BottomBorderInfo.InnerMargin, bottom, weight);
        }
        /// <summary>
        /// Releases the lock on all four borders
        /// </summary>
        public void Unlock() {
            this.LeftBorderInfo = new BorderInfo(this.LeftBorderInfo.InnerMargin);
            this.RightBorderInfo = new BorderInfo(this.RightBorderInfo.InnerMargin);
            this.TopBorderInfo = new BorderInfo(this.TopBorderInfo.InnerMargin);
            this.BottomBorderInfo = new BorderInfo(this.BottomBorderInfo.InnerMargin);
        }
        /// <summary>
        /// boundary can shrink no more than this
        /// </summary>
        public double MinWidth
        {
            get;
            set;
        }
        /// <summary>
        /// boundary can shrink no more than this
        /// </summary>
        public double MinHeight
        {
            get;
            set;
        }

        
    }
}
