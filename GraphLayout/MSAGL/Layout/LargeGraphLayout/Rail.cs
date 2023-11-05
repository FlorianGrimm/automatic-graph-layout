using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing.Visibility;
using LineSegment = Microsoft.Msagl.Core.Geometry.Curves.LineSegment;

namespace Microsoft.Msagl.Layout.LargeGraphLayout {


    /// <summary>
    /// keeps a part of EdgeGeometry which is visible
    /// </summary>
    public class Rail {
        public Point A;
        public Point B;
        public Point targetA;
        public Point targetB;
        public Point initialA;
        public Point initialB;
        public Point Left;
        public Point Right;
        public int Weight = 1;

        // public List<int> unnecessaryTransfer = new List<int>();
#if TEST_MSAGL
        private static int railCount;
        private int id;
#endif

        /// <summary>
        /// the number of higlighted edges passing through the rail
        /// </summary>
        private bool _isHighlighted;

        /// <summary>
        /// can be ICurve or Arrowhead
        /// </summary>
        public object Geometry;

        /// <summary>
        /// the point where the edge curve touches the arrowhead
        /// </summary>
        public Point CurveAttachmentPoint;

        /// <summary>
        /// the corresponding LgEdgeInfo
        /// </summary>
        public LgEdgeInfo TopRankedEdgeInfoOfTheRail;
        private bool _isUsedOnPreviousLevel;

        public double MinPassingEdgeZoomLevel = Double.MaxValue;

        public bool IsUsedOnPreviousLevel
        {
            get { return this._isUsedOnPreviousLevel; }
            set { this._isUsedOnPreviousLevel = value; }
        }

        public int ZoomLevel;
        public List<object> Color;
#if TEST_MSAGL
        private Rail() {
            railCount++;
            this.id = railCount;
        }
        /// <summary>
        /// returning id for debugging
        /// </summary>
        /// <returns></returns>
        public int GetId() {
            return this.id;
        }
        /// <summary>
        /// Returns a string that represents the current object.
        /// </summary>
        /// <returns>
        /// A string that represents the current object.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override string ToString() {
            return this.id + this.StartEndString();
        }

        private string StartEndString() {
            Point s, t;
            return this.GetStartEnd(out s, out t) ? " " + new LineSegment(s, t) : "";
        }
#endif

        internal Rail(ICurve curveSegment, LgEdgeInfo topRankedEdgeInfoOfTheRail, int zoomLevel)
#if TEST_MSAGL
            : this()
#endif
        {
            this.TopRankedEdgeInfoOfTheRail = topRankedEdgeInfoOfTheRail;
            this.ZoomLevel = zoomLevel;
            this.Geometry = curveSegment;
        }

        internal Rail(Arrowhead arrowhead, Point curveAttachmentPoint, LgEdgeInfo topRankedEdgeInfoOfTheRail,
            int zoomLevel)
#if TEST_MSAGL
            : this()
#endif
        {
            this.TopRankedEdgeInfoOfTheRail = topRankedEdgeInfoOfTheRail;
            this.Geometry = arrowhead.Clone();
            this.CurveAttachmentPoint = curveAttachmentPoint;
            this.ZoomLevel = zoomLevel;
        }

        public Rectangle BoundingBox {
            get {
                var icurve = this.Geometry as ICurve;
                if (icurve != null) {
                    return icurve.BoundingBox;
                }

                var arrowhead = (Arrowhead)this.Geometry;
                var rec = new Rectangle(arrowhead.TipPosition, this.CurveAttachmentPoint);
                rec.Pad(arrowhead.Width); // sometimes this box will not cover the arrowhead, but rarely
                return rec;
            }
        }

        /// <summary>
        /// the number of higlighted edges passing through the rail
        /// </summary>
        public bool IsHighlighted {
            get { return this._isHighlighted; }
            set { this._isHighlighted = value; }
        }



        internal Tuple<Point, Point> PointTuple() {
            var icurve = this.Geometry as ICurve;
            if (icurve != null) {
                return new Tuple<Point, Point>(icurve.Start, icurve.End);
            }

            var arrowhead = (Arrowhead)this.Geometry;
            return new Tuple<Point, Point>(arrowhead.TipPosition, this.CurveAttachmentPoint);
        }

        
        public Rail(ICurve curveSegment, int zoomLevel)
        {
            this.ZoomLevel = zoomLevel;
#if TEST_MSAGL
            railCount++;
            this.id = railCount;
#endif
            this.Geometry = curveSegment;
        }

        public bool GetStartEnd(out Point p0, out Point p1) {
            var curve = this.Geometry as ICurve;
            if (curve != null) {
                p0 = curve.Start;
                p1 = curve.End;
                return true;
            }
            var arrow = this.Geometry as Arrowhead;
            if (arrow != null) {
                p0 = this.CurveAttachmentPoint;
                p1 = arrow.TipPosition;
                return true;
            }
            p0 = new Point();
            p1 = new Point();
            return false;
        }

        public void UpdateTopEdgeInfo(LgEdgeInfo ei)
        {
            if (this.TopRankedEdgeInfoOfTheRail == null || this.TopRankedEdgeInfoOfTheRail.Rank < ei.Rank) {
                this.TopRankedEdgeInfoOfTheRail = ei;
            }

            this.MinPassingEdgeZoomLevel = 1+Math.Min(this.MinPassingEdgeZoomLevel, ei.ZoomLevel);
        }

        public LgEdgeInfo GetTopEdgeInfo()
        {
            return this.TopRankedEdgeInfoOfTheRail;
        }
    }
}
