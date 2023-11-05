using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    ///     Keeps the curve of the edge and arrowhead positions
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class EdgeGeometry {
        private ICurve? _Curve;
        private SmoothedPolyline smoothedPolyline;

        /// <summary>
        /// </summary>
        public EdgeGeometry() {
        }

        internal EdgeGeometry(Port sourcePort, Port targetPort) {
            this.SourcePort = sourcePort;
            this.TargetPort = targetPort;
        }

        /// <summary>
        /// </summary>
        public Arrowhead SourceArrowhead { get; set; }

        /// <summary>
        /// </summary>
        public Arrowhead TargetArrowhead { get; set; }

        /// <summary>
        ///     Defines the way the edge connects to the source.
        ///     The member is used at the moment only when adding an edge to the graph.
        /// </summary>
        public Port? SourcePort { get; set; }

        /// <summary>
        ///     defines the way the edge connects to the target
        ///     The member is used at the moment only when adding an edge to the graph.
        /// </summary>
        public Port? TargetPort { get; set; }

        
        /// <summary>
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return String.Format(CultureInfo.InvariantCulture, "{0}->{1}", this.SourcePort.Location, this.TargetPort.Location);
        }

        /// <summary>
        ///     edge thickness
        /// </summary>
        public double LineWidth { get; set; }


        /// <summary>
        ///     A curve representing the edge
        /// </summary>
        public ICurve? Curve {
            get { return this._Curve; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this._Curve = value;                
            }
        }

        /// <summary>
        ///     the polyline of the untrimmed spline
        /// </summary>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public SmoothedPolyline SmoothedPolyline {
            get { return this.smoothedPolyline; }
            set { this.smoothedPolyline = value; }
        }

        /// <summary>
        ///     getting the bounding box of the curve and optional arrow heads
        /// </summary>
        public Rectangle BoundingBox {
            get {
                Rectangle bBox = this.Curve.BoundingBox;
                if (this.SourceArrowhead != null) {
                    bBox.Add(this.SourceArrowhead.TipPosition);
                }

                if (this.TargetArrowhead != null) {
                    bBox.Add(this.TargetArrowhead.TipPosition);
                }

                double del = 0.5* this.LineWidth;
                var delta = new Point(-del, del);
                bBox.Add(bBox.LeftTop + delta);
                bBox.Add(bBox.RightBottom - delta);
                return bBox;
            }
        }
        
        internal void SetSmoothedPolylineAndCurve(SmoothedPolyline poly) {
            this.SmoothedPolyline = poly;
            this.Curve = poly.CreateCurve();
        }

        /// <summary>
        ///     Translate all the geometries with absolute positions by the specified delta
        /// </summary>
        /// <param name="delta">vector by which to translate</param>
        public void Translate(Point delta) {
            if (delta.X == 0 && delta.Y == 0) {
                return;
            }

            this.RaiseLayoutChangeEvent(delta);
            if (this.Curve != null) {
                this.Curve.Translate(delta);
            }

            if (this.SmoothedPolyline != null) {
                for (CornerSite s = this.SmoothedPolyline.HeadSite, s0 = this.SmoothedPolyline.HeadSite;
                     s != null;
                     s = s.Next, s0 = s0.Next) {
                    s.Point = s0.Point + delta;
                }
            }

            if (this.SourceArrowhead != null) {
                this.SourceArrowhead.TipPosition += delta;
            }

            if (this.TargetArrowhead != null) {
                this.TargetArrowhead.TipPosition += delta;
            }
        }

        internal double GetMaxArrowheadLength() {
            double l = 0;
            if (this.SourceArrowhead != null) {
                l = this.SourceArrowhead.Length;
            }

            if (this.TargetArrowhead != null && this.TargetArrowhead.Length > l) {
                return this.TargetArrowhead.Length;
            }

            return l;
        }

        
        /// <summary>
        /// </summary>
        public event EventHandler<LayoutChangeEventArgs> LayoutChangeEvent;

        
        /// <summary>
        /// </summary>
        /// <param name="newValue"></param>
        public void RaiseLayoutChangeEvent(object newValue) {
            if (LayoutChangeEvent != null) {
                LayoutChangeEvent(this, new LayoutChangeEventArgs{DataAfterChange = newValue});
            }
        }
    }
}