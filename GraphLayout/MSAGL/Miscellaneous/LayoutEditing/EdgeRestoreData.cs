using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Prototype.LayoutEditing{
    /// <summary>
    /// holds the data needed to restore the edge after the editing
    /// </summary>
    public class EdgeRestoreData : RestoreData {
        private Point labelCenter;

        [SuppressMessage("Microsoft.Performance", "CA1804:RemoveUnusedLocals", MessageId = "p")]
        internal EdgeRestoreData(Edge edge) {
            if (edge.UnderlyingPolyline == null) {
                var asCurve = (edge.Curve as Curve) ?? new Curve(new List<ICurve> {edge.Curve});
                edge.UnderlyingPolyline =
                    SmoothedPolyline.FromPoints(
                        new[] {edge.Source.Center}.Concat(Polyline.PolylineFromCurve(asCurve)).
                            Concat(new[] {edge.Target.Center}));
            }
            this.UnderlyingPolyline = edge.UnderlyingPolyline.Clone();

            this.Curve = edge.Curve.Clone();
            if (edge.EdgeGeometry.SourceArrowhead != null) {
                this.ArrowheadAtSourcePosition = edge.EdgeGeometry.SourceArrowhead.TipPosition;
            }

            if (edge.EdgeGeometry.TargetArrowhead != null) {
                this.ArrowheadAtTargetPosition = edge.EdgeGeometry.TargetArrowhead.TipPosition;
            }

            if (edge.Label != null && edge.UnderlyingPolyline != null) {
                this.labelCenter = edge.Label.Center;
                Curve untrimmedCurve = edge.UnderlyingPolyline.CreateCurve();
                this.LabelAttachmentParameter = untrimmedCurve.ClosestParameter(this.labelCenter);
            }
        }

        /// <summary>
        /// the underlying polyline
        /// </summary>
        [SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Polyline")]
        public SmoothedPolyline UnderlyingPolyline { get; set; }

        /// <summary>
        /// the initial center
        /// </summary>
        public Point LabelCenter {
            get { return this.labelCenter; }
            set { this.labelCenter = value; }
        }

        /// <summary>
        /// the edge original curve
        /// </summary>
        public ICurve Curve { get; set; }

        /// <summary>
        /// the arrow head position at source
        /// </summary>
        public Point ArrowheadAtSourcePosition { get; set; }

        /// <summary>
        /// the arrow head position at target
        /// </summary>
        public Point ArrowheadAtTargetPosition { get; set; }

        

        /// <summary>
        /// the closest point to the label center
        /// </summary>
        public double LabelAttachmentParameter { get; set; }
    }
}