//
// ScanDirection.cs
// MSAGL ScanDirection class for Rectilinear Edge Routing line generation.
//
// Copyright Microsoft Corporation.
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Rectilinear {
    // Encapsulate a direction and some useful values derived therefrom.
    internal class ScanDirection {
        // The direction of primary interest, either the direction of the sweep (the
        // coordinate the scanline sweeps "up" in) or along the scan line ("sideways"
        // to the sweep direction, scanning for obstacles).
        internal Direction Direction { get; private set; }
        internal Point DirectionAsPoint { get; private set; }

        // The perpendicular direction - opposite of comments for Direction.
        internal Direction PerpDirection { get; private set; }
        internal Point PerpDirectionAsPoint { get; private set; }

        // The oppposite direction of the primary direction.
        internal Direction OppositeDirection { get; private set; }

        // Use the internal static xxxInstance properties to get an instance.
        private ScanDirection(Direction directionAlongScanLine) {
            System.Diagnostics.Debug.Assert(StaticGraphUtility.IsAscending(directionAlongScanLine),
                "directionAlongScanLine must be ascending");
            this.Direction = directionAlongScanLine;
            this.DirectionAsPoint = CompassVector.ToPoint(this.Direction);
            this.PerpDirection = (Direction.North == directionAlongScanLine) ? Direction.East : Direction.North;
            this.PerpDirectionAsPoint = CompassVector.ToPoint(this.PerpDirection);
            this.OppositeDirection = CompassVector.OppositeDir(directionAlongScanLine);
        }

        internal bool IsHorizontal { get { return Direction.East == this.Direction; } }
        internal bool IsVertical { get { return Direction.North == this.Direction; } }

        // Compare in perpendicular direction first, then parallel direction.
        internal int Compare(Point lhs, Point rhs) {
            int cmp = this.ComparePerpCoord(lhs, rhs);
            return (0 != cmp) ? cmp : this.CompareScanCoord(lhs, rhs);
        }

        internal int CompareScanCoord(Point lhs, Point rhs) {
            return PointComparer.Compare((lhs - rhs) * this.DirectionAsPoint, 0.0);
        }
        internal int ComparePerpCoord(Point lhs, Point rhs) {
            return PointComparer.Compare((lhs - rhs) * this.PerpDirectionAsPoint, 0.0);
        }

        internal bool IsFlat(SegmentBase seg) {
            return this.IsFlat(seg.Start, seg.End);
        }
        internal bool IsFlat(Point start, Point end) {
            // Return true if there is no change in the perpendicular direction.
            return PointComparer.Equal((end - start) * this.PerpDirectionAsPoint, 0.0);
        }
        internal bool IsPerpendicular(SegmentBase seg) {
            return this.IsPerpendicular(seg.Start, seg.End);
        }
        internal bool IsPerpendicular(Point start, Point end) {
            // Return true if there is no change in the primary direction.
            return PointComparer.Equal((end - start) * this.DirectionAsPoint, 0.0);
        }

        internal double Coord(Point point) {
            return point * this.DirectionAsPoint;
        }

        internal Point Min(Point first, Point second) {
            return (this.Compare(first, second) <= 0) ? first : second;
        }
        internal Point Max(Point first, Point second) {
            return (this.Compare(first, second) >= 0) ? first : second;
        }

// ReSharper disable InconsistentNaming
        private static readonly ScanDirection horizontalInstance = new ScanDirection(Direction.East);
        private static readonly ScanDirection verticalInstance = new ScanDirection(Direction.North);
// ReSharper restore InconsistentNaming
        internal static ScanDirection HorizontalInstance { get { return horizontalInstance; } }
        internal static ScanDirection VerticalInstance { get { return verticalInstance; } }
        internal ScanDirection PerpendicularInstance { get { return this.IsHorizontal ? VerticalInstance : HorizontalInstance; } }
        static internal ScanDirection GetInstance(Direction dir) { return StaticGraphUtility.IsVertical(dir) ? VerticalInstance : HorizontalInstance; }

        /// <summary/>
        public override string ToString() {
            return this.Direction.ToString();
        }
    }
}