
namespace Microsoft.Msagl.Core.Geometry {
    internal class HullPoint {
        private Point point;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Point Point {
            get { return this.point; }
            set { this.point = value; }
        }

        private bool deleted;

        internal bool Deleted {
            get { return this.deleted; }
            set { this.deleted = value; }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal HullPoint(Point point) {
            this.Point = point;
        }

        public override string ToString() {
            return this.point + (this.Deleted ? "X" : "");
        }
    }
}
