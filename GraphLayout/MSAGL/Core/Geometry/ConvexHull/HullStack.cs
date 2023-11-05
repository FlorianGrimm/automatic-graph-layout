namespace Microsoft.Msagl.Core.Geometry {
    internal class HullStack {
        private Point hullPoint;

        internal Point Point {
            get { return this.hullPoint; }
            set { this.hullPoint = value; }
        }

        private HullStack next;

        internal HullStack Next {
            get { return this.next; }
            set { this.next = value; }
        }

        internal HullStack(Point hullPoint) {
            this.Point = hullPoint;
        }

        public override string ToString() {
            return this.hullPoint.ToString();
        }

    }
}
