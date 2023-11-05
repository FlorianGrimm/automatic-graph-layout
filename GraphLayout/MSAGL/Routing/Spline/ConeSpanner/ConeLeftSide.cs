using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    internal class ConeLeftSide:ConeSide {
        internal ConeLeftSide(Cone cone) { this.Cone = cone; }

        internal override Point Start {
            get { return this.Cone.Apex; }
        }

        internal override Point Direction {
            get { return this.Cone.LeftSideDirection; }
        }
#if TEST_MSAGL
        public override string ToString() {
            return "ConeLeftSide " + this.Start + " " + this.Direction;
        }
#endif
    }
}
