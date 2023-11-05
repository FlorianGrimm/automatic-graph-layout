using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    internal class ConeRightSide:ConeSide {
        internal ConeRightSide(Cone cone) {
            this.Cone = cone; 
        }

        internal override Point Start {
            get { return this.Cone.Apex; }
        }

        internal override Point Direction {
            get { return this.Cone.RightSideDirection; }
        }

        public override string ToString() {
            return "ConeRightSide " + this.Start + " " + this.Direction;
        }
    }
}
