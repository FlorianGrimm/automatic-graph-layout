using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    internal class Cone {
        private bool removed;

        internal bool Removed {
            get { return this.removed; }
            set { this.removed = value; }
        }

        private Point apex;
        private readonly IConeSweeper coneSweeper;

        internal Cone(Point apex, IConeSweeper coneSweeper) {
            this.apex = apex;
            this.coneSweeper = coneSweeper;
        }

        internal Point Apex {
            get { return this.apex; }
            set { this.apex = value; }
        }

        internal Point RightSideDirection {
            get { return this.coneSweeper.ConeRightSideDirection; }
        }

        internal Point LeftSideDirection {
            get { return this.coneSweeper.ConeLeftSideDirection; }
        }



        private ConeSide rightSide;

        internal ConeSide RightSide {
            get { return this.rightSide; }
            set {
                this.rightSide = value;
                this.rightSide.Cone = this;
            }
        }
        private ConeSide leftSide;

        internal ConeSide LeftSide {
            get { return this.leftSide; }
            set {
                this.leftSide = value;
                this.leftSide.Cone = this;
            }
        }
    }
}
