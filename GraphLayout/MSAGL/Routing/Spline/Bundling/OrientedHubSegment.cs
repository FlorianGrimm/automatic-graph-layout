using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class OrientedHubSegment {
        private ICurve segment;
        internal bool Reversed;
        internal int Index;
        internal BundleBase BundleBase;

        internal OrientedHubSegment(ICurve seg, bool reversed, int index, BundleBase bundleBase) {
            this.Segment = seg;
            this.Reversed = reversed;
            this.Index = index;
            this.BundleBase = bundleBase;
        }

        internal Point this[double t] { get { return this.Reversed ? this.Segment[this.Segment.ParEnd - t] : this.Segment[t]; } }

        internal OrientedHubSegment Other { get; set; }
        internal ICurve Segment {
            get { return this.segment; }
            set {
                this.segment = value;
            }
        }
    }
}