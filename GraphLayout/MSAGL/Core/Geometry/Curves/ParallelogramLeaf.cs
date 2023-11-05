using System;

namespace Microsoft.Msagl.Core.Geometry.Curves {

    /// <summary>
    /// A leaf of the ParallelogramNodeOverICurve hierarchy.
    /// Is used in curve intersectons routine.
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    internal class ParallelogramLeaf : ParallelogramNodeOverICurve {
        private double low;

        internal double Low {
            get {
                return this.low;
            }
            set {
                this.low = value;
            }
        }

        private double high;

        internal double High {
            get {
                return this.high;
            }
            set {
                this.high = value;
            }
        }

        internal ParallelogramLeaf(double low, double high, Parallelogram box, ICurve seg, double leafBoxesOffset)
            : base(seg, leafBoxesOffset) {
            this.low = low;
            this.high = high;
            this.Parallelogram = box;
        }

        private LineSegment chord;

        internal LineSegment Chord {
            get { return this.chord; }
            set {
                this.chord = value;
                if (!ApproximateComparer.Close(this.Seg[this.low], this.chord.Start)) {
                    throw new InvalidOperationException();
                }
            }
        }
    }
}
