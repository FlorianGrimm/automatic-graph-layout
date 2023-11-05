using System.Diagnostics;

namespace Microsoft.Msagl.Core.ProjectionSolver{
    internal class UniformSolverVar{
        private double lowBound = double.NegativeInfinity;
        private double upperBound = double.PositiveInfinity;
        internal bool IsFixed;
        private double position;

        internal double Width { get; set; }

        internal double Position{
            get { return this.position; }
            set {
                if (value < this.lowBound) {
                    this.position = this.lowBound;
                } else if (value > this.upperBound) {
                    this.position = this.upperBound;
                } else {
                    this.position = value;
                }
            }
        }

        internal double LowBound {
            get { return this.lowBound; }
            set {
                Debug.Assert(value<= this.upperBound);
                this.lowBound = value;
            }
        }

        internal double UpperBound {
            get { return this.upperBound; }
            set {
                Debug.Assert(value>= this.LowBound);
                this.upperBound = value;
            }
        }

        

#if TEST_MSAGL
        public override string ToString() {
            return this.lowBound + " " + this.Position + " " + this.upperBound;
        }
#endif
    }
}