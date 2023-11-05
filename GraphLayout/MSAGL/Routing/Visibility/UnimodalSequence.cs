using System;

namespace Microsoft.Msagl.Routing.Visibility {
    /// <summary>
    /// A real functionf defined on
    /// the integers 0, 1, . . . , n-1 is said to be unimodal if there exists an integer m such that f is strictly increasing (respectively, decreasing) on [ 0, m] and
    /// decreasing (respectively, increasing) on [m + 1, n-1]
    /// No three sequential elements have the same value
    /// </summary>
    internal class UnimodalSequence {
        private Func<int,double> sequence;

        /// <summary>
        /// the sequence values
        /// </summary>
        internal Func<int,double> Sequence {
            get { return this.sequence; }
            set { this.sequence = value; }
        }

        private int length;
        /// <summary>
        /// the length of the sequence: the sequence starts from 0
        /// </summary>
        internal int Length {
            get { return this.length; }
            set { this.length = value; }
        }

        internal UnimodalSequence(Func<int,double> sequenceDelegate, int length) {
            this.Sequence = sequenceDelegate;
            this.Length = length;
        }

        internal enum Behavior {
            Increasing, Decreasing, Extremum,
        }

        internal int FindMinimum() {
            //find out first that the minimum is inside of the domain
            int a = 0;
            int b = this.Length - 1;
            int m = a + (b - a) / 2;
            double valAtM = this.Sequence(m);
            if (valAtM >= this.Sequence(0) && valAtM >= this.Sequence(this.Length - 1)) {
                return this.Sequence(0) < this.Sequence(this.Length - 1) ? 0 : this.Length - 1;
            }

            while (b - a > 1) {
                m = a + (b - a) / 2;
                switch (this.BehaviourAtIndex(m)) {
                    case Behavior.Decreasing:
                        a = m;
                        break;
                    case Behavior.Increasing:
                        b = m;
                        break;
                    case Behavior.Extremum:
                        return m;
                }
            }
            if (a == b) {
                return a;
            }

            return this.Sequence(a) <= this.Sequence(b) ? a : b;
        }

        private Behavior BehaviourAtIndex(int m) {
            double seqAtM= this.Sequence(m);
            if (m == 0) {
                double seqAt1 = this.Sequence(1);
                if (seqAt1 == seqAtM) {
                    return Behavior.Extremum;
                }

                return seqAt1 > seqAtM ? Behavior.Increasing : Behavior.Decreasing;
            }
            if (m == this.Length -1) {
                double seqAt1 = this.Sequence(this.Length -2);
                if (seqAt1 == seqAtM) {
                    return Behavior.Extremum;
                }

                return seqAt1 > seqAtM ? Behavior.Decreasing : Behavior.Increasing;
            }

            double delLeft = seqAtM - this.Sequence(m - 1);
            double delRight = this.Sequence(m + 1) - seqAtM;
            if (delLeft * delRight <= 0) {
                return Behavior.Extremum;
            }

            return delLeft > 0 ? Behavior.Increasing : Behavior.Decreasing;
        }

        internal int FindMaximum() {
            //find out first that the maximum is inside of the domain
            int a = 0;
            int b = this.Length - 1;
            int m = a + (b - a) / 2;
            double valAtM = this.Sequence(m);
            if (valAtM <= this.Sequence(0) && valAtM <= this.Sequence(this.Length - 1)) {
                return this.Sequence(0) > this.Sequence(this.Length - 1) ? 0 : this.Length - 1;
            }

            while (b - a > 1) {
                m = a + (b - a) / 2;
                switch (this.BehaviourAtIndex(m)) {
                    case Behavior.Decreasing:
                        b = m;
                        break;
                    case Behavior.Increasing:
                        a = m;
                        break;
                    case Behavior.Extremum:
                        return m;
                }
            }
            if (a == b) {
                return a;
            }

            return this.Sequence(a) >= this.Sequence(b) ? a : b;
            
        }
    }
}
