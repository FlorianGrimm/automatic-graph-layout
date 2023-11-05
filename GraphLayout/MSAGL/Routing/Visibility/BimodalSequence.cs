using System;

namespace Microsoft.Msagl.Routing.Visibility {
    /// <summary>
    /// For our purposes, it suffices to define a bimodal function as
    /// one for which there is an r in [ 0, n-1] such that f(r), f(r + I), . . . , f(n), f( l), . . . ,
    /// f(r - 1) is unimodal. In our case no three sequential elements have the same value
    /// </summary>
    internal class BimodalSequence {

        internal BimodalSequence(Func<int,double> sequence, int length) {
            this.sequence = sequence;
            this.length = length;
        }

        private Func<int,double> sequence;

        /// <summary>
        /// the sequence values
        /// </summary>
        internal Func<int,double> Sequence {
            get { return this.sequence; }
        }

        private int length;
        /// <summary>
        /// the length of the sequence: the sequence starts from 0
        /// </summary>
        internal int Length {
            get { return this.length; }
        }
        //following Chazelle, Dobkin
        internal int FindMinimum(){
            if (this.sequence(0) == this.sequence(this.Length - 1)) //we have a unimodal function
{
                return (new UnimodalSequence(this.sequence, this.Length)).FindMinimum();
            }

            return (new UnimodalSequence(this.GetAdjustedSequenceForMinimum(), this.Length)).FindMinimum();

           
        }

        private Func<int,double> GetAdjustedSequenceForMinimum() {
            double leftVal= this.Sequence(0);
            double rightVal= this.Sequence(this.Length -1);
            double k=(rightVal-leftVal)/(this.Length -1);
            return delegate(int i) { return Math.Min(this.Sequence(i), leftVal + k * i); };
        }

        internal int FindMaximum() {
            if (this.sequence(0) == this.sequence(this.Length - 1)) //we have a unimodal function
{
                return (new UnimodalSequence(this.sequence, this.Length)).FindMaximum();
            }

            return (new UnimodalSequence(this.GetAdjustedSequenceForMaximum(), this.Length)).FindMaximum();

        }

        private Func<int,double> GetAdjustedSequenceForMaximum() {
            double leftVal = this.Sequence(0);
            double rightVal = this.Sequence(this.Length - 1);
            double k = (rightVal - leftVal) / (this.Length - 1);
            return delegate(int i) { return Math.Max(this.Sequence(i), leftVal + k * i); };
        }
    }
}
