using System;

namespace Microsoft.Msagl.Core.DataStructures {
    /// <summary>
    /// this class behaves like one dimensional bounding box
    /// </summary>
    public class RealNumberSpan{
        internal RealNumberSpan(){
            this.IsEmpty = true;
        }

        internal bool IsEmpty { get; set; }

        internal void AddValue(double x){
            if(this.IsEmpty) {
                this.Min = this.Max = x;
                this.IsEmpty = false;
            } else if(x < this.Min) {
                this.Min = x;
            } else if(x > this.Max) {
                this.Max = x;
            }
        }

        internal double Min { get; set; }
        internal double Max { get; set; }
        /// <summary>
        /// 
        /// </summary>
        public double Length{
            get { return this.Max - this.Min; }
        }
#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.String.Format(System.String,System.Object,System.Object)")]
        public override string ToString() {
            return this.IsEmpty ? "empty" : String.Format("{0},{1}", this.Min, this.Max);
        }
#endif
    }
}