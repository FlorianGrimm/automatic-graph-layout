namespace Microsoft.Msagl.DebugHelpers {
    internal class DoubleStreamElement : CurveStreamElement {
        public DoubleStreamElement(double res) {
            this.Value = res;
        }

        internal double Double { get { return (double)this.Value; } }
    }
}