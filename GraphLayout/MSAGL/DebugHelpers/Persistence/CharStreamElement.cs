namespace Microsoft.Msagl.DebugHelpers.Persistence {
    internal class CharStreamElement : CurveStreamElement {
        internal CharStreamElement(char ch) {
            this.Value = ch;
        }

        internal char Char { get { return (char)this.Value; } }
    }
}