using System;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.Visibility {
    internal class Tangent {
        private Tangent comp;

        //the complimentary tangent
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Tangent Comp {
            get { return this.comp; }
            set { this.comp = value; }
        }

        internal bool IsHigh {
            get { return !this.IsLow; }
        }

        private bool lowTangent; //true means that it is a low tangent to Q, false meanst that it is a high tangent to Q

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal bool IsLow {
            get { return this.lowTangent; }
            set { this.lowTangent = value; }
        }

        private bool separatingPolygons;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal bool SeparatingPolygons
        {
            get { return this.separatingPolygons; }
            set { this.separatingPolygons = value; }
        }

        private Diagonal diagonal;
        /// <summary>
        /// the diagonal will be not a null only when it is active
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Diagonal Diagonal {
            get { return this.diagonal; }
            set { this.diagonal = value; }
        }

        private PolylinePoint start;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal PolylinePoint Start {
            get { return this.start; }
            set { this.start = value; }
        }

        private PolylinePoint end;

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        public PolylinePoint End {
            get { return this.end; }
            set { this.end = value; }
        }
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Tangent(PolylinePoint start, PolylinePoint end) {
            this.Start = start;
            this.End = end;
        }

        public override string ToString() {
            return String.Format(System.Globalization.CultureInfo.InvariantCulture, "{0},{1}", this.Start, this.End);
        }
    }
}
