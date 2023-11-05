using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging
{
#pragma warning disable CS0659 // Type overrides Object.Equals(object o) but does not override Object.GetHashCode()
    internal class SegWithIndex {
#pragma warning restore CS0659 // Type overrides Object.Equals(object o) but does not override Object.GetHashCode()
        internal Point[] Points;
        internal int I;//offset
    
        internal SegWithIndex(Point[] pts, int i) {
            Debug.Assert(i<pts.Length&&i>=0);
            this.Points = pts;
            this.I = i;
        }

        internal Point Start {get{return this.Points[this.I];}}
        internal Point End{ get{return this.Points[this.I +1];}}
    
        override public bool Equals(object obj) {
            var other = (SegWithIndex) obj;
            return other.Points== this.Points && other.I == this.I;
        }

        

    }
}