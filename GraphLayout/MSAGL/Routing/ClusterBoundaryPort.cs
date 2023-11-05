using System;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Routing {
    ///<summary>
    ///this is a port for routing from a cluster
    ///</summary>
    public class ClusterBoundaryPort : RelativeFloatingPort {
        private Polyline loosePolyline;
        internal Polyline LoosePolyline {
            get { return this.loosePolyline; }
            set { this.loosePolyline = value; }
        }

        ///<summary>
        ///constructor
        ///</summary>
        ///<param name="curveDelegate"></param>
        ///<param name="centerDelegate"></param>
        ///<param name="locationOffset"></param>
        public ClusterBoundaryPort(Func<ICurve> curveDelegate, Func<Point> centerDelegate, Point locationOffset)
            : base(curveDelegate, centerDelegate, locationOffset) { }

        ///<summary>
        ///constructor 
        ///</summary>
        ///<param name="curveDelegate"></param>
        ///<param name="centerDelegate"></param>
        public ClusterBoundaryPort(Func<ICurve> curveDelegate, Func<Point> centerDelegate)
            : base(curveDelegate, centerDelegate) { }
    }
}