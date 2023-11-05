using System;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;

namespace Microsoft.Msagl.Routing {
    /// <summary>
    /// the edge direction is from the upperSite to lowerSite
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif

    public class CdtEdge {
        ///<summary>
        ///</summary>
        public CdtSite upperSite;
        ///<summary>
        ///</summary>
        public CdtSite lowerSite;

        ///<summary>
        ///</summary>
        private CdtTriangle ccwTriangle; //in this triangle the edge goes counterclockwise

        ///<summary>
        ///</summary>
        private CdtTriangle cwTriangle; //in this triangle the edge goes clockwise, against the triangle orientation
        ///<summary>
        /// is an obstacle side, or a given segment
        ///</summary>
        public bool Constrained;

        ///<summary>
        ///</summary>
        ///<param name="a"></param>
        ///<param name="b"></param>
        public CdtEdge(CdtSite a, CdtSite b) {
            var above = Cdt.Above(a.Point, b.Point);
            if (above == 1) {
                this.upperSite = a;
                this.lowerSite = b;
            }
            else {
                Debug.Assert(above != 0);
                this.lowerSite = a;
                this.upperSite = b;
            }

            this.upperSite.AddEdgeToSite(this);
        }

        /// <summary>
        /// the amount of free space around the edge
        /// </summary>
        internal double Capacity = 1000000;


        /// <summary>
        /// the amount of residual free space around the edge
        /// </summary>
        public double ResidualCapacity { get; set; }

        ///<summary>
        ///</summary>
        public CdtTriangle CcwTriangle {
            get { return this.ccwTriangle; }
            set {
                Debug.Assert(value == null || this.cwTriangle ==null || value.OppositeSite(this) !=
                    this.cwTriangle.OppositeSite(this));
                this.ccwTriangle = value;
            }
        }

        ///<summary>
        ///</summary>
        public CdtTriangle CwTriangle {
            get { return this.cwTriangle; }
            set {
                Debug.Assert(value == null || this.ccwTriangle ==null || value.OppositeSite(this) !=
                    this.ccwTriangle.OppositeSite(this));
                this.cwTriangle = value;
            }
        }


        /// <summary>
        /// returns the trianlge on the edge opposite to the site 
        /// </summary>
        /// <param name="p"></param>
        /// <returns></returns>
        public CdtTriangle GetOtherTriangle(CdtSite p) {
            return this.cwTriangle.Contains(p) ? this.ccwTriangle : this.cwTriangle;
        }

        ///<summary>
        ///</summary>
        ///<param name="pi"></param>
        ///<returns></returns>
        public bool IsAdjacent(CdtSite pi) {
            return pi == this.upperSite || pi == this.lowerSite;
        }

        ///<summary>
        ///</summary>
        ///<param name="triangle"></param>
        ///<returns></returns>
        public CdtTriangle GetOtherTriangle(CdtTriangle triangle) {
            return this.ccwTriangle == triangle ? this.cwTriangle : this.ccwTriangle;
        }

        /// <summary>
        /// Returns a string that represents the current object.
        /// </summary>
        /// <returns>
        /// A string that represents the current object.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override string ToString() {
            return String.Format("({0},{1})", this.upperSite, this.lowerSite);
        }

        ///<summary>
        ///</summary>
        ///<param name="site"></param>
        ///<returns></returns>
        public CdtSite OtherSite(CdtSite site) {
            Debug.Assert(this.IsAdjacent(site));
            return this.upperSite == site ? this.lowerSite : this.upperSite;
        }
    }
}
