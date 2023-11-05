using System;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation {
    /// <summary>
    /// a trianlge oriented counterclockwise
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif

    public class CdtTriangle {
        ///<summary>
        /// the edges
        ///</summary>
        public readonly ThreeArray<CdtEdge> Edges = new ThreeArray<CdtEdge>();
        ///<summary>
        /// the sites
        ///</summary>
        public readonly ThreeArray<CdtSite> Sites = new ThreeArray<CdtSite>();

        public TriangleOrientation Orientation { get { return Point.GetTriangleOrientation(this.Sites[0].Point, this.Sites[1].Point, this.Sites[2].Point); } }

        internal CdtTriangle(CdtSite a, CdtSite b, CdtSite c, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            var orientation = Point.GetTriangleOrientation(a.Point, b.Point, c.Point);
            switch (orientation) {
                case TriangleOrientation.Counterclockwise:
                    this.FillCcwTriangle(a, b, c, createEdgeDelegate);
                    break;
                case TriangleOrientation.Clockwise:
                    this.FillCcwTriangle(a, c, b, createEdgeDelegate);
                    break;
                default: throw new InvalidOperationException();
            }
        }

        internal CdtTriangle(CdtSite pi, CdtEdge edge, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            switch (Point.GetTriangleOrientation(edge.upperSite.Point, edge.lowerSite.Point, pi.Point)) {
                case TriangleOrientation.Counterclockwise:
                    edge.CcwTriangle = this;
                    this.Sites[0] = edge.upperSite;
                    this.Sites[1] = edge.lowerSite;
                    break;
                case TriangleOrientation.Clockwise:
                    edge.CwTriangle = this;
                    this.Sites[0] = edge.lowerSite;
                    this.Sites[1] = edge.upperSite;
                    break;
                default:
                    throw new InvalidOperationException();
            }
            this.Edges[0] = edge;
            this.Sites[2] = pi;
            this.CreateEdge(1, createEdgeDelegate);
            this.CreateEdge(2, createEdgeDelegate);
        }

        //
        internal CdtTriangle(CdtSite aLeft, CdtSite aRight, CdtSite bRight, CdtEdge a, CdtEdge b, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            // Debug.Assert(Point.GetTriangleOrientation(aLeft.Point, aRight.Point, bRight.Point) == TriangleOrientation.Counterclockwise);
            this.Sites[0] = aLeft;
            this.Sites[1] = aRight;
            this.Sites[2] = bRight;
            this.Edges[0] = a;
            this.Edges[1] = b;
            this.BindEdgeToTriangle(aLeft, a);
            this.BindEdgeToTriangle(aRight, b);
            this.CreateEdge(2, createEdgeDelegate);
            Debug.Assert(this.Orientation != TriangleOrientation.Collinear);
        }

        /// <summary>
        /// in the trianlge, which is always oriented counterclockwise, the edge starts at site 
        /// </summary>
        /// <param name="site"></param>
        /// <param name="edge"></param>
        private void BindEdgeToTriangle(CdtSite site, CdtEdge edge) {
            if (site == edge.upperSite) {
                edge.CcwTriangle = this;
            } else {
                edge.CwTriangle = this;
            }
        }

        /// <summary>
        /// here a,b,c comprise a ccw triangle
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <param name="createEdgeDelegate"></param>
        private void FillCcwTriangle(CdtSite a, CdtSite b, CdtSite c, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            this.Sites[0] = a; this.Sites[1] = b; this.Sites[2] = c;
            for (int i = 0; i < 3; i++) {
                this.CreateEdge(i, createEdgeDelegate);
            }
        }

        private void CreateEdge(int i, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            var a = this.Sites[i];
            var b = this.Sites[i + 1];
            var edge = this.Edges[i] = createEdgeDelegate(a, b);
            this.BindEdgeToTriangle(a, edge);
        }

        internal bool Contains(CdtSite cdtSite) {
            return this.Sites.Contains(cdtSite);
        }

        internal CdtEdge OppositeEdge(CdtSite pi) {
            var index = this.Sites.Index(pi);
            Debug.Assert(index != -1);
            return this.Edges[index + 1];
        }

#if TEST_MSAGL
        /// <summary>
        /// Returns a <see cref="T:System.String"/> that represents the current <see cref="T:System.Object"/>.
        /// </summary>
        /// <returns>
        /// A <see cref="T:System.String"/> that represents the current <see cref="T:System.Object"/>.
        /// </returns>
        /// <filterpriority>2</filterpriority>
        public override string ToString() {
            return String.Format("({0},{1},{2}", this.Sites[0], this.Sites[1], this.Sites[2]);
        }
#endif

        internal CdtSite OppositeSite(CdtEdge cdtEdge) {
            var i = this.Edges.Index(cdtEdge);
            return this.Sites[i + 2];
        }

        internal Rectangle BoundingBox() {
            Rectangle rect = new Rectangle(this.Sites[0].Point, this.Sites[1].Point);
            rect.Add(this.Sites[2].Point);
            return rect;
        }
    }
}
