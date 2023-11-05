using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation {
    internal class EdgeInserter {
        private readonly CdtEdge edge;
        private readonly Set<CdtTriangle> triangles;
        private readonly RbTree<CdtFrontElement> front;
        private readonly Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate;
        private List<CdtSite> rightPolygon = new List<CdtSite>();
        private List<CdtSite> leftPolygon = new List<CdtSite>();
        private List<CdtTriangle> addedTriangles=new List<CdtTriangle>();

        public EdgeInserter(CdtEdge edge, Set<CdtTriangle> triangles, RbTree<CdtFrontElement> front, Func<CdtSite, CdtSite, CdtEdge> createEdgeDelegate) {
            this.edge = edge;
            this.triangles = triangles;
            this.front = front;
            this.createEdgeDelegate = createEdgeDelegate;
        }

        public void Run() {
            this.TraceEdgeThroughTriangles();
            this.TriangulatePolygon(this.rightPolygon, this.edge.upperSite, this.edge.lowerSite, true);
            this.TriangulatePolygon(this.leftPolygon, this.edge.upperSite, this.edge.lowerSite, false);
            this.UpdateFront();
        }

        private void UpdateFront() {
            var newFrontEdges = new Set<CdtEdge>();
            foreach (var t in this.addedTriangles) {
                foreach (var e in t.Edges) {
                    if (e.CwTriangle == null || e.CcwTriangle == null) {
                        newFrontEdges.Insert(e);
                    }
                }
            }

            foreach (var e in newFrontEdges) {
                this.AddEdgeToFront(e);
            }
        }

        private void AddEdgeToFront(CdtEdge e) {
            var leftSite=e.upperSite.Point.X<e.lowerSite.Point.X?e.upperSite:e.lowerSite;
            this.front.Insert(new CdtFrontElement(leftSite, e));
        }

        private void TriangulatePolygon(List<CdtSite> polygon, CdtSite a, CdtSite b, bool reverseTrangleWhenCompare) {
            if (polygon.Count > 0) {
                this.TriangulatePolygon(0, polygon.Count - 1, polygon, a, b,reverseTrangleWhenCompare);
            }
        }

        private void TriangulatePolygon(int start, int end, List<CdtSite> polygon, CdtSite a, CdtSite b, bool reverseTrangleWhenCompare) {
//            if(CdtSweeper.db)
//               CdtSweeper.ShowFront(triangles,front, Enumerable.Range(start, end-start+1).Select(i=> new Ellipse(10,10,polygon[i].Point)).ToArray(), new[]{new LineSegment(a.Point,b.Point)});
            var c = polygon[start];
            int cIndex = start;
            for (int i = start + 1; i <= end; i++) {
                var v = polygon[i];
                if (LocalInCircle(v, a, b, c, reverseTrangleWhenCompare)) {
                    cIndex = i;
                    c = v;
                }
            }
            var t = new CdtTriangle(a, b, c, this.createEdgeDelegate);
            this.triangles.Insert(t);
            this.addedTriangles.Add(t);
            if (start < cIndex) {
                this.TriangulatePolygon(start, cIndex - 1, polygon, a, c, reverseTrangleWhenCompare);
            }

            if (cIndex < end) {
                this.TriangulatePolygon(cIndex + 1, end, polygon, c, b, reverseTrangleWhenCompare);
            }
        }

        private static bool LocalInCircle(CdtSite v, CdtSite a, CdtSite b, CdtSite c, bool reverseTrangleWhenCompare) {
            return reverseTrangleWhenCompare ? CdtSweeper.InCircle(v, a, c, b) : CdtSweeper.InCircle(v, a, b, c);
        }

        private void TraceEdgeThroughTriangles() {
            var edgeTracer = new EdgeTracer(this.edge, this.triangles, this.front, this.leftPolygon, this.rightPolygon);
            edgeTracer.Run();

        }
    }
}