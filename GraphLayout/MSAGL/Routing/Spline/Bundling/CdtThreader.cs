using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class CdtThreader {
        private readonly Point start;
        private readonly Point end;
        private int positiveSign;
        private int negativeSign;
        private CdtEdge currentPiercedEdge;
        internal CdtEdge CurrentPiercedEdge {get { return this.currentPiercedEdge; } }

        private CdtTriangle currentTriangle;
        internal CdtTriangle CurrentTriangle { get { return this.currentTriangle; } }

        internal CdtThreader(CdtTriangle startTriangle,Point start,Point end) {
            Debug.Assert(PointLocationForTriangle(start, startTriangle) != PointLocation.Outside);
            this.currentTriangle = startTriangle;
            this.start = start;
            this.end = end;
        }

#if TEST_MSAGL || DEVTRACE
        internal IEnumerable<CdtTriangle> Triangles() {
            while(this.MoveNext()) {
                yield return this.CurrentTriangle;
            }
        }
#endif

        internal CdtEdge FindFirstPiercedEdge() {
            Debug.Assert(PointLocationForTriangle(this.start, this.currentTriangle) != PointLocation.Outside);
            Debug.Assert(PointLocationForTriangle(this.end, this.currentTriangle) == PointLocation.Outside);

            var sign0 = this.GetHyperplaneSign(this.currentTriangle.Sites[0]);
            var sign1 = this.GetHyperplaneSign(this.currentTriangle.Sites[1]);
            if (sign0 != sign1) {
                if (Point.GetTriangleOrientation(this.end, this.currentTriangle.Sites[0].Point, this.currentTriangle.Sites[1].Point) == TriangleOrientation.Clockwise) {
                    this.positiveSign = sign0;
                    this.negativeSign = sign1;
                    return this.currentTriangle.Edges[0];
                }
            }
            var sign2 = this.GetHyperplaneSign(this.currentTriangle.Sites[2]);
            if (sign1 != sign2) {
                if (Point.GetTriangleOrientation(this.end, this.currentTriangle.Sites[1].Point, this.currentTriangle.Sites[2].Point) == TriangleOrientation.Clockwise) {
                    this.positiveSign = sign1;
                    this.negativeSign = sign2;
                    return this.currentTriangle.Edges[1];
                }
            }

            this.positiveSign = sign2;
            this.negativeSign = sign0;
            /*
            if (positiveSign <= negativeSign) {
                //if (cdt != null) {
                //    Stream stream = File.Open(@"c:/tmp/triangles", FileMode.Create);
                //    BinaryFormatter bformatter = new BinaryFormatter();

                //    bformatter.Serialize(stream,  cdt.GetTriangles().ToArray());
                //    bformatter.Serialize(stream, start);
                //    bformatter.Serialize(stream, end);
                //    stream.Close();

                //}
                
               var  l = new List<DebugCurve>();
                l.Add(new DebugCurve(0.05, "red", new LineSegment(start, end)));
                var piercedEdge = currentTriangle.Edges[2];
                l.Add(new DebugCurve(0.05, "blue", new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point)));
                l.Add(new DebugCurve(new Ellipse(3, 3, end)));

                var ii = currentTriangle.Edges.Index(piercedEdge);
                for (int j = ii + 1; j <= ii + 2; j++)
                    l.Add(new DebugCurve(0.05, "brown", new LineSegment(currentTriangle.Edges[j].upperSite.Point, currentTriangle.Edges[j].lowerSite.Point)));
                LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
            }*/
            Debug.Assert(this.positiveSign > this.negativeSign);
            return this.currentTriangle.Edges[2];
        }

        internal static PointLocation PointLocationForTriangle(Point p, CdtTriangle triangle) {
            bool seenBoundary = false;
            for (int i = 0; i < 3; i++) {
                var area = Point.SignedDoubledTriangleArea(p, triangle.Sites[i].Point, triangle.Sites[i + 1].Point);
                if (area < -ApproximateComparer.DistanceEpsilon) {
                    return PointLocation.Outside;
                }

                if (area < ApproximateComparer.DistanceEpsilon) {
                    seenBoundary = true;
                }
            }

            return seenBoundary ? PointLocation.Boundary : PointLocation.Inside;
        }


        internal void FindNextPierced() {
            Debug.Assert(this.negativeSign < this.positiveSign);
            /*List<DebugCurve> l = null;
            if (db) {
                l = new List<DebugCurve>();
                l.Add(new DebugCurve(0.05, "red", new LineSegment(start, end)));
                l.Add(new DebugCurve(0.05, "blue", new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point)));
                l.Add(new DebugCurve(new Ellipse(3, 3, end)));

                var ii = currentTriangle.Edges.Index(piercedEdge);
                for (int j = ii + 1; j <= ii + 2; j++)
                    l.Add(new DebugCurve(0.05, "brown", new LineSegment(currentTriangle.Edges[j].upperSite.Point, currentTriangle.Edges[j].lowerSite.Point)));
            }*/
            this.currentTriangle = this.currentPiercedEdge.GetOtherTriangle(this.currentTriangle);
//            ShowDebug(null,currentPiercedEdge,currentTriangle);
            /* if (db) {
                 for (int j = 0; j <= 2; j++) {
                     var piercedEdge = currentTriangle.Edges[j];
                     if (piercedEdge == piercedEdge) continue;
                     l.Add(new DebugCurve(0.05, new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point)));
                 }

                 LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l.ToArray());
             }*/
            if (this.currentTriangle == null) {
                this.currentPiercedEdge = null;
                return;
            }
            var i = this.currentTriangle.Edges.Index(this.currentPiercedEdge);
            int j; //pierced index
            var oppositeSite = this.currentTriangle.Sites[i + 2];
            var oppositeSiteSign = this.GetHyperplaneSign(oppositeSite);
            if (this.negativeSign == 0) {
                Debug.Assert(this.positiveSign == 1);
                if (oppositeSiteSign == -1 || oppositeSiteSign == 0) {
                    this.negativeSign = oppositeSiteSign;
                    j = i + 1;
                }
                else {
                    j = i + 2;
                }
            }
            else if (this.positiveSign == 0) {
                Debug.Assert(this.negativeSign == -1);
                if (oppositeSiteSign == 1 || oppositeSiteSign == 0) {
                    this.positiveSign = oppositeSiteSign;
                    j = i + 2;
                }
                else {
                    j = i + 1;
                }
            }
            else if (oppositeSiteSign != this.positiveSign) {
                this.negativeSign = oppositeSiteSign;
                j = i + 1;
            }
            else {
                Debug.Assert(this.negativeSign != oppositeSiteSign);
                this.positiveSign = oppositeSiteSign;
                j = i + 2;
            }

            this.currentPiercedEdge =
                Point.SignedDoubledTriangleArea(this.end, this.currentTriangle.Sites[j].Point, this.currentTriangle.Sites[j + 1].Point) <
               - ApproximateComparer.DistanceEpsilon
                    ? this.currentTriangle.Edges[j]
                    : null;

//            ShowDebug(null,currentPiercedEdge, currentTriangle);

        }

//        void ShowDebug(IEnumerable<CdtTriangle> cdtTriangles, CdtEdge cdtEdge, CdtTriangle cdtTriangle) {
//            var l = new List<DebugCurve> { new DebugCurve(10,"red",new LineSegment(start,end)) };
//            if(cdtEdge!=null)
//                l.Add(new DebugCurve(100,3,"navy", new LineSegment(cdtEdge.upperSite.Point,cdtEdge.lowerSite.Point)));
//            AddTriangleToListOfDebugCurves(l,cdtTriangle,100,2,"brown");
//            LayoutAlgorithmSettings.ShowDebugCurvesEnumeration(l);
//
//        }
//        static void AddTriangleToListOfDebugCurves(List<DebugCurve> debugCurves,CdtTriangle triangle,byte transparency,double width,string color) {
//            foreach(var cdtEdge in triangle.Edges) {
//                debugCurves.Add(new DebugCurve(transparency,width,color,new LineSegment(cdtEdge.upperSite.Point,cdtEdge.lowerSite.Point)));
//            }
//        }

        internal int GetHyperplaneSign(CdtSite cdtSite) {
            var area = Point.SignedDoubledTriangleArea(this.start, cdtSite.Point, this.end);
            if (area > ApproximateComparer.DistanceEpsilon) {
                return 1;
            }

            if (area < -ApproximateComparer.DistanceEpsilon) {
                return -1;
            }

            return 0;
        }

        internal bool MoveNext() {
            if (this.currentPiercedEdge == null) {
                this.currentPiercedEdge = this.FindFirstPiercedEdge();
            } else {
                this.FindNextPierced();
            }

            return this.currentPiercedEdge != null;
        }
    }
}