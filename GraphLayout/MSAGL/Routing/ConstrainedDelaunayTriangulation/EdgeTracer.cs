using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation {
    internal class EdgeTracer {
        private readonly CdtEdge edge;
        private readonly Set<CdtTriangle> triangles;
        private readonly RbTree<CdtFrontElement> front;
        private readonly List<CdtSite> leftPolygon;
        private readonly List<CdtSite> rightPolygon;

        /// <summary>
        /// the upper site of the edge
        /// </summary>
        private CdtSite a;

        /// <summary>
        /// the lower site of the edge
        /// </summary>
        private CdtSite b;
        private CdtEdge piercedEdge;
        private CdtTriangle piercedTriangle;
        private RBNode<CdtFrontElement> piercedToTheLeftFrontElemNode;
        private RBNode<CdtFrontElement> piercedToTheRightFrontElemNode;
        private List<CdtFrontElement> elementsToBeRemovedFromFront = new List<CdtFrontElement>();
        private List<CdtTriangle> removedTriangles = new List<CdtTriangle>();

        public EdgeTracer(CdtEdge edge, Set<CdtTriangle> triangles, RbTree<CdtFrontElement> front, List<CdtSite> leftPolygon, List<CdtSite> rightPolygon) {
            this.edge = edge;
            this.triangles = triangles;
            this.front = front;
            this.leftPolygon = leftPolygon;
            this.rightPolygon = rightPolygon;
            this.a = edge.upperSite;
            this.b = edge.lowerSite;
        }

        public void Run() {
            this.Init();
            this.Traverse();
        }

        private void Traverse() {
            while (!this.BIsReached()) {
                if (this.piercedToTheLeftFrontElemNode != null) { this.ProcessLeftFrontPiercedElement(); }
                else if (this.piercedToTheRightFrontElemNode != null) {
                    this.ProcessRightFrontPiercedElement();
                }
                else {
                    this.ProcessPiercedEdge();
                }
            }
            if (this.piercedTriangle != null) {
                this.RemovePiercedTriangle(this.piercedTriangle);
            }

            this.FindMoreRemovedFromFrontElements();

            foreach (var elem in this.elementsToBeRemovedFromFront) {
                this.front.Remove(elem);
            }
        }

        private void ProcessLeftFrontPiercedElement() {
                       // CdtSweeper.ShowFront(triangles, front,new []{new LineSegment(a.Point, b.Point),new LineSegment(piercedToTheLeftFrontElemNode.Item.Edge.lowerSite.Point,piercedToTheLeftFrontElemNode.Item.Edge.upperSite.Point)},null);
            var v = this.piercedToTheLeftFrontElemNode;

            do {
                this.elementsToBeRemovedFromFront.Add(v.Item);
                this.AddSiteToLeftPolygon(v.Item.LeftSite);
                v = this.front.Previous(v);
            } while (Point.PointToTheLeftOfLine(v.Item.LeftSite.Point, this.a.Point, this.b.Point)); //that is why we are adding to the left polygon
            this.elementsToBeRemovedFromFront.Add(v.Item);
            this.AddSiteToRightPolygon(v.Item.LeftSite);
            if (v.Item.LeftSite == this.b) {
                this.piercedToTheLeftFrontElemNode = v; //this will stop the traversal
                return;
            }
            this.FindPiercedTriangle(v);
            this.piercedToTheLeftFrontElemNode = null;
        }

        private void FindPiercedTriangle(RBNode<CdtFrontElement> v) {
            var e = v.Item.Edge;
            var t = e.CcwTriangle ?? e.CwTriangle;
            var eIndex = t.Edges.Index(e);
            for (int i = 1; i <= 2; i++) {
                var ei = t.Edges[i + eIndex];
                var signedArea0 = ApproximateComparer.Sign(Point.SignedDoubledTriangleArea(ei.lowerSite.Point, this.a.Point, this.b.Point));
                var signedArea1 = ApproximateComparer.Sign(Point.SignedDoubledTriangleArea(ei.upperSite.Point, this.a.Point, this.b.Point));
                if (signedArea1 * signedArea0 <= 0) {
                    this.piercedTriangle = t;
                    this.piercedEdge = ei;
                    break;
                }
            }
        }

        private void FindMoreRemovedFromFrontElements() {
            foreach (var triangle in this.removedTriangles) {
                foreach (var e in triangle.Edges) {
                    if (e.CcwTriangle == null && e.CwTriangle == null) {
                        var site = e.upperSite.Point.X < e.lowerSite.Point.X ? e.upperSite : e.lowerSite;
                        var frontNode = CdtSweeper.FindNodeInFrontBySite(this.front, site);
                        if (frontNode.Item.Edge == e) {
                            this.elementsToBeRemovedFromFront.Add(frontNode.Item);
                        }
                    }
                }
            }
        }

        private void ProcessPiercedEdge() {
            //if(CdtSweeper.db)
              //          CdtSweeper.ShowFront(triangles, front, new[] { new LineSegment(a.Point, b.Point) },
                //                      new[] { new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point) });
            if (this.piercedEdge.CcwTriangle == this.piercedTriangle) {
                this.AddSiteToLeftPolygon(this.piercedEdge.lowerSite);
                this.AddSiteToRightPolygon(this.piercedEdge.upperSite);
            }
            else {
                this.AddSiteToLeftPolygon(this.piercedEdge.upperSite);
                this.AddSiteToRightPolygon(this.piercedEdge.lowerSite);
            }

            this.RemovePiercedTriangle(this.piercedTriangle);
            this.PrepareNextStateAfterPiercedEdge();
        }

        private void PrepareNextStateAfterPiercedEdge() {
            var t = this.piercedEdge.CwTriangle ?? this.piercedEdge.CcwTriangle;
            var eIndex = t.Edges.Index(this.piercedEdge);
            for (int i = 1; i <= 2; i++) {
                var e = t.Edges[i + eIndex];
                var signedArea0 = ApproximateComparer.Sign(Point.SignedDoubledTriangleArea(e.lowerSite.Point, this.a.Point, this.b.Point));
                var signedArea1 = ApproximateComparer.Sign(Point.SignedDoubledTriangleArea(e.upperSite.Point, this.a.Point, this.b.Point));
                if (signedArea1 * signedArea0 <= 0) {
                    if (e.CwTriangle != null && e.CcwTriangle != null) {
                        this.piercedTriangle = t;
                        this.piercedEdge = e;
                        break;
                    }
                    //e has to belong to the front, and its triangle has to be removed
                    this.piercedTriangle = null;
                    this.piercedEdge = null;
                    var leftSite = e.upperSite.Point.X < e.lowerSite.Point.X ? e.upperSite : e.lowerSite;
                    var frontElem = CdtSweeper.FindNodeInFrontBySite(this.front, leftSite);
                    Debug.Assert(frontElem != null);
                    if (leftSite.Point.X < this.a.Point.X) {
                        this.piercedToTheLeftFrontElemNode = frontElem;
                    } else {
                        this.piercedToTheRightFrontElemNode = frontElem;
                    }

                    this.RemovePiercedTriangle(e.CwTriangle ?? e.CcwTriangle);
                    break;
                }
            }
        }

        private void RemovePiercedTriangle(CdtTriangle t) {
            this.triangles.Remove(t);
            foreach (var e in t.Edges) {
                if (e.CwTriangle == t) {
                    e.CwTriangle = null;
                } else {
                    e.CcwTriangle = null;
                }
            }

            this.removedTriangles.Add(t);
        }

        private void ProcessRightFrontPiercedElement() {
            var v = this.piercedToTheRightFrontElemNode;

            do {
                this.elementsToBeRemovedFromFront.Add(v.Item);
                this.AddSiteToRightPolygon(v.Item.RightSite);
                v = this.front.Next(v);
            } while (Point.PointToTheRightOfLine(v.Item.RightSite.Point, this.a.Point, this.b.Point)); //that is why we are adding to the right polygon
            this.elementsToBeRemovedFromFront.Add(v.Item);
            this.AddSiteToLeftPolygon(v.Item.RightSite);
            if (v.Item.RightSite == this.b) {
                this.piercedToTheRightFrontElemNode = v; //this will stop the traversal
                return;
            }
            this.FindPiercedTriangle(v);
            this.piercedToTheRightFrontElemNode = null;
        }

        private void AddSiteToLeftPolygon(CdtSite site) {
            this.AddSiteToPolygonWithCheck(site, this.leftPolygon);
        }

        private void AddSiteToPolygonWithCheck(CdtSite site, List<CdtSite> list) {
            if (site == this.b) {
                return;
            }

            if (list.Count == 0 || list[list.Count - 1] != site) {
                list.Add(site);
            }
        }

        private void AddSiteToRightPolygon(CdtSite site) {
            this.AddSiteToPolygonWithCheck(site, this.rightPolygon);
        }

        private bool BIsReached() {
            var node = this.piercedToTheLeftFrontElemNode ?? this.piercedToTheRightFrontElemNode;
            if (node != null) {
                return node.Item.Edge.IsAdjacent(this.b);
            }

            return this.piercedEdge.IsAdjacent(this.b);
        }

        private void Init() {
//            if (CdtSweeper.D)
//                CdtSweeper.ShowFront(triangles, front, new[] {new LineSegment(a.Point, b.Point)},null);
                                     //new[] {new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point)});
            var frontElemNodeRightOfA = CdtSweeper.FindNodeInFrontBySite(this.front, this.a);
            var frontElemNodeLeftOfA = this.front.Previous(frontElemNodeRightOfA);
            if (Point.PointToTheLeftOfLine(this.b.Point, frontElemNodeLeftOfA.Item.LeftSite.Point, frontElemNodeLeftOfA.Item.RightSite.Point)) {
                this.piercedToTheLeftFrontElemNode = frontElemNodeLeftOfA;
            } else if (Point.PointToTheRightOfLine(this.b.Point, frontElemNodeRightOfA.Item.RightSite.Point, frontElemNodeRightOfA.Item.LeftSite.Point)) {
                this.piercedToTheRightFrontElemNode = frontElemNodeRightOfA;
            } else {
                foreach (var e in this.a.Edges) {
                    var t = e.CcwTriangle;
                    if (t == null) {
                        continue;
                    }

                    if (Point.PointToTheLeftOfLine(this.b.Point, e.lowerSite.Point, e.upperSite.Point)) {
                        continue;
                    }

                    var eIndex = t.Edges.Index(e);
                    var site = t.Sites[eIndex + 2];
                    if (Point.PointToTheLeftOfLineOrOnLine(this.b.Point, site.Point, e.upperSite.Point)) {
                        this.piercedEdge = t.Edges[eIndex + 1];
                        this.piercedTriangle = t;
//                                                CdtSweeper.ShowFront(triangles, front, new[] { new LineSegment(e.upperSite.Point, e.lowerSite.Point) }, 
//                                                    new[] { new LineSegment(piercedEdge.upperSite.Point, piercedEdge.lowerSite.Point) });
                        break;
                    }
                }
            }
        }


    }
}