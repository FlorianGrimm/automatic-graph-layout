using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Layout.Layered {
    //  internal delegate bool Direction(ref Point a, ref Point b, ref Point c);
    internal delegate IEnumerable<Point> Points();

    internal class RefinerBetweenTwoLayers {
        private int topNode;
        private int bottomNode;
        private CornerSite topSite;
        private CornerSite bottomSite;
        private CornerSite currentTopSite;
        private CornerSite currentBottomSite;
        private LayerArrays layerArrays;
        private ProperLayeredGraph layeredGraph;
        private GeometryGraph originalGraph;
        private Points topCorners;
        private Points bottomCorners;
        private Anchor[] anchors;
        private Random random = new Random(1);
        private double layerSeparation;

        private RefinerBetweenTwoLayers(
                int topNodeP,
                int bottomNodeP,
                CornerSite topSiteP,
                LayerArrays layerArraysP,
                ProperLayeredGraph layeredGraphP, GeometryGraph originalGraphP, Anchor[] anchorsP, double layerSeparation) {
            this.topNode = topNodeP;
            this.bottomNode = bottomNodeP;
            this.topSite = topSiteP;
            this.bottomSite = topSiteP.Next;
            this.currentTopSite = topSiteP;
            this.currentBottomSite = topSiteP.Next;
            this.layerArrays = layerArraysP;
            this.layeredGraph = layeredGraphP;
            this.originalGraph = originalGraphP;
            this.anchors = anchorsP;
            this.layerSeparation = layerSeparation;
        }

        internal static void Refine(
            int topNodeP,
            int bottomNode,
            CornerSite topSiteP,
            Anchor[] anchors,
            LayerArrays layerArraysP,
            ProperLayeredGraph layeredGraph,
            GeometryGraph originalGraph,
            double layerSeparation) {
            RefinerBetweenTwoLayers refiner = new RefinerBetweenTwoLayers(topNodeP,
                                                                          bottomNode, topSiteP, layerArraysP,
                                                                          layeredGraph, originalGraph, anchors,
                                                                          layerSeparation);
            refiner.Refine();
        }

        private void Refine() {
            this.Init();
            while (this.InsertSites()) {
                ;
            }
        }

        private Point FixCorner(Point start, Point corner, Point end) {
            if (start == corner) {
                return corner;
            }

            Point a = Point.ClosestPointAtLineSegment(corner, start, end);
            Point offsetInTheChannel = corner - a;
            double y = Math.Abs(offsetInTheChannel.Y);
            double sep = this.layerSeparation / 2.0;
            if (y > sep) //push the return value closer to the corner
{
                offsetInTheChannel *= (sep / (y * 2));
            }

            return offsetInTheChannel + corner;
        }


        private bool InsertSites() {
            if (this.random.Next(2) == 0) {
                return this.CalculateNewTopSite() | this.CalculateNewBottomSite();
            } else {
                return this.CalculateNewBottomSite() | this.CalculateNewTopSite();
            }
        }

        /// <summary>
        /// circimvating from the side
        /// </summary>
        /// <returns></returns>
        private bool CalculateNewBottomSite() {
            Point mainSeg = this.currentBottomSite.Point - this.currentTopSite.Point;
            double cotan = AbsCotan(mainSeg);
            Point vOfNewSite = new Point();//to silence the compiler
            bool someBottomCorners = false;
            foreach (Point p in this.bottomCorners()) {
                double cornerCotan = AbsCotan(p - this.currentBottomSite.Point);
                if (cornerCotan < cotan) {
                    cotan = cornerCotan;
                    vOfNewSite = p;
                    someBottomCorners = true;
                }
            }

            if (!someBottomCorners) {
                return false;
            }

            if (!ApproximateComparer.Close(cotan, AbsCotan(mainSeg))) {
                this.currentBottomSite = new CornerSite(this.currentTopSite, this.FixCorner(this.currentTopSite.Point, vOfNewSite, this.currentBottomSite.Point), this.currentBottomSite);//consider a different FixCorner
                return true;
            }

            return false; //no progress


        }

        private static double AbsCotan(Point mainSeg) {
            return Math.Abs(mainSeg.X / mainSeg.Y);
        }

        private bool CalculateNewTopSite() {
            Point mainSeg = this.currentBottomSite.Point - this.currentTopSite.Point;
            double cotan = AbsCotan(mainSeg);
            Point vOfNewSite = new Point();//to silence the compiler
            bool someTopCorners = false;
            foreach (Point p in this.topCorners()) {
                double cornerCotan = AbsCotan(p - this.currentTopSite.Point);
                if (cornerCotan < cotan) {
                    cotan = cornerCotan;
                    vOfNewSite = p;
                    someTopCorners = true;
                }
            }
            if (!someTopCorners) {
                return false;
            }

            if (!ApproximateComparer.Close(cotan, AbsCotan(mainSeg))) {
                this.currentTopSite = new CornerSite(this.currentTopSite,
                    this.FixCorner(this.currentTopSite.Point, vOfNewSite, this.currentBottomSite.Point),
                    this.currentBottomSite
                    );//consider a different FixCorner
                return true;
            }

            return false; //no progress
        }

        //private Site AvoidBottomLayer() {
        //    Point corner;
        //    if (StickingCornerFromTheBottomLayer(out corner)) {
        //        corner = FixCorner(this.currentTopSite.v, corner, this.currentBottomSite.v);
        //        return new Site(this.currentTopSite, corner, this.currentBottomSite);
        //    } else
        //        return null;
        //}

        //private Site AvoidTopLayer() {
        //    Point corner;
        //    if (StickingCornerFromTheTopLayer(out corner)) {
        //        corner = FixCorner(this.currentTopSite.v, corner, this.currentBottomSite.v);
        //        return new Site(this.currentTopSite, corner, this.currentBottomSite);
        //    } else
        //        return null;
        //}

        //private bool StickingCornerFromTheTopLayer(out Point corner) {
        //    corner = this.currentBottomSite.v;
        //    foreach (Point l in this.topCorners()) {
        //        Point p = l;
        //        if (this.counterClockwise(ref currentTopSite.v, ref p, ref corner)) 
        //            corner = p;
        //    }
        //    return corner != this.currentBottomSite.v;
        //}
        //private bool StickingCornerFromTheBottomLayer(out Point corner) {
        //    corner = this.currentTopSite.v;
        //    foreach (Point l in this.bottomCorners()) {
        //        Point p = l;
        //        if (this.counterClockwise(ref currentBottomSite.v, ref p, ref corner))
        //            corner = p;
        //    }
        //    return corner != this.currentTopSite.v;
        //}

        private void Init() {
            if (this.IsTopToTheLeftOfBottom()) {
                this.topCorners = new Points(this.CornersToTheRightOfTop);
                this.bottomCorners = new Points(this.CornersToTheLeftOfBottom);
            } else {
                this.topCorners = new Points(this.CornersToTheLeftOfTop);
                this.bottomCorners = new Points(this.CornersToTheRightOfBottom);
            }
        }

        private bool IsTopToTheLeftOfBottom() {
            return (this.topSite.Point.X < this.topSite.Next.Point.X);
        }

        private IEnumerable<Point> NodeCorners(int node) {
            foreach (Point p in this.NodeAnchor(node).PolygonalBoundary) {
                yield return p;
            }
        }

        private Anchor NodeAnchor(int node) {
            return this.anchors[node];
        }

        private IEnumerable<Point> CornersToTheLeftOfBottom() {
            int bottomPosition = this.layerArrays.X[this.bottomNode];
            double leftMost = this.currentTopSite.Point.X;
            double rightMost = this.currentBottomSite.Point.X;
            foreach (int node in this.LeftFromTheNode(this.NodeLayer(this.bottomNode), bottomPosition,
                NodeKind.Bottom, leftMost, rightMost)) {
                foreach (Point p in this.NodeCorners(node)) {
                    if (p.Y > this.currentBottomSite.Point.Y && PossibleCorner(leftMost, rightMost, p)) {
                        yield return p;
                    }
                }
            }
        }

        private IEnumerable<Point> CornersToTheLeftOfTop() {
            int topPosition = this.layerArrays.X[this.topNode];
            double leftMost = this.currentBottomSite.Point.X;
            double rightMost = this.currentTopSite.Point.X;
            foreach (int node in this.LeftFromTheNode(this.NodeLayer(this.topNode), topPosition, NodeKind.Top, leftMost, rightMost)) {
                foreach (Point p in this.NodeCorners(node)) {
                    if (p.Y < this.currentTopSite.Point.Y && PossibleCorner(leftMost, rightMost, p)) {
                        yield return p;
                    }
                }
            }
        }

        private IEnumerable<Point> CornersToTheRightOfBottom() {
            int bottomPosition = this.layerArrays.X[this.bottomNode];
            double leftMost = this.currentBottomSite.Point.X;
            double rightMost = this.currentTopSite.Point.X;

            foreach (int node in this.RightFromTheNode(this.NodeLayer(this.bottomNode), bottomPosition,
                NodeKind.Bottom, leftMost, rightMost)) {
                foreach (Point p in this.NodeCorners(node)) {
                    if (p.Y > this.currentBottomSite.Point.Y && PossibleCorner(leftMost, rightMost, p)) {
                        yield return p;
                    }
                }
            }
        }

        private IEnumerable<Point> CornersToTheRightOfTop() {
            int topPosition = this.layerArrays.X[this.topNode];
            double leftMost = this.currentTopSite.Point.X;
            double rightMost = this.currentBottomSite.Point.X;
            foreach (int node in this.RightFromTheNode(this.NodeLayer(this.topNode), topPosition, NodeKind.Top, leftMost, rightMost)) {
                foreach (Point p in this.NodeCorners(node)) {
                    if (p.Y < this.currentTopSite.Point.Y && PossibleCorner(leftMost, rightMost, p)) {
                        yield return p;
                    }
                }
            }
        }

        private static bool PossibleCorner(double leftMost, double rightMost, Point p) {
            return p.X > leftMost && p.X < rightMost;
        }

        private int[] NodeLayer(int j) {
            return this.layerArrays.Layers[this.layerArrays.Y[j]];
        }

        //private static bool CounterClockwise(ref Point topPoint, ref Point cornerPoint, ref Point p) {
        //    return Point.GetTriangleOrientation(topPoint, cornerPoint, p) == TriangleOrientation.Counterclockwise;
        //}

        //private static bool Clockwise(ref Point topPoint, ref Point cornerPoint, ref Point p) {
        //    return Point.GetTriangleOrientation(topPoint, cornerPoint, p) == TriangleOrientation.Clockwise;
        //}

        private bool IsLabel(int u) {
            return this.anchors[u].RepresentsLabel;
        }

        private bool NodeUCanBeCrossedByNodeV(int u, int v) {
            if (this.IsLabel(u) || this.IsLabel(v)) {
                return false;
            }

            if (this.IsVirtualVertex(u) && this.IsVirtualVertex(v) && this.AdjacentEdgesIntersect(u, v)) {
                return true;
            }

            return false;
        }

        private bool AdjacentEdgesIntersect(int u, int v) {
            return this.Intersect(this.IncomingEdge(u), this.IncomingEdge(v)) || this.Intersect(this.OutcomingEdge(u), this.OutcomingEdge(v));
        }

        private bool Intersect(LayerEdge e, LayerEdge m) {
            return (this.layerArrays.X[e.Source] - this.layerArrays.X[m.Source]) * (this.layerArrays.X[e.Target] - this.layerArrays.X[m.Target]) < 0;
        }

        private LayerEdge IncomingEdge(int u) {
            foreach (LayerEdge le in this.layeredGraph.InEdges(u)) {
                return le;
            }

            throw new InvalidOperationException();
        }
        //here u is a virtual vertex
        private LayerEdge OutcomingEdge(int u) {
            foreach (LayerEdge le in this.layeredGraph.OutEdges(u)) {
                return le;
            }

            throw new InvalidOperationException();
        }

        private bool IsVirtualVertex(int v) {
            return v >= this.originalGraph.Nodes.Count;
        }

        private IEnumerable<int> RightFromTheNode(int[] layer, int vPosition, NodeKind nodeKind, double leftMostX, double rightMostX) {
            double t = 0, b = 0;
            if (nodeKind == NodeKind.Bottom) {
                b = Single.MaxValue;//we don't have bottom boundaries here since they will be cut off
            } else if (nodeKind == NodeKind.Top) {
                t = Single.MaxValue;//we don't have top boundaries here since they will be cut off
            }

            int v = layer[vPosition];

            for (int i = vPosition + 1; i < layer.Length; i++) {
                int u = layer[i];
                if (this.NodeUCanBeCrossedByNodeV(u, v)) {
                    continue;
                }

                Anchor anchor = this.anchors[u];
                if (anchor.Left >= rightMostX) {
                    break;
                }

                if (anchor.Right > leftMostX) {
                    if (anchor.TopAnchor > t + ApproximateComparer.DistanceEpsilon) {
                        t = anchor.TopAnchor;
                        yield return u;
                    } else if (anchor.BottomAnchor > b + ApproximateComparer.DistanceEpsilon) {
                        b = anchor.BottomAnchor;
                        yield return u;
                    }
                }
            }
        }

        private IEnumerable<int> LeftFromTheNode(int[] layer, int vPosition, NodeKind nodeKind, double leftMostX, double rightMostX) {
            double t = 0, b = 0;
            if (nodeKind == NodeKind.Bottom) {
                b = Single.MaxValue;//we don't have bottom boundaries here since they will be cut off
            } else if (nodeKind == NodeKind.Top) {
                t = Single.MaxValue;//we don't have top boundaries here since they will be cut off
            }

            int v = layer[vPosition];

            for (int i = vPosition - 1; i > -1; i--) {
                int u = layer[i];
                if (this.NodeUCanBeCrossedByNodeV(u, v)) {
                    continue;
                }

                Anchor anchor = this.anchors[u];
                if (anchor.Right <= leftMostX) {
                    break;
                }

                if (anchor.Left < rightMostX) {
                    if (anchor.TopAnchor > t + ApproximateComparer.DistanceEpsilon) {
                        t = anchor.TopAnchor;
                        yield return u;
                    } else if (anchor.BottomAnchor > b + ApproximateComparer.DistanceEpsilon) {
                        b = anchor.BottomAnchor;
                        yield return u;
                    }
                }
            }
        }
    }
}
