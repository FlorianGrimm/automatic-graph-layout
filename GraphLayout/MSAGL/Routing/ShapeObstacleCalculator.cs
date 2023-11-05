using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing {
    /// <summary>
    /// The class calculates obstacles under the shape.
    /// We assume that the boundaries are not set for the shape children yet
    /// </summary>
    internal class ShapeObstacleCalculator {
        private RectangleNode<Polyline,Point> tightHierarchy;
        private RectangleNode<TightLooseCouple,Point> coupleHierarchy;

        public RectangleNode<Shape,Point> RootOfLooseHierarchy { get; set; }

        internal ShapeObstacleCalculator(Shape shape, double tightPadding, double loosePadding, 
            Dictionary<Shape, TightLooseCouple> shapeToTightLooseCouples) {
            this.MainShape = shape;
            this.TightPadding = tightPadding;
            this.LoosePadding = loosePadding;
            this.ShapesToTightLooseCouples = shapeToTightLooseCouples;
        }

        private Dictionary<Shape, TightLooseCouple> ShapesToTightLooseCouples { get; set; }

        private double TightPadding { get; set; }
        private double LoosePadding { get; set; }

        private Shape MainShape { get; set; }

        internal bool OverlapsDetected { get; set; }


        internal void Calculate() {
            if (this.MainShape.Children.Count() == 0) {
                return;
            }

            this.CreateTightObstacles();
            this.CreateTigthLooseCouples();
            this.FillTheMapOfShapeToTightLooseCouples();
        }

        private void FillTheMapOfShapeToTightLooseCouples() {
            var childrenShapeHierarchy =
                RectangleNode<Shape,Point>.CreateRectangleNodeOnEnumeration(
                    this.MainShape.Children.Select(s => new RectangleNode<Shape,Point>(s, s.BoundingBox)));
            RectangleNodeUtils.CrossRectangleNodes(childrenShapeHierarchy, this.coupleHierarchy,
                                                   this.TryMapShapeToTightLooseCouple);
        }

        private void TryMapShapeToTightLooseCouple(Shape shape, TightLooseCouple tightLooseCouple) {
            if (ShapeIsInsideOfPoly(shape, tightLooseCouple.TightPolyline)) {
                this.ShapesToTightLooseCouples[shape] = tightLooseCouple;
            }
#if TEST_MSAGL
            tightLooseCouple.LooseShape.UserData = (string) shape.UserData + "x";
#endif
        }


        /// <summary>
        /// this test is valid in our situation were the tight polylines are disjoint and the shape can cross only one of them
        /// </summary>
        /// <param name="shape"></param>
        /// <param name="tightPolyline"></param>
        /// <returns></returns>
        private static bool ShapeIsInsideOfPoly(Shape shape, Polyline tightPolyline) {
            return Curve.PointRelativeToCurveLocation(shape.BoundaryCurve.Start, tightPolyline) == PointLocation.Inside;
        }

        private void CreateTigthLooseCouples() {
            var couples = new List<TightLooseCouple>();

            foreach (var tightPolyline in this.tightHierarchy.GetAllLeaves()) {
                var distance = InteractiveObstacleCalculator.FindMaxPaddingForTightPolyline(this.tightHierarchy, tightPolyline, this.LoosePadding);
                var loosePoly = InteractiveObstacleCalculator.LoosePolylineWithFewCorners(tightPolyline, distance);
                couples.Add(new TightLooseCouple(tightPolyline, new Shape(loosePoly), distance));
            }
            this.coupleHierarchy = RectangleNode<TightLooseCouple,Point>.
                CreateRectangleNodeOnEnumeration(couples.Select(c => new RectangleNode<TightLooseCouple,Point>(c, c.TightPolyline.BoundingBox)));
        }

        private void CreateTightObstacles() {
            var tightObstacles = new Set<Polyline>(this.MainShape.Children.Select(this.InitialTightPolyline));
            int initialNumberOfTightObstacles = tightObstacles.Count;
            this.tightHierarchy = InteractiveObstacleCalculator.RemovePossibleOverlapsInTightPolylinesAndCalculateHierarchy(tightObstacles);
            this.OverlapsDetected = initialNumberOfTightObstacles > tightObstacles.Count;
        }

        private Polyline InitialTightPolyline(Shape shape) {
            var poly = InteractiveObstacleCalculator.PaddedPolylineBoundaryOfNode(shape.BoundaryCurve, this.TightPadding);
            var stickingPointsArray = this.LoosePolylinesUnderShape(shape).SelectMany(l => l).Where(
                p => Curve.PointRelativeToCurveLocation(p, poly) == PointLocation.Outside).ToArray();
            if (stickingPointsArray.Length <= 0) {
                return poly;
            }

            return new Polyline(
                ConvexHull.CalculateConvexHull(poly.Concat(stickingPointsArray))) {
                    Closed = true
                };
        }

        private IEnumerable<Polyline> LoosePolylinesUnderShape(Shape shape) {
            return shape.Children.Select(child => (Polyline)(this.ShapesToTightLooseCouples[child].LooseShape.BoundaryCurve));
        }
    }
}