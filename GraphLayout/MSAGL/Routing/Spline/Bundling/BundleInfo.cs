using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using System.Diagnostics;
using System;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class BundleInfo {
        private const double FeasibleWidthEpsilon = 0.1; //??

        internal readonly BundleBase SourceBase;
        internal readonly BundleBase TargetBase;
        private readonly Set<Polyline> obstaclesToIgnore;
        readonly internal double EdgeSeparation;
        readonly internal double[] HalfWidthArray;
        private readonly double longEnoughSideLength;
        private List<Polyline> tightObstaclesInTheBoundingBox;
        internal double TotalRequiredWidth;

        internal BundleInfo(BundleBase sourceBase, BundleBase targetBase, Set<Polyline> obstaclesToIgnore, double edgeSeparation, double[] halfWidthArray) {
            this.SourceBase = sourceBase;
            this.TargetBase = targetBase;
            this.obstaclesToIgnore = obstaclesToIgnore;
            this.EdgeSeparation = edgeSeparation;
            this.HalfWidthArray = halfWidthArray;
            this.TotalRequiredWidth = this.EdgeSeparation * (this.HalfWidthArray.Length-1) + this.HalfWidthArray.Sum() * 2;
            this.longEnoughSideLength = new Rectangle(sourceBase.Curve.BoundingBox, targetBase.Curve.BoundingBox).Diagonal;

            //sometimes TotalRequiredWidth is too large to fit into the circle, so we evenly scale everything
            double mn = Math.Max(sourceBase.Curve.BoundingBox.Diagonal, targetBase.Curve.BoundingBox.Diagonal);
            if (this.TotalRequiredWidth > mn) {
                double scale = this.TotalRequiredWidth / mn;
                for (int i = 0; i < this.HalfWidthArray.Length; i++) {
                    this.HalfWidthArray[i] /= scale;
                }

                this.TotalRequiredWidth /= scale;
                this.EdgeSeparation /= scale;
            }
        }

        internal void SetParamsFeasiblySymmetrically(RectangleNode<Polyline, Point> tightTree) {
            this.CalculateTightObstaclesForBundle(tightTree, this.obstaclesToIgnore);
            this.SetEndParamsSymmetrically();
        }

        private void CalculateTightObstaclesForBundle(RectangleNode<Polyline, Point> tightTree, Set<Polyline> obstaclesToIgnore) {
            double sRadius = this.SourceBase.Curve.BoundingBox.Diagonal / 2;
            double tRadius = this.TargetBase.Curve.BoundingBox.Diagonal / 2;

            //Polyline bundle = Intersections.Create4gon(SourceBase.CurveCenter, TargetBase.CurveCenter, sRadius * 2, tRadius * 2);
            Polyline bundle = Intersections.Create4gon(this.SourceBase.Position, this.TargetBase.Position, sRadius * 2, tRadius * 2);

            this.tightObstaclesInTheBoundingBox = tightTree.AllHitItems(bundle.BoundingBox,
                p => !obstaclesToIgnore.Contains(p) && Curve.ClosedCurveInteriorsIntersect(bundle, p)).ToList();
        }

        private void SetEndParamsSymmetrically() {
            Point targetPos = this.TargetBase.Position;
            Point sourcePos = this.SourceBase.Position;

            var dir = (targetPos - sourcePos).Normalize();
            var perp = dir.Rotate90Ccw();
            var middle = 0.5 * (targetPos + sourcePos);
            var a = middle + this.longEnoughSideLength * dir;
            var b = middle - this.longEnoughSideLength * dir; // [a,b] is a long enough segment

            //we are already fine
            if (this.SetRLParamsIfWidthIsFeasible(this.TotalRequiredWidth * perp / 2, a, b)) {
                this.SetInitialMidParams();
                return;
            }

            //find the segment using binary search
            var uw = this.TotalRequiredWidth;
            var lw = 0.0;
            var mw = uw / 2;
            while (uw - lw > FeasibleWidthEpsilon) {
                if (this.SetRLParamsIfWidthIsFeasible(mw * perp / 2, a, b)) {
                    lw = mw;
                } else {
                    uw = mw;
                }

                mw = 0.5 * (uw + lw);
            }

            if (mw <= FeasibleWidthEpsilon) {
                //try one side
                if (this.SetRLParamsIfWidthIsFeasibleTwoPerps(2 * FeasibleWidthEpsilon * perp / 2, new Point(), a, b)) {
                    mw = 2 * FeasibleWidthEpsilon;
                } else if (this.SetRLParamsIfWidthIsFeasibleTwoPerps(new Point(), -2 * FeasibleWidthEpsilon * perp / 2, a, b)) {
                    mw = 2 * FeasibleWidthEpsilon;
                }
            }

            Debug.Assert(mw > FeasibleWidthEpsilon);

            this.SourceBase.InitialMidParameter = this.SourceBase.AdjustParam(this.SourceBase.ParRight + this.SourceBase.Span / 2);
            this.TargetBase.InitialMidParameter = this.TargetBase.AdjustParam(this.TargetBase.ParRight + this.TargetBase.Span / 2);
        }

        private bool SetRLParamsIfWidthIsFeasible(Point perp, Point a, Point b) {
            return this.SetRLParamsIfWidthIsFeasibleTwoPerps(perp, -perp, a, b);
        }

        private bool SetRLParamsIfWidthIsFeasibleTwoPerps(Point perpL, Point perpR, Point a, Point b) {
            double sourceRParam, targetRParam, sourceLParam, targetLParam;
            var ls = this.TrimSegWithBoundaryCurves(new LineSegment(a + perpL, b + perpL), out sourceLParam, out targetRParam);
            if (ls == null) {
                return false;
            }

            if (this.tightObstaclesInTheBoundingBox.Any(t => Intersections.LineSegmentIntersectPolyline(ls.Start, ls.End, t))) {
                return false;
            }

            ls = this.TrimSegWithBoundaryCurves(new LineSegment(a + perpR, b + perpR), out sourceRParam, out targetLParam);
            if (ls == null) {
                return false;
            }

            if (this.tightObstaclesInTheBoundingBox.Any(t => Intersections.LineSegmentIntersectPolyline(ls.Start, ls.End, t))) {
                return false;
            }

            if (this.SourceBase.IsParent) {
                this.SourceBase.ParRight = sourceLParam;
                this.SourceBase.ParLeft = sourceRParam;
            }
            else {
                this.SourceBase.ParRight = sourceRParam;
                this.SourceBase.ParLeft = sourceLParam;
            }

            //SourceBase.InitialMidParameter = SourceBase.AdjustParam(SourceBase.ParRight + SourceBase.Span / 2);

            if (this.TargetBase.IsParent) {
                this.TargetBase.ParRight = targetLParam;
                this.TargetBase.ParLeft = targetRParam;
            }
            else {
                this.TargetBase.ParRight = targetRParam;
                this.TargetBase.ParLeft = targetLParam;
            }

            //TargetBase.InitialMidParameter = TargetBase.AdjustParam(TargetBase.ParRight + TargetBase.Span / 2);

            return true;
        }

        private void SetInitialMidParams() {
            double sourceParam, targetParam;
            this.TrimSegWithBoundaryCurves(new LineSegment(this.TargetBase.CurveCenter, this.SourceBase.CurveCenter), out sourceParam, out targetParam);

            this.SourceBase.InitialMidParameter = sourceParam;
            this.TargetBase.InitialMidParameter = targetParam;
        }

        private LineSegment TrimSegWithBoundaryCurves(LineSegment ls, out double sourcePar, out double targetPar) {
            //ls goes from target to source
            var inters = Curve.GetAllIntersections(ls, this.SourceBase.Curve, true);
            if (inters.Count == 0) {
                sourcePar = targetPar = 0;
                return null;
            }
            IntersectionInfo i0;
            if (inters.Count == 1) {
                i0 = inters[0];
            } else {
                if (!this.SourceBase.IsParent) {
                    i0 = inters[0].Par0 < inters[1].Par0 ? inters[0] : inters[1];
                } else {
                    i0 = inters[0].Par0 < inters[1].Par0 ? inters[1] : inters[0];
                }
            }

            inters = Curve.GetAllIntersections(ls, this.TargetBase.Curve, true);
            if (inters.Count == 0) {
                sourcePar = targetPar = 0;
                return null;
            }
            IntersectionInfo i1;
            if (inters.Count == 1) {
                i1 = inters[0];
            } else {
                if (!this.TargetBase.IsParent) {
                    i1 = inters[0].Par0 > inters[1].Par0 ? inters[0] : inters[1];
                } else {
                    i1 = inters[0].Par0 > inters[1].Par0 ? inters[1] : inters[0];
                }
            }

            sourcePar = i0.Par1;
            targetPar = i1.Par1;
            return new LineSegment(i0.IntersectionPoint, i1.IntersectionPoint);
        }

        internal void RotateBy(int rotationOfSourceRightPoint, int rotationOfSourceLeftPoint, int rotationOfTargetRightPoint, int rotationOfTargetLeftPoint, double parameterChange) {
            bool needToUpdateSource = rotationOfSourceRightPoint != 0 || rotationOfSourceLeftPoint != 0;
            bool needToUpdateTarget = rotationOfTargetRightPoint != 0 || rotationOfTargetLeftPoint != 0;

            if (needToUpdateSource) {
                this.SourceBase.RotateBy(rotationOfSourceRightPoint, rotationOfSourceLeftPoint, parameterChange);
            }

            if (needToUpdateTarget) {
                this.TargetBase.RotateBy(rotationOfTargetRightPoint, rotationOfTargetLeftPoint, parameterChange);
            }

            this.UpdateSourceAndTargetBases(needToUpdateSource, needToUpdateTarget);
        }

        internal void UpdateSourceAndTargetBases(bool sourceChanged, bool targetChanged) {
            if (sourceChanged) {
                this.UpdatePointsOnBundleBase(this.SourceBase);
            }

            if (targetChanged) {
                this.UpdatePointsOnBundleBase(this.TargetBase);
            }

            this.UpdateTangentsOnBases();
        }

        private void UpdateTangentsOnBases() {
            int count = this.TargetBase.Count;
            //updating tangents
            for (int i = 0; i < count; i++) {
                Point d = this.TargetBase.Points[i] - this.SourceBase.Points[count - 1 - i];
                double len = d.Length;
                if (len >= ApproximateComparer.Tolerance) {
                    d /= len;
                    this.TargetBase.Tangents[i] = d;
                    this.SourceBase.Tangents[count - 1 - i] = d.Negate();
                }
            }
        }

        private void UpdatePointsOnBundleBase(BundleBase bb) {
            int count = bb.Count;

            Point[] pns = bb.Points;
            var ls = new LineSegment(bb.LeftPoint, bb.RightPoint);
            var scale = 1 / this.TotalRequiredWidth;
            var t = this.HalfWidthArray[0];
            pns[0] = ls[t*scale];
            for (int i = 1; i < count; i++) {
                t += this.HalfWidthArray[i-1]+ this.EdgeSeparation + this.HalfWidthArray[i];
                pns[i] = ls[t * scale];
            }
        }

        internal bool RotationIsLegal(int rotationOfSourceRightPoint, int rotationOfSourceLeftPoint,
            int rotationOfTargetRightPoint, int rotationOfTargetLeftPoint, double parameterChange) {
            //1. we can't have intersections with obstacles
            //(we check borderlines of the bundle only)
            if (!this.SourceBase.IsParent && !this.TargetBase.IsParent) {
                if (rotationOfSourceLeftPoint != 0 || rotationOfTargetRightPoint != 0) {
                    Point rSoP = this.SourceBase.RotateLeftPoint(rotationOfSourceLeftPoint, parameterChange);
                    Point lTarP = this.TargetBase.RotateRigthPoint(rotationOfTargetRightPoint, parameterChange);
                    if (!this.LineIsLegal(rSoP, lTarP)) {
                        return false;
                    }
                }

                if (rotationOfSourceRightPoint != 0 || rotationOfTargetLeftPoint != 0) {
                    Point lSoP = this.SourceBase.RotateRigthPoint(rotationOfSourceRightPoint, parameterChange);
                    Point rTarP = this.TargetBase.RotateLeftPoint(rotationOfTargetLeftPoint, parameterChange);
                    if (!this.LineIsLegal(lSoP, rTarP)) {
                        return false;
                    }
                }
            }
            else {
                if (rotationOfSourceLeftPoint != 0 || rotationOfTargetLeftPoint != 0) {
                    Point lSoP = this.SourceBase.RotateLeftPoint(rotationOfSourceLeftPoint, parameterChange);
                    Point lTarP = this.TargetBase.RotateLeftPoint(rotationOfTargetLeftPoint, parameterChange);
                    if (!this.LineIsLegal(lSoP, lTarP)) {
                        return false;
                    }
                }

                if (rotationOfSourceRightPoint != 0 || rotationOfTargetRightPoint != 0) {
                    Point rSoP = this.SourceBase.RotateRigthPoint(rotationOfSourceRightPoint, parameterChange);
                    Point rTarP = this.TargetBase.RotateRigthPoint(rotationOfTargetRightPoint, parameterChange);
                    if (!this.LineIsLegal(rSoP, rTarP)) {
                        return false;
                    }
                }
            }

            //2. we are also not allowed to change the order of bundles around a hub
            if (rotationOfSourceRightPoint != 0 || rotationOfSourceLeftPoint != 0) {
                if (!this.SourceBase.RelativeOrderOfBasesIsPreserved(rotationOfSourceRightPoint, rotationOfSourceLeftPoint, parameterChange)) {
                    return false;
                }
            }

            if (rotationOfTargetRightPoint != 0 || rotationOfTargetLeftPoint != 0) {
                if (!this.TargetBase.RelativeOrderOfBasesIsPreserved(rotationOfTargetRightPoint, rotationOfTargetLeftPoint, parameterChange)) {
                    return false;
                }
            }

            return true;
        }

        private bool LineIsLegal(Point a, Point b) {
            return this.tightObstaclesInTheBoundingBox.All(t => !Intersections.LineSegmentIntersectPolyline(a, b, t));
        }
    }
}
