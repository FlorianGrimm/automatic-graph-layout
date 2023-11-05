using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using System.Diagnostics;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class BundleBase {

        /// <summary>
        /// only one of those is not null
        /// </summary>
        internal BundleInfo OutgoingBundleInfo;

        internal BundleInfo IncomingBundleInfo;
        private readonly Point[] points;
        private readonly Point[] tangents;

        internal OrientedHubSegment[] OrientedHubSegments;

        /// <summary>
        /// boundary of cluster or hub containing this base 
        /// </summary>
        internal ICurve Curve;

        /// <summary>
        /// this bundle base sits on a cluster boundary and the opposite base sits on a child of the cluster 
        /// </summary>
        internal bool IsParent;

        /// <summary>
        /// if true then the base sits on a real node or cluster, otherwise it belongs to an intermediate hub
        /// </summary>
        internal bool BelongsToRealNode;

        /// <summary>
        /// position of the station containing the base
        /// (could be a center of a hub, or a point on the boundary of a cluster)
        /// </summary>
        internal Point Position;

        //need for debug only
        internal int stationIndex;

        /// <summary>
        /// constructor
        /// </summary>
        internal BundleBase(int count, ICurve boundaryCurve, Point position, bool belongsToRealNode, int stationIndex) {
            this.BelongsToRealNode = belongsToRealNode;
            this.Curve = boundaryCurve;
            this.Position = position;
            this.stationIndex = stationIndex;
            this.points = new Point[count];
            this.tangents = new Point[count];
            this.OrientedHubSegments = new OrientedHubSegment[count];
            this.ParameterSpan = this.Curve.ParEnd - this.Curve.ParStart;
        }

        internal Point CurveCenter { get { return this.Curve.BoundingBox.Center; } }

        internal BundleBase OppositeBase { get { return this.OutgoingBundleInfo != null ? this.OutgoingBundleInfo.TargetBase : this.IncomingBundleInfo.SourceBase; } }

        internal int Count { get { return this.points.Length; } }

        internal Point[] Points { get { return this.points; } }

        internal Point[] Tangents { get { return this.tangents; } }

        private double initialMidParameter;

        internal double InitialMidParameter {
            get { return this.initialMidParameter; }
            set {
                this.initialMidParameter = value;
                this.InitialMidPoint = this.Curve[value];
            }
        }

        internal Point InitialMidPoint { get; set; }

        private double parRight;
        /// <summary>
        /// corresponds to the left point of the base
        /// </summary>
        internal double ParRight {
            get { return this.parRight; }
            set {
                this.parRight = value;
                this.RightPoint = this.Curve[this.parRight];
            }
        }

        private double parLeft;
        /// <summary>
        /// corresponds to the right point of the base
        /// </summary>
        internal double ParLeft {
            get { return this.parLeft; }
            set {
                this.parLeft = value;
                this.LeftPoint = this.Curve[this.parLeft];
            }
        }

        internal double ParMid {
            get { return (this.parRight + this.parLeft) / 2; }
        }

        internal Point RightPoint { get; set; }

        internal Point LeftPoint { get; set; }

        internal Point MidPoint {
            get { return (this.RightPoint + this.LeftPoint) / 2; }
        }

        //previous in ccw order
        internal BundleBase Prev;
        //next in ccw order
        internal BundleBase Next;

        internal double ParameterSpan;

        internal double Span { get { return this.SpanBetweenTwoPoints(this.parRight, this.parLeft); } }

        internal double SpanBetweenTwoPoints(double right, double left) {
            return (right <= left ? left - right : left - right + this.ParameterSpan);
        }

        internal Point RotateLeftPoint(int rotationOfSourceLeftPoint, double parameterChange) {
            if (rotationOfSourceLeftPoint == 0) {
                return this.LeftPoint;
            }

            return this.RotatePoint(rotationOfSourceLeftPoint, this.parLeft, parameterChange);
        }

        internal Point RotateRigthPoint(int rotationOfSourceRightPoint, double parameterChange) {
            if (rotationOfSourceRightPoint == 0) {
                return this.RightPoint;
            }

            return this.RotatePoint(rotationOfSourceRightPoint, this.parRight, parameterChange);
        }

        private Point RotatePoint(int rotation, double t, double parameterChange) {
            double change = this.ParameterSpan * parameterChange;

            t = t + rotation * change;
            t = this.AdjustParam(t);

            return this.Curve[t];
        }

        internal double AdjustParam(double t) {
            if (t > this.Curve.ParEnd) {
                t = this.Curve.ParStart + (t - this.Curve.ParEnd);
            } else if (t < this.Curve.ParStart) {
                t = this.Curve.ParEnd - (this.Curve.ParStart - t);
            }

            return t;
        }

        internal void RotateBy(int rotationOfRightPoint, int rotationOfLeftPoint, double parameterChange) {
            double change = this.ParameterSpan * parameterChange;
            if (rotationOfRightPoint != 0) {
                this.ParRight = this.AdjustParam(this.ParRight + rotationOfRightPoint * change);
            }

            if (rotationOfLeftPoint != 0) {
                this.ParLeft = this.AdjustParam(this.ParLeft + rotationOfLeftPoint * change);
            }
        }

        internal bool Intersect(BundleBase other) {
            return this.Intersect(this.parRight, this.parLeft, other.parRight, other.parLeft);
        }

        internal bool Intersect(double lParRight, double lParLeft, double rParRight, double rParLeft) {
            if (lParRight > lParLeft) {
                return this.Intersect(lParRight, this.Curve.ParEnd, rParRight, rParLeft) || this.Intersect(this.Curve.ParStart, lParLeft, rParRight, rParLeft);
            }

            if (rParRight > rParLeft) {
                return this.Intersect(lParRight, lParLeft, rParRight, this.Curve.ParEnd) || this.Intersect(lParRight, lParLeft, this.Curve.ParStart, rParLeft);
            }

            Debug.Assert(lParRight <= lParLeft);
            Debug.Assert(rParRight <= rParLeft);
            if (ApproximateComparer.LessOrEqual(lParLeft, rParRight)) {
                return false;
            }

            if (ApproximateComparer.LessOrEqual(rParLeft, lParRight)) {
                return false;
            }

            return true;
        }

        internal bool RelativeOrderOfBasesIsPreserved(int rotationOfRightPoint, int rotationOfLeftPoint, double parameterChange) {
            double change = this.ParameterSpan * parameterChange;

            //we do not swap parRight and parLeft
            double rnew = this.parRight + rotationOfRightPoint * change;
            double lnew = (this.parRight < this.parLeft ? this.parLeft + rotationOfLeftPoint * change : this.parLeft + this.ParameterSpan + rotationOfLeftPoint * change);
            if (rnew > lnew) {
                return false;
            }

            //span could not be greater than pi
            if (this.SpanBetweenTwoPoints(rnew, lnew) > this.ParameterSpan / 2.0) {
                return false;
            }

            //the base is the only base in the hub
            if (this.Prev == null) {
                return true;
            }

            //distance between mid points is larger than parameterChange => we can't change the order
            if (this.SpanBetweenTwoPoints(this.Prev.ParMid, this.ParMid) > change && this.SpanBetweenTwoPoints(this.ParMid, this.Next.ParMid) > change) {
                return true;
            }

            Point rSoP = this.RotateLeftPoint(rotationOfLeftPoint, parameterChange);
            Point lSoP = this.RotateRigthPoint(rotationOfRightPoint, parameterChange);
            Point newMidPoint = (rSoP + lSoP) / 2.0;
            Point curMidPoint = this.MidPoint;

            //check Prev
            if (Point.GetTriangleOrientation(this.CurveCenter, this.Prev.MidPoint, curMidPoint) != Point.GetTriangleOrientation(this.CurveCenter, this.Prev.MidPoint, newMidPoint)) {
                return false;
            }

            //Next
            if (Point.GetTriangleOrientation(this.CurveCenter, this.Next.MidPoint, curMidPoint) != Point.GetTriangleOrientation(this.CurveCenter, this.Next.MidPoint, newMidPoint)) {
                return false;
            }

            return true;
        }
    }
}