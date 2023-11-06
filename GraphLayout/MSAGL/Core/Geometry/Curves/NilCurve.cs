// #region Using directives

using System;
//#endregion

namespace Microsoft.Msagl.Core.Geometry.Curves {
#if TEST_MSAGL
    [Serializable]
#endif
    public sealed class NilCurve : ICurve {
        private  static NilCurve? _Empty;
        public static NilCurve Empty => (_Empty??=new NilCurve());

        private NilCurve() {
            
        }

        public Point this[double t] => throw new NotImplementedException();

        public ParallelogramNodeOverICurve ParallelogramNodeOverICurve => throw new NotImplementedException();

        public Rectangle BoundingBox => throw new NotImplementedException();

        public double ParStart => throw new NotImplementedException();

        public double ParEnd => throw new NotImplementedException();

        public Point Start => throw new NotImplementedException();

        public Point End => throw new NotImplementedException();

        public double Length => throw new NotImplementedException();

        public ICurve Clone() {
            throw new NotImplementedException();
        }

        public double ClosestParameter(Point targetPoint) {
            throw new NotImplementedException();
        }

        public double ClosestParameterWithinBounds(Point targetPoint, double low, double high) {
            throw new NotImplementedException();
        }

        public double Curvature(double t) {
            throw new NotImplementedException();
        }

        public double CurvatureDerivative(double t) {
            throw new NotImplementedException();
        }

        public double CurvatureSecondDerivative(double t) {
            throw new NotImplementedException();
        }

        public Point Derivative(double t) {
            throw new NotImplementedException();
        }

        public double GetParameterAtLength(double length) {
            throw new NotImplementedException();
        }

        public Point LeftDerivative(double t) {
            throw new NotImplementedException();
        }

        public double LengthPartial(double start, double end) {
            throw new NotImplementedException();
        }

        public ICurve? OffsetCurve(double offset, Point dir) => default;

        public ICurve? Reverse() => default;

        public Point RightDerivative(double t) {
            throw new NotImplementedException();
        }

        public ICurve ScaleFromOrigin(double xScale, double yScale) {
            throw new NotImplementedException();
        }

        public Point SecondDerivative(double t) {
            throw new NotImplementedException();
        }

        public Point ThirdDerivative(double t) {
            throw new NotImplementedException();
        }

        public ICurve Transform(PlaneTransformation transformation) {
            throw new NotImplementedException();
        }

        public void Translate(Point delta) {
            throw new NotImplementedException();
        }

        public ICurve Trim(double start, double end) {
            throw new NotImplementedException();
        }

        public ICurve TrimWithWrap(double start, double end) {
            throw new NotImplementedException();
        }
    }
}