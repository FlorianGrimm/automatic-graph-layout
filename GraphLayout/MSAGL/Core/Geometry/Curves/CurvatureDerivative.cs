using System;

namespace Microsoft.Msagl.Core.Geometry.Curves {
    /// <summary>
    /// 
    /// </summary>
    public class CurvatureDerivative : IFunction {
        private ICurve curve;
        private double start;
        private double end;
        /// <summary>
        /// 
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="start"></param>
        /// <param name="end"></param>
        public CurvatureDerivative(ICurve curve, double start, double end) {
            this.curve = curve;
            this.start = start;
            this.end = end;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double this[double t] {
            get { return this.curve.CurvatureDerivative(t); }
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double Derivative(double t) {
            return this.curve.CurvatureSecondDerivative(t);
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double SecondDerivative(double t) {
            throw new NotImplementedException();
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="t"></param>
        /// <returns></returns>
        public double ThirdDerivative(double t) {
            throw new NotImplementedException();
        }
        /// <summary>
        /// 
        /// </summary>
        public double ParStart {
            get {return this.start; }
        }
/// <summary>
/// 
/// </summary>
        public double ParEnd {
            get { return this.end; }
        }

    }
}
