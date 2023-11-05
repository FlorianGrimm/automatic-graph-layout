using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// 
    /// </summary>
    public class CurvePort:Port  {
        private double parameter;
        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="curve"></param>
        /// <param name="parameter"></param>
        public CurvePort(ICurve curve, double parameter) {
            this.curve = curve;
            this.parameter = parameter;
        }

       
        /// <summary>
        /// empty constructor
        /// </summary>
        public CurvePort() { }
        /// <summary>
        /// 
        /// </summary>
        public double Parameter {
            get { return this.parameter; }
            set { this.parameter = value; }
        }

        private ICurve curve;
        /// <summary>
        /// 
        /// </summary>
        override public ICurve Curve {
            get { return this.curve; }
            set { this.curve = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        public override Point Location {
#if SHARPKIT
            get { return Curve[parameter].Clone(); }
#else
            get { return this.Curve[this.parameter]; }
#endif
        }
    }
}
