using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// floating port: specifies that the edge is routed to the Location 
    /// </summary>
    public class FloatingPort : Port {
        private ICurve curve; //a curve associated with the port

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="location"></param>
        /// <param name="curve">the port curve, can be null</param>
        public FloatingPort(ICurve curve, Point location){
            this.curve = curve;
#if SHARPKIT
            this.location = location.Clone();
#else
            this.location = location;
#endif
        }

        private Point location;
        /// <summary>
        /// the location of the port
        /// </summary>
        override public Point Location {
            get { return this.location; }
        }
        /// <summary>
        /// translate the port location by delta
        /// </summary>
        /// <param name="delta"></param>
        public virtual void Translate(Point delta)
        {
            this.location += delta;
        }
/// <summary>
/// the port's curve
/// </summary>
        public override ICurve Curve {
            get { return this.curve; }
            set { this.curve = value; }
        }

        /// <summary>
        /// Return a string representation of the Port location
        /// </summary>
        /// <returns>a string representation of the Port location</returns>
        public override string ToString() {
            return this.Location.ToString();
        }
    }
}
