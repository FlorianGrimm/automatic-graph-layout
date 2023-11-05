using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Core.Layout{
    internal struct PortObstacle : IObstacle {
        internal Point Location;
        internal PortObstacle(Point c) {
            this.Location = c;
        }
        /// <summary>
        /// 
        /// </summary>
        public Rectangle Rectangle {
            get { return new Rectangle(this.Location); }
        }
    }
}