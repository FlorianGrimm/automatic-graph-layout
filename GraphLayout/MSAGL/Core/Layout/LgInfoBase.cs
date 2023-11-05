using System;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// the base class for LgNodeInfo and LgEdgeInfo
    /// </summary>
    public class LgInfoBase {
        private double _slidingZoomLevel = double.PositiveInfinity;
        private double _zoomLevel = double.PositiveInfinity;
        private double _rank;

        /// <summary>
        /// 
        /// </summary>
        public double SlidingZoomLevel {
            get { return this._slidingZoomLevel; }
            set { this._slidingZoomLevel = value; }
        }

        /// <summary>
        /// if the zoom is at least ZoomLevel the node should be rendered
        /// </summary>
        public double ZoomLevel {
            get { return Math.Min(this._zoomLevel, this.SlidingZoomLevel); }
            set { this._zoomLevel = value; }
        }

        /// <summary>
        /// the rank of the element
        /// </summary>
        public double Rank {
            get { return this._rank; }
            set { this._rank = value; }
        }

        internal bool ZoomLevelIsNotSet {
            get { return this.ZoomLevel == double.PositiveInfinity; }

        }
    }
}