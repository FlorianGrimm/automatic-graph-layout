﻿using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;

namespace Microsoft.Msagl.Routing.Visibility {
    internal class PortObstacleEvent : SweepEvent {
        private readonly Point site;

        public PortObstacleEvent(Point site) {
            this.site = site;
        }

        internal override Point Site {
            get { return this.site; }
        }
    }
}