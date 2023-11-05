using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// a wrapper arownd VisibilityEdge representing the same edge 
    /// but oriented along the X or the Y axis
    /// </summary>
    internal class AxisEdge:VisibilityEdge {
        internal Direction Direction { get; set; }
        internal AxisEdge(VisibilityVertex source, VisibilityVertex target)
            : base(source, target){
            this.RightBound = double.PositiveInfinity;
            this.LeftBound = double.NegativeInfinity;
            this.Direction = CompassVector.DirectionsFromPointToPoint(source.Point, target.Point);
            Debug.Assert(this.Direction == Direction.East || this.Direction == Direction.North);
        }

        readonly internal Set<AxisEdge> RightNeighbors = new Set<AxisEdge>();

        internal void AddRightNeighbor(AxisEdge edge) {
            this.RightNeighbors.Insert(edge);
        }

        internal double LeftBound { get; set; }

        internal double RightBound { get; private set; }

        private readonly Set<LongestNudgedSegment> setOfLongestSegs = new Set<LongestNudgedSegment>();
        
        internal IEnumerable<LongestNudgedSegment> LongestNudgedSegments { get { return this.setOfLongestSegs; } }
        
        internal void AddLongestNudgedSegment(LongestNudgedSegment segment) {
            this.setOfLongestSegs.Insert(segment);
        }

        internal void BoundFromRight(double rightbound) {
            rightbound = Math.Max(rightbound, this.LeftBound);
            this.RightBound = Math.Min(rightbound, this.RightBound);
        }

        internal void BoundFromLeft(double leftbound) {
            leftbound = Math.Min(leftbound, this.RightBound);
            this.LeftBound = Math.Max(leftbound, this.LeftBound);
        }
    }
}