using System;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Visibility;
using System.Diagnostics;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    [DebuggerDisplay("({SourcePoint.X},{SourcePoint.Y})->({TargetPoint.X},{TargetPoint.Y})")]
    internal class SdBoneEdge {
        internal readonly VisibilityEdge VisibilityEdge;
        internal readonly SdVertex Source;
        internal readonly SdVertex Target;
        private int numberOfPassedPaths;

        internal SdBoneEdge(VisibilityEdge visibilityEdge, SdVertex source, SdVertex target) {
            this.VisibilityEdge = visibilityEdge;
            this.Source = source;
            this.Target = target;
        }

        internal Point TargetPoint {
            get { return this.Target.Point; }
        }

        internal Point SourcePoint {
            get { return this.Source.Point; }
        }

        internal bool IsOccupied {
            get { return this.numberOfPassedPaths > 0; }
        }

        internal Set<CdtEdge> CrossedCdtEdges { get; set; }

        internal bool IsPassable {
            get {
                return this.Target.IsTargetOfRouting || this.Source.IsSourceOfRouting ||
                       this.VisibilityEdge.IsPassable == null ||
                       this.VisibilityEdge.IsPassable();
            }
        }

        internal void AddOccupiedEdge() {
            this.numberOfPassedPaths++;
        }

        internal void RemoveOccupiedEdge() {
            this.numberOfPassedPaths--;
            Debug.Assert(this.numberOfPassedPaths >= 0);
        }
    }
}