using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    internal class SdVertex {
        internal VisibilityVertex VisibilityVertex;
        internal List<SdBoneEdge> InBoneEdges = new List<SdBoneEdge>();
        internal List<SdBoneEdge> OutBoneEdges = new List<SdBoneEdge>();

        internal SdVertex Prev {
            get {
                if (this.PrevEdge == null) {
                    return null;
                }

                return this.PrevEdge.Source == this ? this.PrevEdge.Target : this.PrevEdge.Source;
            }
        }

        internal SdBoneEdge PrevEdge { get; set; }

        internal SdVertex(VisibilityVertex visibilityVertex) {
            this.VisibilityVertex = visibilityVertex;
        }

        internal CdtTriangle Triangle;

        internal bool IsSourceOfRouting { get; set; }

        internal bool IsTargetOfRouting { get; set; }

        internal Point Point { get { return this.VisibilityVertex.Point; } }

        private double cost;

        internal double Cost {
            get {
                if (this.IsSourceOfRouting) {
                    return this.cost;
                }

                return this.Prev == null ? double.PositiveInfinity : this.cost;
            }
            set { this.cost = value; }
        }

        public void SetPreviousToNull() {
            this.PrevEdge = null;
        }
    }
}