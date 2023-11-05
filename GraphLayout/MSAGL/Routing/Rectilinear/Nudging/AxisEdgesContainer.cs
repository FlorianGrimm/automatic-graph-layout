using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// keeps a list of active overlapping AxisEdges discovered during the sweep
    ///  </summary>
    internal class AxisEdgesContainer {
        private readonly Set<AxisEdge> edges = new Set<AxisEdge>();
        internal IEnumerable<AxisEdge> Edges {get { return this.edges; } }
        /// <summary>
        /// it is not necessarely the upper point but some point above the source
        /// </summary>
        internal Point UpPoint;


        internal void AddEdge(AxisEdge edge){
            this.UpPoint = edge.TargetPoint;
            Debug.Assert(this.edges.Contains(edge)==false);
            this.edges.Insert(edge);
        }

        internal AxisEdgesContainer(Point source){
            this.Source = source;
        }

        public Point Source { get; set; }

        
        
        internal void RemoveAxis(AxisEdge edge){
            Debug.Assert(this.edges.Contains(edge));
            this.edges.Remove(edge);
        }

        internal bool IsEmpty(){
            return this.edges.Count == 0;
        }
    }
}