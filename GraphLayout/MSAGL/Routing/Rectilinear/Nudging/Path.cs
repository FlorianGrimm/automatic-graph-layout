using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;

using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// represents the path for an EdgeGeometry 
    /// </summary>
    internal class Path {
        /// <summary>
        /// the corresponding edge geometry
        /// </summary>
        internal EdgeGeometry EdgeGeometry { get; set; }
        /// <summary>
        /// the path points
        /// </summary>
        internal IEnumerable<Point>? PathPoints { get; set; }

        internal double Width { get { return this.EdgeGeometry.LineWidth;} }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="edgeGeometry"></param>
        internal Path(EdgeGeometry edgeGeometry) {
            this.EdgeGeometry = edgeGeometry;
        }


        internal Point End {
            get {
                return this.LastEdge.Target;
            }
        }

        internal Point Start {
            get {
                return this.FirstEdge.Source;
            }
        }

        
        internal IEnumerable<PathEdge> PathEdges {
            get {
                for (var e = this.FirstEdge; e != null; e = e.Next) {
                    yield return e;
                }
            }
        }

        internal PathEdge FirstEdge { get; set; }

        internal PathEdge LastEdge { get; set; }


        internal void AddEdge(PathEdge edge) {
            edge.Path = this;
            Debug.Assert(edge.Source == this.LastEdge.Target);
            this.LastEdge.Next = edge;
            edge.Prev = this.LastEdge;
            this.LastEdge = edge;
        }

        internal void SetFirstEdge(PathEdge edge) {
            this.LastEdge = this.FirstEdge = edge;
            edge.Path = this;
        }
    
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return String.Format(CultureInfo.InvariantCulture, "{0}->{1}", this.EdgeGeometry.SourcePort.Location, this.EdgeGeometry.TargetPort.Location);
        }
    }
}