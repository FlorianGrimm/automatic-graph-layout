using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    /// Represent a maximal straight segment of a path
    /// </summary>
    internal class LongestNudgedSegment : SegmentBase {


        internal LongestNudgedSegment(int variable) {
            this.Id = variable;
        }

        
        /// <summary>
        /// has to be North or East
        /// </summary>
        internal Direction CompassDirection { get; set; }

        //the segment can go only North or East independently of the edge directions
        private readonly List<PathEdge> edges = new List<PathEdge>();
        private Point start;
        private Point end;
        internal override Point Start { get { return this.start; } }
        internal override Point End { get { return this.end; } }



        /// <summary>
        /// the list of edges holding the same offset and direction
        /// </summary>
        internal List<PathEdge> Edges {
            get { return this.edges; }
        }

        internal void AddEdge(PathEdge edge) {
            if (this.Edges.Count == 0) {
                var dir = (edge.Target - edge.Source).CompassDirection;
                switch (dir) {
                    case Core.Geometry.Direction.South:
                        dir = Core.Geometry.Direction.North;
                        break;
                    case Core.Geometry.Direction.West:
                        dir = Core.Geometry.Direction.East;
                        break;
                }
                this.CompassDirection = dir;
                this.start = edge.Source;
                this.end = edge.Source; //does not matter; it will be fixed immediately
            }

            switch (this.CompassDirection) {
                case Core.Geometry.Direction.North:
                    this.TryPointForStartAndEndNorth(edge.Source);
                    this.TryPointForStartAndEndNorth(edge.Target);
                    break;
                case Core.Geometry.Direction.East:
                    this.TryPointForStartAndEndEast(edge.Source);
                    this.TryPointForStartAndEndEast(edge.Target);
                    break;
            }
            this.Edges.Add(edge);
        }

        private void TryPointForStartAndEndNorth(Point p) {
            if (p.Y < this.start.Y) {
                this.start = p;
            } else if (p.Y > this.end.Y) {
                this.end = p;
            }
        }

        private void TryPointForStartAndEndEast(Point p) {
            if (p.X < this.start.X) {
                this.start = p;
            } else if (p.X > this.end.X) {
                this.end = p;
            }
        }

        private bool isFixed;

        /// <summary>
        /// the segments constraining "this" from the right
        /// </summary>

        internal bool IsFixed {
            get { return this.isFixed; }
            set { this.isFixed = value; }
        }

        internal int Id = -1;
        

        /// <summary>
        /// the maximal width of the edges 
        /// </summary>
        public double Width {
            get { return this.edges.Max(edge => edge.Width); }
        }


        internal double GetLeftBound() {
            if (!this.IsFixed) {
                return this.Edges.Max(edge => edge.AxisEdge.LeftBound);
            }

            return this.CompassDirection == Core.Geometry.Direction.North ? this.Edges[0].Source.X : -this.Edges[0].Source.Y;
        }

        internal double GetRightBound() {
            if (!this.IsFixed) {
                return this.Edges.Min(edge => edge.AxisEdge.RightBound);
            }

            return this.Position();
        }

        private double Position() {
            return this.CompassDirection == Core.Geometry.Direction.North ? this.Edges[0].Source.X : -this.Edges[0].Source.Y;
        }

        internal double IdealPosition {get;set;}
    }
}