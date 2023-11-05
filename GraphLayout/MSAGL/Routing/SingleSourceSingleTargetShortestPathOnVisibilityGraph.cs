using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.GraphmapsWithMesh;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing
{
    internal class SingleSourceSingleTargetShortestPathOnVisibilityGraph
    {
        public Tiling _g;
        private readonly VisibilityVertex _source;
        private readonly VisibilityVertex _target;
        private VisibilityGraph _visGraph;
        private double _lengthMultiplier = 1;
        public double LengthMultiplier
        {
            get { return this._lengthMultiplier; }
            set { this._lengthMultiplier = value; }
        }

        private double _lengthMultiplierForAStar = 1;
        public double LengthMultiplierForAStar
        {
            get { return this._lengthMultiplierForAStar; }
            set { this._lengthMultiplierForAStar = value; }
        }

        internal SingleSourceSingleTargetShortestPathOnVisibilityGraph(VisibilityGraph visGraph, VisibilityVertex sourceVisVertex, VisibilityVertex targetVisVertex, Tiling g)
        {
            this._visGraph = visGraph;
            this._source = sourceVisVertex;
            this._target = targetVisVertex;
            this._source.Distance = 0;
            this._g = g;
        }
        internal SingleSourceSingleTargetShortestPathOnVisibilityGraph(VisibilityGraph visGraph, VisibilityVertex sourceVisVertex, VisibilityVertex targetVisVertex)
        {
            this._visGraph = visGraph;
            this._source = sourceVisVertex;
            this._target = targetVisVertex;
            this._source.Distance = 0;
        }

        /// <summary>
        /// Returns  a  path
        /// </summary>
        /// <returns>a path or null if the target is not reachable from the source</returns>
        internal IEnumerable<VisibilityVertex> GetPath(bool shrinkEdgeLength)
        {
            var pq = new GenericBinaryHeapPriorityQueue<VisibilityVertex>();

            this._source.Distance = 0;
            this._target.Distance = double.PositiveInfinity;
            pq.Enqueue(this._source, this.H(this._source));

            while (!pq.IsEmpty())
            {
                double hu;
                var u = pq.Dequeue(out hu);
                if (hu >= this._target.Distance) {
                    break;
                }

                foreach (var e in u.OutEdges)
                {

                    if (this.PassableOutEdge(e))
                    {
                        var v = e.Target;
                        if (u != this._source && u.isReal) {
                            this.ProcessNeighbor(pq, u, e, v, 1000);
                        } else {
                            this.ProcessNeighbor(pq, u, e, v);
                        }
                    }
                }

                foreach (var e in u.InEdges)
                {
                    if (this.PassableInEdge(e))
                    {
                        var v = e.Source;
                        this.ProcessNeighbor(pq, u, e, v);
                    }
                }

            }
            return this._visGraph.PreviosVertex(this._target) == null
                ? null
                : this.CalculatePath(shrinkEdgeLength);
        }


        internal void AssertEdgesPassable(List<VisibilityEdge> path)
        {
            foreach (var edge in path)
            {
                Debug.Assert(this.PassableOutEdge(edge) || this.PassableInEdge(edge));
            }
        }

        private bool PassableOutEdge(VisibilityEdge e)
        {
            return e.Source == this._source || e.Target == this._target || !IsForbidden(e);
        }

        private bool PassableInEdge(VisibilityEdge e)
        {
            return e.Source == this._target || e.Target == this._source || !IsForbidden(e);
        }

        internal static bool IsForbidden(VisibilityEdge e)
        {
            return e.IsPassable != null && !e.IsPassable() || e is TollFreeVisibilityEdge;
        }

        private void ProcessNeighbor(GenericBinaryHeapPriorityQueue<VisibilityVertex> pq, VisibilityVertex u, VisibilityEdge l, VisibilityVertex v, int penalty)
        {
            var len = l.Length + penalty;
            var c = u.Distance + len;

            if (v != this._source && this._visGraph.PreviosVertex(v) == null)
            {
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                if (v != this._target)
                {
                    pq.Enqueue(v, this.H(v));
                }
            }
            else if (v != this._source && c < v.Distance)
            { //This condition should never hold for the dequeued nodes.
                //However because of a very rare case of an epsilon error it might!
                //In this case DecreasePriority will fail to find "v" and the algorithm will continue working.
                //Since v is not in the queue changing its .Distance will not influence other nodes.
                //Changing v.Prev is fine since we come up with the path with an insignificantly
                //smaller distance.
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                if (v != this._target) {
                    pq.DecreasePriority(v, this.H(v));
                }
            }
        }

        private void ProcessNeighbor(GenericBinaryHeapPriorityQueue<VisibilityVertex> pq, VisibilityVertex u, VisibilityEdge l, VisibilityVertex v)
        {
            var len = l.Length;
            var c = u.Distance + len;

            if (v != this._source && this._visGraph.PreviosVertex(v) == null)
            {
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                if (v != this._target)
                {
                    pq.Enqueue(v, this.H(v));
                }
            }
            else if (v != this._source && c < v.Distance)
            { //This condition should never hold for the dequeued nodes.
                //However because of a very rare case of an epsilon error it might!
                //In this case DecreasePriority will fail to find "v" and the algorithm will continue working.
                //Since v is not in the queue changing its .Distance will not influence other nodes.
                //Changing v.Prev is fine since we come up with the path with an insignificantly
                //smaller distance.
                var prevV = this._visGraph.PreviosVertex(v);
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                if (v != this._target) {
                    pq.DecreasePriority(v, this.H(v));
                }
            }
        }

        private double H(VisibilityVertex visibilityVertex)
        {
            return visibilityVertex.Distance + (visibilityVertex.Point - this._target.Point).Length * this.LengthMultiplierForAStar;
        }

        private IEnumerable<VisibilityVertex> CalculatePath(bool shrinkEdgeLength)
        {
            var ret = new List<VisibilityVertex>();
            var v = this._target;
            do
            {
                ret.Add(v);
                if (shrinkEdgeLength) {
                    this._visGraph.ShrinkLengthOfPrevEdge(v, this.LengthMultiplier);
                }

                v = this._visGraph.PreviosVertex(v);
            } while (v != this._source);
            ret.Add(this._source);

            for (int i = ret.Count - 1; i >= 0; i--) {
                yield return ret[i];
            }
        }
    }
}

