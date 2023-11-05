using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing {
    
    internal class MultipleSourceMultipleTargetsShortestPathOnVisibilityGraph {
        //we are not using the A* algorithm since it does not make much sense for muliple targets
        //but we use the upper bound heuristic
        private readonly IEnumerable<VisibilityVertex> sources;
        private readonly Set<VisibilityVertex> targets;
        private VisibilityVertex _current;
        private VisibilityVertex closestTarget;
        private double upperBound = double.PositiveInfinity;
        private VisibilityGraph _visGraph;
        internal MultipleSourceMultipleTargetsShortestPathOnVisibilityGraph(IEnumerable<VisibilityVertex> sourceVisVertices,
                                                                         IEnumerable<VisibilityVertex> targetVisVertices, VisibilityGraph visibilityGraph)
        {
            this._visGraph = visibilityGraph;
            visibilityGraph.ClearPrevEdgesTable();
            foreach (var v in visibilityGraph.Vertices())
            {
                v.Distance = Double.PositiveInfinity;
            }
            this.sources = sourceVisVertices;
            this.targets = new Set<VisibilityVertex>(targetVisVertices);

        }


        /// <summary>
        /// Returns  a  path
        /// </summary>
        /// <returns>a path or null if the target is not reachable from the source</returns>
        internal IEnumerable<VisibilityVertex> GetPath(){ 
            var pq = new GenericBinaryHeapPriorityQueue<VisibilityVertex>();
            foreach (var v in this.sources) {
                v.Distance = 0;
                pq.Enqueue(v, 0);
            }
            while (!pq.IsEmpty()) {
                this._current = pq.Dequeue();
                if (this.targets.Contains(this._current)) {
                    break;
                }

                foreach (var e in this._current.OutEdges.Where(this.PassableOutEdge)) {
                    this.ProcessNeighbor(pq, e, e.Target);
                }

                foreach (var e in this._current.InEdges.Where(this.PassableInEdge)) {
                    this.ProcessNeighbor(pq, e, e.Source);
                }
            }

            return this._visGraph.PreviosVertex(this._current) == null ? null : this.CalculatePath();
        }

        private bool PassableOutEdge(VisibilityEdge e) {
            return
                this.targets.Contains(e.Target) || 
                !IsForbidden(e);
        }

        private bool PassableInEdge(VisibilityEdge e) {
            return this.targets.Contains(e.Source)  || !IsForbidden(e);
        }


        internal static bool IsForbidden(VisibilityEdge e) {
            return e.IsPassable != null && !e.IsPassable() || e is TollFreeVisibilityEdge;
        }

        private void ProcessNeighbor(GenericBinaryHeapPriorityQueue<VisibilityVertex> pq, VisibilityEdge l,
                             VisibilityVertex v)
        {
            var len = l.Length;
            var c = this._current.Distance + len;
            if (c >= this.upperBound) {
                return;
            }

            if (this.targets.Contains(v))
            {
                this.upperBound = c;
                this.closestTarget = v;
            }
            if (v != this.sources && this._visGraph.PreviosVertex(v) == null)
            {
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                pq.Enqueue(v, c);
            }
            else if (c < v.Distance)
            {
                //This condition should never hold for the dequeued nodes.
                //However because of a very rare case of an epsilon error it might!
                //In this case DecreasePriority will fail to find "v" and the algorithm will continue working.
                //Since v is not in the queue changing its .Distance will not mess up the queue.
                //Changing v.Prev is fine since we come up with a path with an insignificantly
                //smaller distance.
                v.Distance = c;
                this._visGraph.SetPreviousEdge(v, l);
                pq.DecreasePriority(v, c);
            }
        }

        private IEnumerable<VisibilityVertex> CalculatePath() {
            if (this.closestTarget == null) {
                return null;
            }

            var ret = new List<VisibilityVertex>();
            var v = this.closestTarget;
            do {
                ret.Add(v);
                v = this._visGraph.PreviosVertex(v);
            } while (v.Distance > 0);
            ret.Add(v);

            ret.Reverse();
            return ret; 
        }
    }
}