using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Layout.LargeGraphLayout;

namespace Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree {
    internal class MstLineSweeper {
        private readonly List<OverlappedEdge> _proximityEdges;
        private readonly Size[] _nodeSizes;
        private readonly Point[] _nodePositions;
        private readonly bool _forLayers;
        private RTree<int, double> _intervalTree;
        private BinaryHeapPriorityQueue _q;
        private int _numberOfOverlaps = 0;

        public MstLineSweeper(List<OverlappedEdge> proximityEdges, Size[] nodeSizes, Point[] nodePositions, bool forLayers) {
            this._proximityEdges = proximityEdges;
            this._nodeSizes = nodeSizes;
            this._nodePositions = nodePositions;
            this._forLayers = forLayers;
            Debug.Assert(nodePositions.Length==nodeSizes.Length);
            this._q = new BinaryHeapPriorityQueue(nodeSizes.Length*2); 
        }

        public int Run() {
            this.InitQueue();
            this.FindOverlaps();
            return this._numberOfOverlaps;
        }

        private void FindOverlaps() {
            while (this._q.Count > 0) {
                int i = this._q.Dequeue();
                if (i < this._nodePositions.Length) {
                    this.FindOverlapsWithInterval(i);
                    this.AddIntervalToTree(i);
                }
                else {
                    i -= this._nodePositions.Length;
                    this.RemoveIntervalFromTree(i);
                }
            }
        }

        private void RemoveIntervalFromTree(int i) {
            this._intervalTree.Remove(this.GetInterval(i), i);
        }

        private void AddIntervalToTree(int i) {
            var interval = this.GetInterval(i);
            if (this._intervalTree == null) {
                this._intervalTree = new RTree<int, double>();
            }

            this._intervalTree.Add(interval, i);
            
        }

        private void FindOverlapsWithInterval(int i) {
            if (this._intervalTree == null) {
                return;
            }

            var interval = this.GetInterval(i);
            foreach (int j in this._intervalTree.GetAllIntersecting(interval)) {
                var tuple = GTreeOverlapRemoval.GetIdealEdge(i, j,
                    this._nodePositions[i], this._nodePositions[j], this._nodeSizes, this._forLayers);

                if (!(tuple.overlapFactor > 1)) {
                    return;
                }

                this._proximityEdges.Add(tuple);
                this._numberOfOverlaps++;
            }
        }

        private Interval GetInterval(int i) {
            var w = this._nodeSizes[i].Width/2;
            var nodeCenterX = this._nodePositions[i].X;
            return new Interval(nodeCenterX-w, nodeCenterX+w);
        }

        private void InitQueue() {
            for (int i = 0; i < this._nodeSizes.Length; i++) {
                var h = this._nodeSizes[i].Height/2;
                var nodeCenterY = this._nodePositions[i].Y;
                this._q.Enqueue(i, nodeCenterY - h); // enqueue the bottom event
                this._q.Enqueue(this._nodeSizes.Length + i, nodeCenterY + h); // enqueue the top event
            }
        }
    }
}