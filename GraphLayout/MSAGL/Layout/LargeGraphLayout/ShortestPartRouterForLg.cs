using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    internal class ShortestPartRouterForLg {
        private readonly Node source;
        private readonly Node target;
        private GenericBinaryHeapPriorityQueue<Node> queue = new GenericBinaryHeapPriorityQueue<Node>();
        private Dictionary<Node, Tuple<Edge,double>> prev = new Dictionary<Node, Tuple<Edge,double>>();
        private Point pathDirection;
        private double alpha = 0.5;
        private double costToTarget;
        private Edge bestEdgeIntoTarget;
        private bool ignoreInteresingEdgesFunc;
        private Func<Node,LgNodeInfo> geomNodeToLgNode;

        internal Func<Node,Node, bool> EdgeIsInterestingFunc { get; set; }
        internal bool ConsiderOnlyInterestingEdges { get; set; }


        public ShortestPartRouterForLg(Node source, Node target, Func<Node, LgNodeInfo> geomNodeToLgNode) {
            this.source = source;
            this.target = target;
            this.geomNodeToLgNode = geomNodeToLgNode;
            this.queue.Enqueue(source, 0);
            this.pathDirection = target.Center - source.Center;
            this.costToTarget = double.PositiveInfinity;
            this.EdgeIsInterestingFunc = this.MonotonicityFunc;
        }

        private bool MonotonicityFunc(Node a, Node b) {
            var edgeVector = a.Center - b.Center;
            return edgeVector* this.pathDirection >= 0;
        }


        public Edge[] Run() {
            this.ignoreInteresingEdgesFunc = false;
            var ret= this.SearchForPath().ToArray();
            if (ret.Length>0) {
                return ret;
            }

            if (this.ConsiderOnlyInterestingEdges) {
                return null;
            }

            this.ignoreInteresingEdgesFunc = true;
            this.Cleanup();
            var p= this.SearchForPath().ToArray();
            return p;
        }

        private void Cleanup() {
            this.costToTarget = double.PositiveInfinity;
            this.bestEdgeIntoTarget = null;
            this.prev.Clear();
            this.queue = new GenericBinaryHeapPriorityQueue<Node>();
            this.queue.Enqueue(this.source, 0);
        }

        private IEnumerable<Edge> SearchForPath() {
            while (this.queue.Count > 0 ) {
                double costPlus;
                var node= this.queue.Dequeue(out costPlus);
                if (costPlus >= this.costToTarget) {
                    break;
                }

                double cost = node == this.source ? 0 : this.prev[node].Item2;
                this.ProcessVertex(node, cost);
            }
            IEnumerable<Edge> ret = this.RecoverPath();
            return ret;
        }

        private IEnumerable<Edge> RecoverPath() {
            if (this.bestEdgeIntoTarget != null) {
                Node currentNode = this.target;
                Edge currentEdge = this.bestEdgeIntoTarget;
                do {
                    yield return currentEdge;
                    currentNode = this.GetOtherVertex(currentEdge, currentNode);
                    if (currentNode == this.source) {
                        break;
                    }

                    currentEdge = this.prev[currentNode].Item1;
                } while (true);
            }
        }

        private Node GetOtherVertex(Edge currentEdge, Node currentNode) {
            return currentEdge.Source == currentNode ? currentEdge.Target : currentEdge.Source;
        }

        private void ProcessVertex(Node node, double cost) {
            foreach (Edge outEdge in node.OutEdges) {
                this.ProcessOutEdge(outEdge, cost);
            }

            foreach (Edge outEdge in node.InEdges) {
                this.ProcessInEdge(outEdge, cost);
            }
        }

        private void ProcessInEdge(Edge inEdge, double cost) {
            var edgeVector = inEdge.Source.Center - inEdge.Target.Center;
            if (!this.ignoreInteresingEdgesFunc && this.EdgeIsInterestingFunc !=null &&  !this.EdgeIsInterestingFunc(inEdge.Target,inEdge.Source)) {
                return;
            }

            double costPlus = this.CalculateCostPlus(inEdge, edgeVector);
            var totalCostToEdgeEnd = cost + costPlus;
            if (inEdge.Source == this.target) {
                if (totalCostToEdgeEnd < this.costToTarget) {
                    this.costToTarget = totalCostToEdgeEnd;
                    this.bestEdgeIntoTarget = inEdge;
                }
            } else {
                double h = this.AStarH(inEdge.Source);
                if (totalCostToEdgeEnd + h >= this.costToTarget) {
                    return;
                }

                Tuple<Edge, double> storedPrev;
                if (this.prev.TryGetValue(inEdge.Source, out storedPrev)) {
                    if (storedPrev.Item2 > totalCostToEdgeEnd) {
                        this.prev[inEdge.Source] = new Tuple<Edge, double>(inEdge, totalCostToEdgeEnd);
                        this.queue.Enqueue(inEdge.Source, totalCostToEdgeEnd + h);
                    }
                } else {
                    this.prev[inEdge.Source]=new Tuple<Edge, double>(inEdge, totalCostToEdgeEnd);
                    this.queue.Enqueue(inEdge.Source, totalCostToEdgeEnd + h);
                }
            }
        }

        private void ProcessOutEdge(Edge outEdge, double cost) {
            var edgeVector = outEdge.Target.Center - outEdge.Source.Center;
            if (!this.ignoreInteresingEdgesFunc && this.EdgeIsInterestingFunc != null && !this.EdgeIsInterestingFunc(outEdge.Source, outEdge.Target)) {
                return;
            }

            double costPlus = this.CalculateCostPlus(outEdge, edgeVector);
            var totalCostToEdgeEnd = cost + costPlus;
            if (outEdge.Target == this.target) {
                if (totalCostToEdgeEnd < this.costToTarget) {
                    this.costToTarget = totalCostToEdgeEnd;
                    this.bestEdgeIntoTarget = outEdge;
                }
            }
            else {
                double h = this.AStarH(outEdge.Target);
                if (totalCostToEdgeEnd + h >= this.costToTarget) {
                    return;
                }

                Tuple<Edge, double> storedPrev;
                if(this.prev.TryGetValue(outEdge.Target, out storedPrev)) {
                    if (storedPrev.Item2 > totalCostToEdgeEnd) {
                        this.prev[outEdge.Target] = new Tuple<Edge, double>(outEdge, totalCostToEdgeEnd);
                        this.queue.Enqueue(outEdge.Target, totalCostToEdgeEnd+h);
                    }
                } else {
                    this.prev[outEdge.Target] = new Tuple<Edge, double>(outEdge, totalCostToEdgeEnd);
                    this.queue.Enqueue(outEdge.Target, totalCostToEdgeEnd + h);
                }
            }
        }

        private double AStarH(Node node) {
            return (node.Center - this.target.Center).Length*(1 - this.alpha);
        }

        private double CalculateCostPlus(Edge edge, Point edgeVector) {
            return edgeVector.Length*(1 - this.alpha / this.ZoomLevel(edge));//diminishing the edge length for important edges
        }

        private double ZoomLevel(Edge edge) {
            var src = this.geomNodeToLgNode(edge.Source);
            var trg = this.geomNodeToLgNode(edge.Target);
            return Math.Max(src.ZoomLevel, trg.ZoomLevel);
        }
    }
}