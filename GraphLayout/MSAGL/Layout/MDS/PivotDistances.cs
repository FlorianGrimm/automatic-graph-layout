using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.MDS
{
    /// <summary>
    /// An algorithm for computing the distances between a selected set of nodes and all nodes.
    /// </summary>
    public class PivotDistances : AlgorithmBase
    {
        private GeometryGraph graph;

        private bool directed;

        private int[] pivotArray;

        /// <summary>
        /// A square matrix with shortest path distances.
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1819:PropertiesShouldNotReturnArrays", Justification = "This is performance critical.  Copying the array would be slow.")]
        public double[][] Result { get; private set; }

        /// <summary>
        /// Computes distances between a selected set of nodes and all nodes.
        /// Pivot nodes are selected with maxmin strategy (first at random, later
        /// ones to maximize distances to all previously selected ones).
        /// </summary>
        /// <param name="graph">A graph.</param>
        /// <param name="directed">Whether shortest paths are directed.</param>
        /// <param name="pivotArray">Number of pivots.</param>
        public PivotDistances(GeometryGraph graph, bool directed, int[] pivotArray)
        {
            this.graph = graph;
            this.directed = directed;
            this.pivotArray = pivotArray;
        }

        /// <summary>
        /// Executes the algorithm.
        /// </summary>
        protected override void  RunInternal()
        {
            this.Result = new double[this.pivotArray.Length][];

            Node[] nodes = new Node[this.graph.Nodes.Count];
            this.graph.Nodes.CopyTo(nodes, 0);
            double[] min = new double[this.graph.Nodes.Count];
            for (int i = 0; i < min.Length; i++)
            {
                min[i] = Double.PositiveInfinity;
            }
            Node pivot = nodes[0];
            this.pivotArray[0] = 0;
            for (int i = 0; ; i++) {
                var ssd = new SingleSourceDistances(this.graph, pivot);
                ssd.Run();
                this.Result[i] = ssd.Result;
                if (i + 1 < this.pivotArray.Length)
                {//looking for the next pivot
                    int argmax = 0;
                    for (int j = 0; j < this.Result[i].Length; j++)
                    {
                        min[j] = Math.Min(min[j], this.Result[i][j]);
                        if (min[j] > min[argmax]) {
                            argmax = j;
                        }
                    }
                    pivot = nodes[argmax];
                    this.pivotArray[i + 1] = argmax;
                }
                else {
                    break;
                }
            }

        }
    }
}
