using System.Collections.Generic;
using Microsoft.Msagl.Core.GraphAlgorithms;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Layered {

    /// <summary>
    /// Preparing the graph for x-coordinate calculation by inserting dummy nodes into the layers
    /// </summary>
    internal class LayerInserter {
        private BasicGraph<Node, PolyIntEdge> intGraph;
        private Database database;

        /// <summary>
        /// Old layered graph: 
        /// </summary>
        private ProperLayeredGraph layeredGraph;

        /// <summary>
        /// new layered graph 
        /// </summary>
        private ProperLayeredGraph nLayeredGraph;
        private PolyIntEdge[] virtNodesToIntEdges;

        internal ProperLayeredGraph NLayeredGraph {
            get { return this.nLayeredGraph; }
        }

        /// <summary>
        /// old layer arrays
        /// </summary>
        private LayerArrays la;

        /// <summary>
        /// new layer arrays
        /// </summary>
        private LayerArrays nla;

        internal LayerArrays Nla {
            get { return this.nla; }
        }


        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="layeredGraph"></param>
        /// <param name="la"></param>
        /// <param name="database"></param>
        /// <param name="intGraphP"></param>
        private LayerInserter(
          ProperLayeredGraph layeredGraph, LayerArrays la, Database database, BasicGraph<Node, PolyIntEdge> intGraphP) {
            this.la = la;
            this.database = database;
            this.layeredGraph = layeredGraph;
            this.intGraph = intGraphP;
        }

        ///// <summary>
        ///// the entry point of the class
        ///// </summary>
        ///// <param name="layeredGraph"></param>
        ///// <param name="la"></param>
        ///// <param name="db"></param>
        static internal void InsertLayers(
          ref ProperLayeredGraph layeredGraph, ref LayerArrays la, Database db, BasicGraph<Node, PolyIntEdge> intGraphP) {
            LayerInserter li = new LayerInserter(layeredGraph, la, db, intGraphP);
            li.InsertLayers();

            layeredGraph = li.NLayeredGraph;
            la = li.Nla.DropEmptyLayers();

        }

        /// <summary>
        /// new Y-layering
        /// </summary>
        private int[] NLayering {
            get {
                return this.nla.Y;
            }
        }

        /// <summary>
        /// does the main work
        /// </summary>
        private void InsertLayers() {

            this.EditOldLayering();

            this.CreateFullLayeredGraph();

            this.InitNewLayering();

            this.MapVirtualNodesToEdges();

            this.FillUnsortedNewOddLayers();

            this.WidenOriginalLayers();

            this.SortNewOddLayers();

        }
        /// <summary>
        /// virtual nodes inside of an edge should be of the form i,i+1, ....
        /// </summary>
        private void EditOldLayering() {
            int curVNode = this.intGraph.NodeCount;

            foreach (List<PolyIntEdge> list in this.database.RegularMultiedges) {
                int span = 0;
                PolyIntEdge e = list[0];
                span = e.LayerSpan * 2;
                if (span > 0) {//ignoring flat edges            
                    foreach (LayerEdge le in e.LayerEdges) {
                        if (le.Target != e.Target) {
                            curVNode++;
                            this.UpdateOldLayer(curVNode++, le.Target);
                        }
                    }
                    curVNode += (span - 1) * (list.Count - 1) + 1;
                }
            }
        }

        private void UpdateOldLayer(int replacingNode, int prevNode) {
            int x = this.la.X[prevNode];
            int y = this.la.Y[prevNode];
            int[] layer = this.la.Layers[y];
            layer[x] = replacingNode;
            //   this.la.X[replacingNode] = x;
            //  this.la.Y[replacingNode] = y;
        }

        /// <summary>
        /// Original layers are represented by even layers in the new layering.
        /// Here we add new virtices in such layers and 
        /// set new x-offsets of original and dummy vertices in these layers.
        /// </summary>
        private void WidenOriginalLayers() {
            for (int i = 0; i < this.la.Layers.Length; i++) {
                int[] layer = this.nla.Layers[i * 2];
                int offset = 0;
                foreach (int v in this.la.Layers[i]) {
                    PolyIntEdge e = this.virtNodesToIntEdges[v];
                    if (e != null) {
                        int layerOffsetInTheEdge = this.NLayering[e.Source] - this.NLayering[v];
                        List<PolyIntEdge> list = this.database.Multiedges[new IntPair(e.Source, e.Target)];

                        foreach (PolyIntEdge ie in list) {
                            if (ie != e) {
                                int u = ie.LayerEdges[layerOffsetInTheEdge].Source;
                                layer[offset] = u;
                                this.nla.X[u] = offset++;
                            } else {
                                layer[offset] = v;
                                this.nla.X[v] = offset++;
                            }
                        }
                    } else {
                        layer[offset] = v;
                        this.nla.X[v] = offset++;
                    }
                }
            }
        }

        /// <summary>
        /// filling new layers not corresponding to the original layers
        /// </summary>
        private void FillUnsortedNewOddLayers() {
            int[] c = new int[this.nla.Layers.Length];
            for (int i = this.intGraph.NodeCount; i < this.nLayeredGraph.NodeCount; i++) {
                int layer = this.NLayering[i];
                if (layer % 2 == 1) {//new layers have odd numbers
                    this.nla.Layers[layer][c[layer]++] = i;
                }
            }
        }


        /// <summary>
        /// create the mapping from the vertices to edges to which they belong
        /// </summary>
        private void MapVirtualNodesToEdges() {
            this.virtNodesToIntEdges = new PolyIntEdge[this.NLayering.Length];
            foreach (PolyIntEdge e in this.database.AllIntEdges) {
                if (e.Source != e.Target && e.LayerEdges!=null) {
                    foreach (LayerEdge le in e.LayerEdges) {
                        if (le.Target != e.Target) {
                            this.virtNodesToIntEdges[le.Target] = e;
                        }
                    }
                }
            }
        }

        private int totalNodes;
        /// <summary>
        /// Creating buckets for multi edges and allocating the graph.
        /// </summary>
        private void CreateFullLayeredGraph() {
            this.totalNodes = this.intGraph.NodeCount;
            foreach (List<PolyIntEdge> list in this.database.RegularMultiedges) {
                int span = 0;
                bool first = true;
                foreach (PolyIntEdge e in list) {
                    if (first) {
                        first = false;
                        span = e.LayerSpan * 2;
                    }
                    if (span > 0) {
                        e.LayerEdges = new LayerEdge[span];
                        for (int i = 0; i < span; i++) {
                            int source = EdgePathsInserter.GetSource(ref this.totalNodes, e, i);
                            int target = EdgePathsInserter.GetTarget(ref this.totalNodes, e, i, span);
                            e.LayerEdges[i] = new LayerEdge(source, target, e.CrossingWeight);
                        }
                        LayerInserter.RegisterDontStepOnVertex(this.database, e);
                    }
                }
            }
            this.nLayeredGraph = new ProperLayeredGraph(this.intGraph);
        }


        /// <summary>
        /// Sort new odd layers by the sum of x-coordinatates of predecessors and the successors of 
        /// dummy nodes.
        /// </summary>
        private void SortNewOddLayers() {

            for (int i = 1; i < this.nla.Layers.Length; i += 2) {
                SortedDictionary<int, object> sd = new SortedDictionary<int, object>();
                int[] layer = this.nla.Layers[i];
                foreach (int v in layer) {

                    //find unique predecessor and successor
                    int predecessor = -1;
                    foreach (LayerEdge ie in this.nLayeredGraph.InEdges(v)) {
                        predecessor = ie.Source;
                    }

                    int successor = -1;
                    foreach (LayerEdge ie in this.nLayeredGraph.OutEdges(v)) {
                        successor = ie.Target;
                    }

                    int x = this.nla.X[predecessor] + this.nla.X[successor];

                    if (sd.ContainsKey(x)) {
                        object o = sd[x];
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=347
                        if (o.GetType() == typeof(int)) {
#else
                        if (o is int) {
#endif
                            List<int> l = new List<int>();
                            l.Add((int)o);
                            l.Add(v);
                            sd[x] = l;
                        } else {
                            List<int> l = o as List<int>;
                            l.Add(v);
                        }
                    } else {
                        sd[x] = v;
                    }
                }
                //fill the layer according to this order
                int c = 0;
                foreach (object v in sd.Values) {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=347
                    if (v.GetType() == typeof(int))
#else
                    if (v is int)
#endif
                        layer[c++] = (int)v;
                    else {
                        foreach (int k in v as List<int>) {
                            layer[c++] = k;
                        }
                    }
                }

                //update X now
                for (int m = 0; m < layer.Length; m++) {
                    this.nla.X[layer[m]] = m;
                }
            }
        }

        /// <summary>
        /// Allocating new layering and filling its y-layers
        /// </summary>
        private void InitNewLayering() {


            this.nla = new LayerArrays(new int[this.totalNodes]);

            for (int i = 0; i < this.layeredGraph.NodeCount; i++) {
                this.NLayering[i] = this.la.Y[i] * 2;
            }

            foreach (KeyValuePair<IntPair, List<PolyIntEdge>> kv in this.database.Multiedges) {
                IntPair ip = kv.Key;

                if (ip.First != ip.Second && this.la.Y[ip.First] != this.la.Y[ip.Second]) {//not a self edge and not a flat edge
                    int top = this.la.Y[ip.x] * 2;
                    foreach (PolyIntEdge e in kv.Value) {
                        int layer = top - 1;
                        foreach (LayerEdge le in e.LayerEdges) {
                            if (le.Target != e.Target) {
                                this.NLayering[le.Target] = layer--;
                            }
                        }
                    }
                }
            }

            int[][] newLayers = new int[2 * this.la.Layers.Length - 1][];

            //count new layer widths
            int[] counts = new int[newLayers.Length];

            foreach (int l in this.NLayering) {
                counts[l]++;
            }

            for (int i = 0; i < counts.Length; i++) {
                newLayers[i] = new int[counts[i]];
            }

            this.nla = new LayerArrays(this.NLayering);
            this.nla.Layers = newLayers;

        }
        ///// <summary>
        ///// mark the vertex as one representing a label
        ///// or a middle of a multi edge
        ///// </summary>
        ///// <param name="db"></param>
        ///// <param name="bucket"></param>
        ///// <param name="parent"></param>
        ///// <param name="i"></param>
        internal static void RegisterDontStepOnVertex(Database db, PolyIntEdge parent) {
            if (db.Multiedges[new IntPair(parent.Source, parent.Target)].Count > 1) {
                LayerEdge e = parent.LayerEdges[parent.LayerEdges.Count / 2];
                db.MultipleMiddles.Insert(e.Source);
            }
        }
    }
}
