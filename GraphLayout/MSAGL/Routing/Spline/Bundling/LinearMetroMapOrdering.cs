using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.DebugHelpers;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// Linear algorithm as described in our paper
    /// Edge Routing with Ordered Bunldles
    /// </summary>
    public class LinearMetroMapOrdering : IMetroMapOrderingAlgorithm {
        /// <summary>
        /// bundle lines
        /// </summary>
        private readonly List<Metroline> MetrolinesGlobal;
        private List<int[]> Metrolines;

        /// <summary>
        /// Station positions
        /// </summary>
        private Point[] positions;

        /// <summary>
        /// Initialize bundle graph and build the ordering
        /// </summary>
        internal LinearMetroMapOrdering(List<Metroline> MetrolinesGlobal, Dictionary<Point, Station> pointToIndex) {
            this.MetrolinesGlobal = MetrolinesGlobal;

            this.ConvertParameters(pointToIndex);

            this.BuildOrder();
        }

        /// <summary>
        /// Get the ordering of lines on station u with respect to the edge (u->v)
        /// </summary>
        IEnumerable<Metroline> IMetroMapOrderingAlgorithm.GetOrder(Station u, Station v) {
            MetroEdge me = MetroEdge.CreateFromTwoNodes(u.SerialNumber, v.SerialNumber);
            List<int> orderedMetrolineListForUv = this.order[me];
            if (u.SerialNumber < v.SerialNumber) {
                foreach (int MetrolineIndex in orderedMetrolineListForUv) {
                    yield return this.MetrolinesGlobal[MetrolineIndex];
                }
            }
            else {
                for (int i = orderedMetrolineListForUv.Count - 1; i >= 0; i--) {
                    yield return this.MetrolinesGlobal[orderedMetrolineListForUv[i]];
                }
            }
        }

        /// <summary>
        /// Get the index of line on the edge (u->v) and node u
        /// </summary>
        int IMetroMapOrderingAlgorithm.GetLineIndexInOrder(Station u, Station v, Metroline Metroline) {
            MetroEdge me = MetroEdge.CreateFromTwoNodes(u.SerialNumber, v.SerialNumber);
            Dictionary<Metroline, int> d = this.lineIndexInOrder[me];
            if (u.SerialNumber < v.SerialNumber) {
                return d[Metroline];
            }
            else {
                return d.Count - 1 - d[Metroline];
            }
        }

        private void ConvertParameters(Dictionary<Point, Station> pointToIndex) {
            this.Metrolines = new List<int[]>();
            this.positions = new Point[pointToIndex.Count];
            foreach (Metroline gline in this.MetrolinesGlobal) {
                List<int> line = new List<int>();
                foreach (Point p in gline.Polyline) {
                    line.Add(pointToIndex[p].SerialNumber);
                    this.positions[pointToIndex[p].SerialNumber] = p;
                }

                this.Metrolines.Add(line.ToArray());
            }
        }

        //order for node u of edge u->v
        private Dictionary<MetroEdge, List<int>> order;
        private Dictionary<MetroEdge, Dictionary<Metroline, int>> lineIndexInOrder;
        private HashSet<int> nonTerminals;
        private HashSet<MetroEdge> initialEdges;

        /// <summary>
        /// Edge in graph H
        /// label is used to distinguish multiple edges
        /// </summary>
        private class MetroEdge {
            private List<int> nodes;

            internal static MetroEdge CreateFromTwoNodes(int u, int v) {
                MetroEdge res = new MetroEdge();
                res.nodes = new List<int>();
                res.nodes.Add(Math.Min(u, v));
                res.nodes.Add(Math.Max(u, v));

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289
                res.UpdateHashKey();
#endif

                return res;
            }

            internal static MetroEdge CreateFromTwoEdges(int v, MetroEdge e1, MetroEdge e2) {
                int s = e1.Source() == v ? e1.Target() : e1.Source();
                int t = e2.Source() == v ? e2.Target() : e2.Source();

                if (s < t) {
                    return CreateFromTwoEdges(v, e1.nodes, e2.nodes);
                } else {
                    return CreateFromTwoEdges(v, e2.nodes, e1.nodes);
                }
            }

            internal static MetroEdge CreateFromTwoEdges(int v, List<int> e1, List<int> e2) {
                List<int> nodes = new List<int>(e1.Count + e2.Count - 1);
                if (e1[0] != v) {
                    for (int i = 0; i < e1.Count; i++) {
                        nodes.Add(e1[i]);
                    }
                }
                else {
                    for (int i = e1.Count - 1; i >= 0; i--) {
                        nodes.Add(e1[i]);
                    }
                }

                if (e2[0] == v) {
                    for (int i = 1; i < e2.Count; i++) {
                        nodes.Add(e2[i]);
                    }
                }
                else {
                    for (int i = e2.Count - 2; i >= 0; i--) {
                        nodes.Add(e2[i]);
                    }
                }

                MetroEdge res = new MetroEdge();
                res.nodes = nodes;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289
                res.UpdateHashKey();
#endif
                return res;
            }

            internal int Source() {
                return this.nodes[0];
            }

            internal int Target() {
                return this.nodes[this.nodes.Count - 1];
            }

            public override string ToString() {
                string s = "(";
                foreach (int i in this.nodes) {
                    s += i + " ";
                }

                s += ")";
                return s;
            }

            private int label;
            private bool labelCached = false;

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            private SharpKit.JavaScript.JsString _hashKey;
            private void UpdateHashKey()
            {
                _hashKey = GetHashCode().ToString();
            }
#endif

            public override int GetHashCode()
            {
                if (!this.labelCached) {
                    ulong hc = (ulong)this.nodes.Count;
                    for (int i = 0; i < this.nodes.Count; i++) {
                        hc = unchecked(hc * 314159 + (ulong)this.nodes[i]);
                    }

                    this.label = (int)hc;
                    this.labelCached = true;
                }

                return this.label;
            }
            /// <summary>
            /// overrides the equality
            /// </summary>
            /// <param name="obj"></param>
            /// <returns></returns>
            public override bool Equals(object obj) {
                if (!(obj is MetroEdge)) {
                    return false;
                }

                return (MetroEdge)obj == this;

            }

            public static bool operator ==(MetroEdge pair0, MetroEdge pair1) {
                if (pair0.GetHashCode() != pair1.GetHashCode()) {
                    return false;
                }

                return true;
                //TODO: are conflicts possible?
                //return pair0.nodes.SequenceEqual(pair1.nodes);
            }

            public static bool operator !=(MetroEdge pair0, MetroEdge pair1) {
                return !(pair0 == pair1);
            }

        }

        /// <summary>
        /// unordered list of paths on a specified edge
        /// </summary>
        private class PathList {
            internal MetroEdge edge;
            internal HashSet<PathOnEdge> paths;
            internal List<PathList> subLists;

            public override string ToString() {
                return this.edge.ToString() + " (" + this.paths.Count + ")";
            }
        }

        private class PathOnEdge {
            internal int index;
            internal LinkedListNode<MetroEdge> node;

            public override string ToString() {
                string s = "(index = " + this.index + ")";
                return s;
            }
        }

        private Dictionary<int, LinkedList<MetroEdge>> orderedAdjacent;
        private Dictionary<Tuple<int, MetroEdge>, LinkedListNode<MetroEdge>> adjacencyIndex;
        private Dictionary<MetroEdge, PathList> e2p;
        private Dictionary<int, LinkedList<MetroEdge>> paths;

        /// <summary>
        /// Do the main job
        /// </summary>
        private void BuildOrder() {
            //init local structures
            this.Initialize();

            //ordering itself
            foreach (int v in this.nonTerminals) {
                this.ProcessNonTerminal(v);
            }

            //get result
            this.RestoreResult();
        }

        private void Initialize() {
            //non terminals and adjacent
            this.nonTerminals = new HashSet<int>();
            this.initialEdges = new HashSet<MetroEdge>();
            //non-sorted adjacent edges. will be sorted later
            Dictionary<int, HashSet<MetroEdge>> adjacent = new Dictionary<int, HashSet<MetroEdge>>();
            for (int mi = 0; mi < this.Metrolines.Count; mi++) {
                int[] Metroline = this.Metrolines[mi];
                for (int i = 0; i + 1 < Metroline.Length; i++) {
                    MetroEdge me = MetroEdge.CreateFromTwoNodes(Metroline[i], Metroline[i + 1]);

                    if (!this.initialEdges.Contains(me)) {
                        this.initialEdges.Add(me);
                    }

                    if (i + 2 < Metroline.Length) {
                        this.nonTerminals.Add(Metroline[i + 1]);
                    }

                    CollectionUtilities.AddToMap(adjacent, Metroline[i], me);
                    CollectionUtilities.AddToMap(adjacent, Metroline[i + 1], me);
                }
            }

            //order neighbors around each vertex
            this.InitAdjacencyData(adjacent);

            //create e2p and paths...
            this.InitPathData();
        }

        private void InitPathData() {
            this.paths = new Dictionary<int, LinkedList<MetroEdge>>();
            this.e2p = new Dictionary<MetroEdge, PathList>();
            for (int mi = 0; mi < this.Metrolines.Count; mi++) {
                int[] Metroline = this.Metrolines[mi];
                this.paths.Add(mi, new LinkedList<MetroEdge>());

                for (int i = 0; i + 1 < Metroline.Length; i++) {
                    MetroEdge me = MetroEdge.CreateFromTwoNodes(Metroline[i], Metroline[i + 1]);

                    if (!this.e2p.ContainsKey(me)) {
                        PathList pl = new PathList();
                        pl.edge = me;
                        pl.paths = new HashSet<PathOnEdge>();
                        this.e2p.Add(me, pl);
                    }

                    PathOnEdge pathOnEdge = new PathOnEdge();
                    pathOnEdge.index = mi;
                    pathOnEdge.node = this.paths[mi].AddLast(me);
                    this.e2p[me].paths.Add(pathOnEdge);
                }
            }
        }

        private void InitAdjacencyData(Dictionary<int, HashSet<MetroEdge>> adjacent) {
            this.orderedAdjacent = new Dictionary<int, LinkedList<MetroEdge>>();
            this.adjacencyIndex = new Dictionary<Tuple<int, MetroEdge>, LinkedListNode<MetroEdge>>();
            foreach (int v in adjacent.Keys) {
                List<MetroEdge> adj = new List<MetroEdge>(adjacent[v]);
                this.orderedAdjacent.Add(v, this.SortAdjacentEdges(v, adj));
            }
        }

        private LinkedList<MetroEdge> SortAdjacentEdges(int v, List<MetroEdge> adjacent) {
            MetroEdge mn = adjacent.First();
            int mnv = this.OppositeNode(mn, v);
            adjacent.Sort(delegate(MetroEdge edge1, MetroEdge edge2) {
                int a = this.OppositeNode(edge1, v);
                int b = this.OppositeNode(edge2, v);

                //TODO: remove angles!
                double angA = Point.Angle(this.positions[a] - this.positions[v], this.positions[mnv] - this.positions[v]);
                double angB = Point.Angle(this.positions[b] - this.positions[v], this.positions[mnv] - this.positions[v]);

                return angA.CompareTo(angB);
            });

            LinkedList<MetroEdge> res = new LinkedList<MetroEdge>();
            foreach (MetroEdge edge in adjacent) {
                LinkedListNode<MetroEdge> node = res.AddLast(edge);
                this.adjacencyIndex.Add(new Tuple<int, MetroEdge>(v, edge), node);
            }
            return res;
        }

        /// <summary>
        /// update adjacencies of node 'a': put new edges instead of oldEdge
        /// </summary>
        private void UpdateAdjacencyData(int a, MetroEdge oldEdge, List<PathList> newSubList) {
            //find a (cached) position of oldEdge in order
            LinkedListNode<MetroEdge> node = this.adjacencyIndex[new Tuple<int, MetroEdge>(a, oldEdge)];
            Debug.Assert(node.Value == oldEdge);

            LinkedListNode<MetroEdge> inode = node;
            foreach (PathList pl in newSubList) {
                MetroEdge newEdge = pl.edge;

                if (oldEdge.Source() == a) {
                    node = node.List.AddAfter(node, newEdge);
                } else {
                    node = node.List.AddBefore(node, newEdge);
                }

                this.adjacencyIndex.Add(new Tuple<int, MetroEdge>(a, newEdge), node);
            }

            this.adjacencyIndex.Remove(new Tuple<int, MetroEdge>(a, oldEdge));
            inode.List.Remove(inode);
        }

        /// <summary>
        /// recursively build an order on the edge
        /// </summary>
        private List<int> RestoreResult(MetroEdge edge) {
            List<int> res = new List<int>();

            PathList pl = this.e2p[edge];
            if (pl.subLists == null) {
                foreach (PathOnEdge path in pl.paths) {
                    res.Add(path.index);
                }
            }
            else {
                foreach (PathList subList in pl.subLists) {
                    List<int> subResult = this.RestoreResult(subList.edge);
                    if (!(edge.Source() == subList.edge.Source() || edge.Target() == subList.edge.Target())) {
                        subResult.Reverse();
                    }

                    res.AddRange(subResult);
                }
            }
            return res;
        }

        private void RestoreResult() {
            this.order = new Dictionary<MetroEdge, List<int>>();
            this.lineIndexInOrder = new Dictionary<MetroEdge, Dictionary<Metroline, int>>();
            foreach (MetroEdge me in this.initialEdges) {
                this.order.Add(me, this.RestoreResult(me));
                Dictionary<Metroline, int> d = new Dictionary<Metroline, int>();
                int index = 0;
                foreach (int v in this.order[me]) {
                    d[this.MetrolinesGlobal[v]] = index++;
                }
                this.lineIndexInOrder.Add(me, d);
            }
        }

        /// <summary>
        /// Remove vertex v from the graph. Update graph and paths correspondingly
        /// </summary>
        private void ProcessNonTerminal(int v) {
            //oldEdge => sorted PathLists
            Dictionary<MetroEdge, List<PathList>> newSubLists = this.RadixSort(v);

            //update current data
            foreach (MetroEdge oldEdge in this.orderedAdjacent[v]) {
                Debug.Assert(this.e2p.ContainsKey(oldEdge));
                List<PathList> newSubList = newSubLists[oldEdge];

                //update e2p[oldEdge]
                this.e2p[oldEdge].paths = null;
                this.e2p[oldEdge].subLists = newSubList;

                //update ordered adjacency data
                this.UpdateAdjacencyData(this.OppositeNode(oldEdge, v), oldEdge, newSubList);

                //update paths and add new edges
                foreach (PathList pl in newSubList) {
                    MetroEdge newEdge = pl.edge;

                    //we could check the reverse edge before
                    if (this.e2p.ContainsKey(newEdge)) {
                        continue;
                    }

                    //add e2p for new edge
                    this.e2p.Add(newEdge, pl);

                    //update paths
                    foreach (PathOnEdge path in pl.paths) {
                        this.UpdatePath(path, v, newEdge);
                    }
                }
            }
        }

        /// <summary>
        /// Linear sorting of paths passing through vertex v
        /// </summary>
        private Dictionary<MetroEdge, List<PathList>> RadixSort(int v) {
            //build a map [old_edge => list_of_paths_on_it]; the relative order of paths is important
            Dictionary<MetroEdge, List<PathOnEdge>> r = new Dictionary<MetroEdge, List<PathOnEdge>>();
            //first index in circular order
            Dictionary<MetroEdge, int> firstIndex = new Dictionary<MetroEdge, int>();

            foreach (MetroEdge oldEdge in this.orderedAdjacent[v]) {
                PathList pathList = this.e2p[oldEdge];
                foreach (PathOnEdge path in pathList.paths) {
                    MetroEdge ej = this.FindNextEdgeOnPath(v, path);
                    CollectionUtilities.AddToMap(r, ej, path);
                }

                firstIndex.Add(oldEdge, (r.ContainsKey(oldEdge) ? r[oldEdge].Count : 0));
            }

            //oldEdge => SortedPathLists
            Dictionary<MetroEdge, List<PathList>> res = new Dictionary<MetroEdge, List<PathList>>();
            //build the desired order for each edge
            foreach (MetroEdge oldEdge in this.orderedAdjacent[v]) {
                //r[oldEdge] is the right order! (up to the circleness)
                List<PathOnEdge> paths = r[oldEdge];
                Debug.Assert(paths.Count > 0);

                List<PathList> subLists = new List<PathList>();
                HashSet<PathOnEdge> curPathSet = new HashSet<PathOnEdge>();

                for (int j = 0; j < paths.Count; j++) {

                    int i = (j + firstIndex[oldEdge]) % paths.Count;
                    MetroEdge nowEdge = paths[i].node.Value;
                    MetroEdge nextEdge = paths[(i + 1) % paths.Count].node.Value;

                    curPathSet.Add(paths[i]);

                    if (j == paths.Count - 1 || nowEdge != nextEdge) {
                        //process
                        MetroEdge newEdge = MetroEdge.CreateFromTwoEdges(v, oldEdge, nowEdge);
                        PathList pl = new PathList();
                        pl.edge = newEdge;
                        pl.paths = curPathSet;
                        subLists.Add(pl);

                        //clear
                        curPathSet = new HashSet<PathOnEdge>();
                    }
                }

                if (oldEdge.Source() == v) {
                    subLists.Reverse();
                }

                res.Add(oldEdge, subLists);
            }

            return res;
        }

        /// <summary>
        /// extract the next edge on a given path after node v
        /// </summary>
        private MetroEdge FindNextEdgeOnPath(int v, PathOnEdge pathOnEdge) {
            if (pathOnEdge.node.Next != null) {
                int o = this.OppositeNode(pathOnEdge.node.Next.Value, v);
                if (o != -1) {
                    return pathOnEdge.node.Next.Value;
                }
            }

            if (pathOnEdge.node.Previous != null) {
                int o = this.OppositeNode(pathOnEdge.node.Previous.Value, v);
                if (o != -1) {
                    return pathOnEdge.node.Previous.Value;
                }
            }

            throw new NotSupportedException();
        }

        /// <summary>
        /// return an opposite vertex of a given edge
        /// </summary>
        private int OppositeNode(MetroEdge edge, int v) {
            if (edge.Source() == v) {
                return edge.Target();
            }

            if (edge.Target() == v) {
                return edge.Source();
            }

            return -1;
        }

        /// <summary>
        /// replace edges (av) and (vb) with edge (ab) on a given path
        /// </summary>
        private void UpdatePath(PathOnEdge pathOnEdge, int v, MetroEdge newEdge) {
            LinkedListNode<MetroEdge> f = pathOnEdge.node;
            Debug.Assert(f.Value.Source() == v || f.Value.Target() == v);

            int a, b;

            a = this.OppositeNode(f.Value, v);

            if (f.Next != null && (b = this.OppositeNode(f.Next.Value, v)) != -1) {
                Debug.Assert((a == newEdge.Source() || a == newEdge.Target()));
                Debug.Assert((b == newEdge.Source() || b == newEdge.Target()));

                f.Value = newEdge;
                f.List.Remove(f.Next);
            }
            else if (f.Previous != null && (b = this.OppositeNode(f.Previous.Value, v)) != -1) {
                Debug.Assert((a == newEdge.Source() || a == newEdge.Target()));
                Debug.Assert((b == newEdge.Source() || b == newEdge.Target()));

                f.Value = newEdge;
                f.List.Remove(f.Previous);
            }
            else {
                throw new NotSupportedException();
            }
        }
    }


}
