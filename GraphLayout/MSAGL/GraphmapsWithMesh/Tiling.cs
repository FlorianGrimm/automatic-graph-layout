using System;
using System.Collections;
using System.Collections.Generic;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.GraphmapsWithMesh
{

    public class Tiling
    {
        public int NumOfnodes;
        public int NumOfnodesBeforeDetour;
        public int maxTheoreticalZoomLevel;

        public Dictionary<Microsoft.Msagl.Core.Layout.Edge, List<int>> pathList = new Dictionary<Microsoft.Msagl.Core.Layout.Edge, List<int>>();
        public Dictionary<int, List<Microsoft.Msagl.Core.Layout.Edge>> JunctionToEdgeList = new Dictionary<int, List<Microsoft.Msagl.Core.Layout.Edge>>();
        public Dictionary<Node, Point> nodeToLoc = new Dictionary<Node, Point>();


        public int[,] NodeMap;
        public int[] DegList;
        public Vertex[] VList;
        public Edge[,] EList;
        public double Maxweight;
        public int maxDeg;
        public int N;

        // TODO: Remove field as it is never used
        private Component _sNet = null;
        private readonly double[] _edgeNodeSeparation;
        private readonly double _angularResolution;
        public bool isPlanar;
        public double thinness;

        public RTree<int, Point> nodeTree = new RTree<int, Point>();
        public RTree<int, Point> edgeTree = new RTree<int, Point>();
        

        public Tiling(int nodeCount, bool isGraph)
        {
            this.thinness = 2;
            this._angularResolution = 0.3;
            this.NumOfnodes = this.N = nodeCount;
            this.maxDeg = 15;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            this.EList = new Edge[10 * this.N, this.maxDeg];
#endif
            this.VList = new Vertex[10 * this.N];
            this.DegList = new int[10 * this.N];
            this._edgeNodeSeparation = new double[20];

            this._edgeNodeSeparation[0] = 0.5;
            this._edgeNodeSeparation[1] = 1;
            this._edgeNodeSeparation[2] = 1;
            this._edgeNodeSeparation[3] = 1;
            this._edgeNodeSeparation[4] = 1;
            this._edgeNodeSeparation[5] = 1;
            this._edgeNodeSeparation[6] = 1;
            this._edgeNodeSeparation[7] = 1;
        }


        public Tiling(int bound)
        {


            int m, n, k = 1; //node location and inddex
            this.N = bound;
            this._angularResolution = 0.3;

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            this.NodeMap = new int[this.N, this.N];
            this.EList = new Edge[this.N * this.N, 20];
            this.VList = new Vertex[this.N * this.N];
            this.DegList = new int[this.N * this.N];
            this._edgeNodeSeparation = new double[20];

            this._edgeNodeSeparation[1] = 1;
            this._edgeNodeSeparation[2] = 1;
            this._edgeNodeSeparation[3] = 1;
            this._edgeNodeSeparation[4] = 1;
            this._edgeNodeSeparation[5] = 1;
            this._edgeNodeSeparation[6] = 1;
            this._edgeNodeSeparation[7] = 1;


            //create vertex list
            for (int y = 1; y < this.N; y++)
            {
                for (int x = 1; x < this.N - 1; x += 2)
                {
                    //create node at location m,n
                    m = x + (y + 1) % 2;
                    n = y;

                    this.VList[k] = new Vertex(m, n) { Id = k };

                    this.DegList[k] = 0;

                    //map location to the vertex index
                    this.NodeMap[m, n] = k;
                    k++;
                }

            }
            this.NumOfnodes = k - 1;

            //create edge list
            for (int index = 1; index < k; index++)
            {
                //find the current location
                m = this.VList[index].XLoc;
                n = this.VList[index].YLoc;

                //find the six neighbors


                //left                
                if (m - 2 > 0 && this.NodeMap[m - 2, n] > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m - 2, n]);
                }

                //right                
                if (m + 2 < this.N && this.NodeMap[m + 2, n] > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m + 2, n]);
                }

                //top-right                
                if (n + 1 < this.N && m + 1 < this.N && this.NodeMap[m + 1, n + 1] > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m + 1, n + 1]);
                }
                //top-left                
                if (n + 1 < this.N && m - 1 > 0 && this.NodeMap[m - 1, n + 1] > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m - 1, n + 1]);
                }
                //bottom-right                
                if (n - 1 > 0 && m + 1 < this.N && this.NodeMap[m + 1, n - 1] > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m + 1, n - 1]);
                }
                //bottom-left                
                if (n - 1 > 0 && m - 1 > 0)
                {
                    this.DegList[index]++;
                    this.EList[index, this.DegList[index]] = new Edge(this.NodeMap[m - 1, n - 1]);
                }

            }
#endif
        }

        public int InsertVertex(int x, int y)
        {
            int index = this.NumOfnodes;
            this.VList[index] = new Vertex(x, y) { Id = index };
            this.NumOfnodes++;
            return index;
        }


        public int InsertVertexWithDuplicateCheck(int x, int y, Dictionary<Point, int> locationtoNode)
        {
            Point p = new Point(x, y);
            if (locationtoNode.ContainsKey(p)) {
                return locationtoNode[p];
            }

            int index = this.NumOfnodes;
            this.VList[index] = new Vertex(x, y) { Id = index };
            locationtoNode.Add(p, index);
            this.NumOfnodes++;
            return index;
        }

        public void ComputeGridEdgeWeights()
        {
            Queue q = new Queue();
            //for each node, it it has a weight, then update edge weights 
            for (int index = 1; index <= this.NumOfnodes; index++)
            {
                if (this.VList[index].Weight == 0) {
                    continue;
                }

                q.Enqueue(index);

                while (q.Count > 0)
                {
                    //take the current node
                    var currentNode = (int)q.Dequeue();
                    if (this.VList[currentNode].Visited) {
                        continue;
                    } else {
                        this.VList[currentNode].Visited = true;
                    }
                    //for each neighbor of the current node
                    for (int neighb = 1; neighb <= this.DegList[currentNode]; neighb++)
                    {
                        var neighbor = this.EList[currentNode, neighb].NodeId;
                        //find an edge such that the target node is never visited; that is the edge has never been visited
                        if (this.VList[neighbor].Visited == false)
                        {
                            //BFS                            
                            q.Enqueue(neighbor);
                            //compute what would be the edge for the current edge
                            var temp = this.GetWeight(index, currentNode, neighbor, (int)this.VList[index].Weight);

                            if (temp < 0 || q.Count > 200)
                            {
                                q.Clear(); break;
                            }

                            //update the weight of the edge
                            this.EList[currentNode, neighb].Weight += temp;
                            //EList[currentNode, neighb].EDist = GetEucledianDist(currentNode, neighbor);

                            if (this.Maxweight < this.EList[currentNode, neighb].Weight) {
                                this.Maxweight = this.EList[currentNode, neighb].Weight;
                            }

                            //find the reverse edge and update it
                            for (int r = 1; r <= this.DegList[neighbor]; r++)
                            {
                                if (this.EList[neighbor, r].NodeId == currentNode)
                                {
                                    this.EList[neighbor, r].Weight += temp;
                                    //EList[neighbor, r].EDist = GetEucledianDist(currentNode, neighbor);
                                }
                            }//endfor
                        }//endif                          
                    } //endfor
                }
                q.Clear();
                for (int j = 1; j <= this.NumOfnodes; j++) {
                    this.VList[j].Visited = false;
                }
            }
        }
        public double GetWeight(int a, int b, int c, int w)
        {
            double d1 = Math.Sqrt((this.VList[a].XLoc - this.VList[b].XLoc) * (this.VList[a].XLoc - this.VList[b].XLoc) + (this.VList[a].YLoc - this.VList[b].YLoc) * (this.VList[a].YLoc - this.VList[b].YLoc));
            double d2 = Math.Sqrt((this.VList[a].XLoc - this.VList[c].XLoc) * (this.VList[a].XLoc - this.VList[c].XLoc) + (this.VList[a].YLoc - this.VList[c].YLoc) * (this.VList[a].YLoc - this.VList[c].YLoc));

            if (this.VList[a].Id == this.VList[b].Id) {
                return 1000;
            }


            //distribute around a disk of radious 
            double sigma = 15;// Math.Sqrt(w);
            w = 50;
            //return w - d;
            var w1 = w * (Math.Exp(-(d1 * d1 / (2 * sigma * sigma))) / (sigma * Math.Sqrt(2 * Math.PI)));
            var w2 = w * (Math.Exp(-(d2 * d2 / (2 * sigma * sigma))) / (sigma * Math.Sqrt(2 * Math.PI)));
            return (w1 + w2) / 2 + .0001;
        }
        public double GetEucledianDist(int a, int b)
        {
            return Math.Sqrt((this.VList[a].XLoc - this.VList[b].XLoc) * (this.VList[a].XLoc - this.VList[b].XLoc) + (this.VList[a].YLoc - this.VList[b].YLoc) * (this.VList[a].YLoc - this.VList[b].YLoc));
        }

        private Dictionary<int, int[]> allcandidates;

        public void buildCrossingCandidates()
        {
            this.allcandidates = new Dictionary<int, int[]>();
            for (int index = this.N; index < this.NumOfnodesBeforeDetour; index++)
            {
                Vertex w = this.VList[index];
                var p1 = new Microsoft.Msagl.Core.Geometry.Point(w.XLoc - 15, w.YLoc - 15);
                var p2 = new Microsoft.Msagl.Core.Geometry.Point(w.XLoc + 15, w.YLoc + 15);
                Microsoft.Msagl.Core.Geometry.Rectangle queryRectangle = new Microsoft.Msagl.Core.Geometry.Rectangle(
                    p1, p2);
                int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);
                this.allcandidates.Add(index, candidateList);
            }
        }

        public void MsaglMoveToMaximizeMinimumAngle()
        {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            int[,] listNeighbors = new int[20, 3];
            double[] d = new double[10];
            int a = 0, b = 0, mincostA = 0, mincostB = 0;
            bool localRefinementsFound = true;
            int iteration = 10;
            int offset = iteration * 2;

            while (localRefinementsFound && iteration > 0)
            {
                iteration--;
                localRefinementsFound = false;


                for (int index = this.N; index < this.NumOfnodes; index++)
                {
                    Vertex w = this.VList[index];

                    int numNeighbors = 0;
                    double profit = 0;

                    for (int k = 0; k < this.DegList[w.Id]; k++)
                    {
                        numNeighbors++;
                        listNeighbors[numNeighbors, 1] = this.EList[w.Id, k].NodeId;
                        listNeighbors[numNeighbors, 2] = k;
                    }

                    if (numNeighbors <= 1) {
                        continue;
                    }

                    for (int counter = 1; counter <= 9; counter++)
                    {
                        d[counter] = 0;

                        if (counter == 1) { a = 1; b = 1; }
                        if (counter == 2) { a = 0; b = 1; }
                        if (counter == 3) { a = -1; b = 1; }
                        if (counter == 4) { a = -1; b = 0; }
                        if (counter == 5) { a = -1; b = -1; }
                        if (counter == 6) { a = 0; b = -1; }
                        if (counter == 7) { a = 1; b = -1; }
                        if (counter == 8) { a = 1; b = 0; }
                        if (counter == 9) { a = 0; b = 0; }


                        for (int k = 1; k <= numNeighbors; k++)
                        {
                            double length = Math.Sqrt((w.XLoc + a - this.VList[listNeighbors[k, 1]].XLoc) *
                                          (w.XLoc + a - this.VList[listNeighbors[k, 1]].XLoc)
                                          +
                                          (w.YLoc + b - this.VList[listNeighbors[k, 1]].YLoc) *
                                          (w.YLoc + b - this.VList[listNeighbors[k, 1]].YLoc)
                                    );
                            if (length < 1)
                            {
                                mincostA = 0; mincostB = 0;
                                break;
                            }

                            // *try to maximize min angle
                            d[counter] = 3.1416;
                            for (int l = 1; l <= numNeighbors; l++)
                            {
                                if (l == k) {
                                    continue;
                                }

                                d[counter] = Math.Min(d[counter],
                                    Angle.getAngleIfSmallerThanPIby2(new Vertex(w.XLoc + a, w.YLoc + b),
                                        this.VList[listNeighbors[k, 1]], this.VList[listNeighbors[l, 1]]));
                            }

                        }
                        // *try to maximize min angle
                        if (profit < d[counter])
                        {
                            profit = d[counter]; mincostA = a; mincostB = b;
                        }


                    }
                    if (!(mincostA == 0 && mincostB == 0))
                    {
                        w.XLoc += mincostA;
                        w.YLoc += mincostB;
                        if (this.GetNode(w.XLoc, w.YLoc) == -1 || this.MsaglGoodResolution(w, listNeighbors, numNeighbors, offset) == false || this.noCrossings(w) == false)
                        {
                            w.XLoc -= mincostA;
                            w.YLoc -= mincostB;
                        }
                        else
                        {
                            localRefinementsFound = true;
                        }
                    }

                }
            }
#endif
        }


        public bool noCrossingsHeuristics(Vertex w, int index)
        {
            int[] candidateList;
            this.allcandidates.TryGetValue(index, out candidateList);
            if (candidateList.Length == 0) {
                return true;
            }

            for (int q = 0; q < candidateList.Length; q++)
            {
                int i = candidateList[q];

                for (int j = 0; j < this.DegList[i]; j++)
                {
                    int k1 = this.EList[i, j].NodeId;
                    Microsoft.Msagl.Core.Geometry.Point a = new Microsoft.Msagl.Core.Geometry.Point(this.VList[i].XLoc, this.VList[i].YLoc);
                    Microsoft.Msagl.Core.Geometry.Point b = new Microsoft.Msagl.Core.Geometry.Point(this.VList[k1].XLoc, this.VList[k1].YLoc);
                    for (int l = 0; l < this.DegList[w.Id]; l++)
                    {
                        int k2 = this.EList[w.Id, l].NodeId;
                        Microsoft.Msagl.Core.Geometry.Point c = new Microsoft.Msagl.Core.Geometry.Point(w.XLoc, w.YLoc);
                        Microsoft.Msagl.Core.Geometry.Point d = new Microsoft.Msagl.Core.Geometry.Point(this.VList[k2].XLoc, this.VList[k2].YLoc);

                        if (w.Id == i || k2 == i || w.Id == k1 || k2 == k1) {
                            continue;
                        }

                        Microsoft.Msagl.Core.Geometry.Point intersectionPoint;
                        if (Microsoft.Msagl.Core.Geometry.Point.SegmentSegmentIntersection(a, b, c, d, out intersectionPoint)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        public bool noCrossings(Vertex w)
        {
            for (int i = 0; i < this.NumOfnodes; i++)
            {
                for (int j = 0; j < this.DegList[i]; j++)
                {
                    int k1 = this.EList[i, j].NodeId;
                    Microsoft.Msagl.Core.Geometry.Point a = new Microsoft.Msagl.Core.Geometry.Point(this.VList[i].XLoc, this.VList[i].YLoc);
                    Microsoft.Msagl.Core.Geometry.Point b = new Microsoft.Msagl.Core.Geometry.Point(this.VList[k1].XLoc, this.VList[k1].YLoc);
                    for (int l = 0; l < this.DegList[w.Id]; l++)
                    {
                        int k2 = this.EList[w.Id, l].NodeId;
                        Microsoft.Msagl.Core.Geometry.Point c = new Microsoft.Msagl.Core.Geometry.Point(w.XLoc, w.YLoc);
                        Microsoft.Msagl.Core.Geometry.Point d = new Microsoft.Msagl.Core.Geometry.Point(this.VList[k2].XLoc, this.VList[k2].YLoc);

                        if (w.Id == i || k2 == i || w.Id == k1 || k2 == k1) {
                            continue;
                        }

                        Microsoft.Msagl.Core.Geometry.Point intersectionPoint;
                        if (Microsoft.Msagl.Core.Geometry.Point.SegmentSegmentIntersection(a, b, c, d, out intersectionPoint)) {
                            return false;
                        }
                    }
                }
            }
            return true;
        }

        public bool noCrossings(Vertex w, Vertex w1, Vertex w2)
        {
            Microsoft.Msagl.Core.Geometry.Point c = new Microsoft.Msagl.Core.Geometry.Point(w1.XLoc, w1.YLoc);
            Microsoft.Msagl.Core.Geometry.Point d = new Microsoft.Msagl.Core.Geometry.Point(w2.XLoc, w2.YLoc);


            int minx, miny, maxx, maxy;
            int offset = 5;

            minx = Math.Min(w1.XLoc, w2.XLoc);
            maxx = Math.Max(w1.XLoc, w2.XLoc);
            miny = Math.Min(w1.YLoc, w2.YLoc);
            maxy = Math.Max(w1.YLoc, w2.YLoc);
                
            var p1 = new Point(minx-offset, miny-offset);
            var p2 = new Point(maxx+offset, maxy+offset);
            Rectangle queryRectangle = new Rectangle(p1, p2);
            int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);

            
            for (int q = 0; q < candidateList.Length; q++)
            {
                int i = candidateList[q];
            //for (int i = 0; i < NumOfnodes; i++)
            //{
                if (w.Id == i || w1.Id == i || w2.Id == i) {
                    continue;
                }

                for (int j = 0; j < this.DegList[i]; j++)
                {
                    int k1 = this.EList[i, j].NodeId;
                    if (w1.Id == k1 || w2.Id == k1) {
                        continue;
                    }

                    Microsoft.Msagl.Core.Geometry.Point a = new Microsoft.Msagl.Core.Geometry.Point(this.VList[i].XLoc, this.VList[i].YLoc);
                    Microsoft.Msagl.Core.Geometry.Point b = new Microsoft.Msagl.Core.Geometry.Point(this.VList[k1].XLoc, this.VList[k1].YLoc);


                    Microsoft.Msagl.Core.Geometry.Point interestionPoint;

                    if (Microsoft.Msagl.Core.Geometry.Point.SegmentSegmentIntersection(a, b, c, d, out interestionPoint)) {
                        return false;
                    }
                }
            }
            return true;
        }

        public bool noCrossings(int[] r, int p, int q)
        {
            Vertex w1 = this.VList[p];
            Vertex w2 = this.VList[q];
            Point c = new Point(w1.XLoc, w1.YLoc);
            Point d = new Point(w2.XLoc, w2.YLoc);

                        
            Rectangle queryRectangle = new Rectangle(c, d);
            int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);

            for (int k  = 0; k < candidateList.Length; k++)
            {
                int i = candidateList[k];
            
            //for (int i = 0; i < NumOfnodes; i++)
            //{
                int index = 0;
                for (; index < r.Length; index++) {
                    if (r[index] == i) {
                        break;
                    }
                }

                if (index < r.Length || w1.Id == i || w2.Id == i) {
                    continue;
                }

                for (int j = 0; j < this.DegList[i]; j++)
                {

                    int k1 = this.EList[i, j].NodeId;

                    index = 0;
                    for (; index < r.Length; index++) {
                        if (r[index] == k1) {
                            break;
                        }
                    }

                    if (index < r.Length || w1.Id == k1 || w2.Id == k1) {
                        continue;
                    }

                    Point a = new Point(this.VList[i].XLoc, this.VList[i].YLoc);
                    Point b = new Point(this.VList[k1].XLoc, this.VList[k1].YLoc);


                    Point interestionPoint;

                    if (Point.SegmentSegmentIntersection(a, b, c, d, out interestionPoint)) {
                        return false;
                    }
                }
            }
            return true;
        }
 

        public bool Crossings(int p, int q)
        {
            Vertex w1 = this.VList[p];
            Vertex w2 = this.VList[q];
            Point c = new Point(w1.XLoc, w1.YLoc);
            Point d = new Point(w2.XLoc, w2.YLoc);


            Rectangle queryRectangle = new Rectangle(c, d);
            int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);

            for (int k = 0; k < candidateList.Length; k++)
            {
                int i = candidateList[k];
                 
                if( i == p || i == q) {
                    continue;
                }

                for (int j = 0; j < this.DegList[i]; j++)
                {
                    int k1 = this.EList[i, j].NodeId;
                    
                    if (p == k1 || q == k1) {
                        continue;
                    }

                    Point a = new Point(this.VList[i].XLoc, this.VList[i].YLoc);
                    Point b = new Point(this.VList[k1].XLoc, this.VList[k1].YLoc);

                    Point interestionPoint;
                    if (Point.SegmentSegmentIntersection(a, b, c, d, out interestionPoint)) {
                        return true;
                    }
                }
            }
            return false;
        }
        public bool MsaglGoodResolution(Vertex w, int[,] listNeighbors, int numNeighbors, int offset)
        {
            for (int i = 1; i < numNeighbors; i++)
            {
                for (int j = i + 1; j <= numNeighbors; j++)
                {
                    //check for angular resolution   
                    if (Angle.getAngleIfSmallerThanPIby2(w, this.VList[listNeighbors[i, 1]], this.VList[listNeighbors[j, 1]]) < this._angularResolution)
                    {
                        return false;
                    }
                }


                //check for distance
                double min_X = Math.Min(w.XLoc, this.VList[listNeighbors[i, 1]].XLoc) - offset;
                double min_Y = Math.Min(w.YLoc, this.VList[listNeighbors[i, 1]].YLoc) - offset;
                double Max_X = Math.Max(w.XLoc, this.VList[listNeighbors[i, 1]].XLoc) + offset;
                double Max_Y = Math.Max(w.YLoc, this.VList[listNeighbors[i, 1]].YLoc) + offset;
                Microsoft.Msagl.Core.Geometry.Point a = new Microsoft.Msagl.Core.Geometry.Point(min_X, min_Y);
                Microsoft.Msagl.Core.Geometry.Point b = new Microsoft.Msagl.Core.Geometry.Point(Max_X, Max_Y);

                Rectangle queryRectangle = new Rectangle(a, b);

                int[] candidateVertex = this.nodeTree.GetAllIntersecting(queryRectangle);

                //check for distance
                for (int index = 0; index < candidateVertex.Length; index++)
                {
                    Vertex z = this.VList[candidateVertex[index]];
                    if (z.Id == w.Id || z.Id == this.VList[listNeighbors[i, 1]].Id) {
                        continue;
                    }

                    //distance from z to w,i
                    if (PointToSegmentDistance.GetDistance(w, this.VList[listNeighbors[i, 1]], z) < this._edgeNodeSeparation[0])
                    {
                        return false;
                    }
                }

            }
            return true;
        }

        public bool GoodResolution(Vertex w, int[,] listNeighbors, int numNeighbors, WeightedPoint[] pt, int numPoints)
        {
            for (int i = 1; i <= numNeighbors; i++)
            {
                for (int j = i + 1; j <= numNeighbors; j++)
                {
                    //check for angular resolution  


                    //if (Math.Abs(Math.Atan2(yDiff1, xDiff1) - Math.Atan2(yDiff2, xDiff2)) < 0.3)
                    if (Angle.getAngleIfSmallerThanPIby2(w, this.VList[listNeighbors[1, 1]], this.VList[listNeighbors[2, 1]]) < 0.5)
                    {
                        return false;
                    }
                }

                //check for distance
                foreach (var z in this._sNet.V)
                {
                    if (z.Id == w.Id || z.Id == this.VList[listNeighbors[i, 1]].Id || z.Invalid) {
                        continue;
                    }

                    //distance from z to w,i

                    if (PointToSegmentDistance.GetDistance(w, this.VList[listNeighbors[i, 1]], z) < this._edgeNodeSeparation[this.EList[w.Id, listNeighbors[i, 2]].Selected])
                    {
                        return false;
                    }
                }

            }
            return true;
        }
        public bool IsWellSeperated(Vertex w, int w1, int w2, WeightedPoint[] pt, int numPoints)
        {

            //add the edge if they are not very close to a point
            for (int index = 1; index <= numPoints; index++)
            {
                if (this.VList[pt[index].GridPoint].Invalid) {
                    continue;
                }

                if ((pt[index].X == this.VList[w1].XLoc && pt[index].Y == this.VList[w1].YLoc)
                    || (pt[index].X == this.VList[w2].XLoc && pt[index].Y == this.VList[w2].YLoc)) {
                    continue;
                }

                int k = 0;
                for (int neighb = 1; neighb <= this.DegList[w.Id]; neighb++) {
                    if (this.EList[w.Id, neighb].NodeId == w1) { k = neighb; break; }
                }

                if (PointToSegmentDistance.GetDistance(this.VList[w1], this.VList[w2], this.VList[pt[index].GridPoint]) < this._edgeNodeSeparation[this.EList[w.Id, k].Selected])
                {
                    return false;
                }
            }


            //check for angular resolution at neighbor1
            for (int neighb = 1; neighb <= this.DegList[w1]; neighb++)
            {
                if (this.EList[w1, neighb].Used == 0) {
                    continue;
                }

                if (this.EList[w1, neighb].NodeId == w.Id || this.EList[w1, neighb].NodeId == w2) {
                    continue;
                }


                //if (Math.Abs(Math.Atan2(yDiff1 , xDiff1) - Math.Atan2(yDiff2 , xDiff2)) < 0.5 )
                if (Angle.getAngleIfSmallerThanPIby2(w, this.VList[w1], this.VList[w2]) < this._angularResolution)
                {
                    return false;
                }
            }

            //check for angular resolution at neighbor2
            for (int neighb = 1; neighb <= this.DegList[w2]; neighb++)
            {
                if (this.EList[w2, neighb].Used == 0) {
                    continue;
                }

                if (this.EList[w2, neighb].NodeId == w.Id || this.EList[w2, neighb].NodeId == w1) {
                    continue;
                }

                float xDiff1 = this.VList[w2].XLoc - this.VList[w1].XLoc;
                float yDiff1 = this.VList[w2].YLoc - this.VList[w1].YLoc;

                float xDiff2 = this.VList[w2].XLoc - this.VList[this.EList[w2, neighb].NodeId].XLoc;
                float yDiff2 = this.VList[w2].YLoc - this.VList[this.EList[w2, neighb].NodeId].YLoc;

                if (Math.Abs(Math.Atan2(yDiff1, xDiff1) - Math.Atan2(yDiff2, xDiff2)) < this._angularResolution)
                {
                    return false;
                }
            }
            return true;
        }



        public bool MsaglIsWellSeperated(Vertex w, int w1, int w2)
        {
            //check if you need to add or subtract some offset while you are doing the query
            var p1 = new Point(this.VList[w1].XLoc, this.VList[w1].YLoc);
            var p2 = new Point(this.VList[w2].XLoc, this.VList[w2].YLoc);
            Rectangle queryRectangle = new Rectangle(p1, p2);
            int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);

            //add the edge w1w2 if they are not very close to a point O(number of points inside rectangle w1 w2)
            for (int q = 0; q < candidateList.Length; q++)
            {
                int index = candidateList[q];

                //if vertex is one of the neighbors forget it
                if (this.VList[index].Invalid || this.DegList[index] == 0 || this.VList[index].Id == w1 || this.VList[index].Id == w2 || this.VList[index].Id == w.Id) {
                    continue;
                }

                //otherwise find distance from index to w1w2                
                if (PointToSegmentDistance.GetDistance(this.VList[w1], this.VList[w2], this.VList[index]) < this._edgeNodeSeparation[0])
                {
                    return false;
                }
            }

            //check for angular resolution at neighbor1 O(1)
            for (int neighb = 0; neighb < this.DegList[w1]; neighb++)
            {
                if (this.EList[w1, neighb].NodeId == w.Id || this.EList[w1, neighb].NodeId == w2) {
                    continue;
                }

                if (Angle.getAngleIfSmallerThanPIby2(this.VList[w1], this.VList[w2], this.VList[this.EList[w1, neighb].NodeId]) < this._angularResolution) {
                    return false;
                }
            }

            //check for angular resolution at neighbor2  O(1)
            for (int neighb = 0; neighb < this.DegList[w2]; neighb++)
            {
                if (this.EList[w2, neighb].NodeId == w.Id || this.EList[w2, neighb].NodeId == w1) {
                    continue;
                }
                //if (Math.Abs(Math.Atan2(yDiff1, xDiff1) - Math.Atan2(yDiff2, xDiff2)) < AngularResolution)
                if (Angle.getAngleIfSmallerThanPIby2(this.VList[w2], this.VList[w1], this.VList[this.EList[w2, neighb].NodeId]) < this._angularResolution) {
                    return false;
                }
            }

            return true;
        }


        public void MsaglRemoveDeg2(Dictionary<int, Node> idToNodes)
        {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            int[,] listNeighbors = new int[20, 3];

            bool localRefinementsFound = true;
            int iteration = 20;


            List<int> Deg2Vertices = new List<int>();
            for (int index = this.N; index < this.NumOfnodes; index++)
            {
                Vertex w = this.VList[index];
                 

                var numNeighbors = 0;
                //compute the deg of w
                for (int k = 0; k < this.DegList[w.Id]; k++)
                {
                    numNeighbors++;
                    listNeighbors[numNeighbors, 1] = this.EList[w.Id, k].NodeId;
                    listNeighbors[numNeighbors, 2] = k;
                }
                //if deg is 1 fix it
                if (numNeighbors == 1) {
                    this.DegList[index] = 0;
                }
                //if deg is 2 then add in the list
                if (numNeighbors == 2) {
                    Deg2Vertices.Add(index);
                }
            }

            while (localRefinementsFound && iteration > 0)
            {
                iteration--;
                localRefinementsFound = false;
                foreach(int index in Deg2Vertices)// (int index = N; index < NumOfnodes; index++)
                {
                    Vertex w = this.VList[index];
                    
                    if(w.Invalid) {
                        continue;
                    }

                    var numNeighbors = 0;

                    for (int k = 0; k < this.DegList[w.Id]; k++)
                    { 
                        numNeighbors++;
                        listNeighbors[numNeighbors, 1] = this.EList[w.Id, k].NodeId;
                        listNeighbors[numNeighbors, 2] = k;
                    }

                    if (numNeighbors == 1) {
                        this.DegList[index] = 0;
                    }

                    if (numNeighbors == 2)
                    {

                        var adjust = this.MsaglIsWellSeperated(w, listNeighbors[1, 1], listNeighbors[2, 1]);
                        adjust = adjust && this.noCrossings(w, this.VList[listNeighbors[1, 1]], this.VList[listNeighbors[2, 1]]);

                        if (adjust)
                        {
                            localRefinementsFound = true;
                            var selected = this.EList[index, listNeighbors[2, 2]].Selected;
                            var used = this.EList[index, listNeighbors[2, 2]].Used;
                            this.RemoveEdge(index, listNeighbors[1, 1]);
                            this.RemoveEdge(index, listNeighbors[2, 1]);

                            this.AddEdge(listNeighbors[1, 1], listNeighbors[2, 1], selected, used);

                            if (this.DegList[w.Id] == 0) {
                                w.Invalid = true;
                            }
                        }
                         
                    }
                }
            }
            Deg2Vertices.Clear();
#endif
        }
        public void RemoveDeg2(WeightedPoint[] pt, int numPoints)
        {
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            int[,] listNeighbors = new int[20, 3];

            bool localRefinementsFound = true;
            int iteration = 100;


            while (localRefinementsFound && iteration > 0)
            {
                iteration--;
                localRefinementsFound = false;
                foreach (Vertex w in this._sNet.V)
                {
                    var numNeighbors = 0;
                    if (w.Weight > 0) {
                        continue;
                    }

                    for (int k = 1; k <= this.DegList[w.Id]; k++)
                    {
                        if (this.EList[w.Id, k].Used > 0)
                        {
                            numNeighbors++;
                            listNeighbors[numNeighbors, 1] = this.EList[w.Id, k].NodeId;
                            listNeighbors[numNeighbors, 2] = k;
                        }
                    }
                    if (numNeighbors <= 1)
                    {
                        w.CId = 0;
                        w.Invalid = true;
                    }
                    if (numNeighbors == 2)
                    {
                        var adjust = this.IsWellSeperated(w, listNeighbors[1, 1], listNeighbors[2, 1], pt, numPoints);

                        //dont remove if length is already large
                        //if (GetEucledianDist(w.Id, listNeighbors[1, 1]) > 10 ||
                        //GetEucledianDist(w.Id, listNeighbors[2, 1]) > 10)
                        //adjust = false;

                        if (adjust)
                        {
                            localRefinementsFound = true;

                            for (int j = 1; j <= this.DegList[listNeighbors[2, 1]]; j++)
                            {
                                if (this.EList[listNeighbors[2, 1], j].NodeId == w.Id)
                                {
                                    adjust = true;
                                    //check if it already exists in the neighbor list
                                    for (int check = 1; check <= this.DegList[listNeighbors[2, 1]]; check++) {
                                        if (this.EList[listNeighbors[2, 1], check].NodeId == listNeighbors[1, 1])
                                        {
                                            this.EList[listNeighbors[2, 1], check].Selected = this.EList[w.Id, listNeighbors[2, 2]].Selected;
                                            this.EList[listNeighbors[2, 1], check].Used = this.EList[w.Id, listNeighbors[2, 2]].Used;
                                            this.EList[listNeighbors[2, 1], j].Selected = 0;
                                            this.EList[listNeighbors[2, 1], j].Used = 0;
                                            adjust = false;
                                        }
                                    }

                                    if (adjust)
                                    {
                                        this.EList[listNeighbors[2, 1], j].NodeId = listNeighbors[1, 1];
                                        //eList[listNeighbors[2, 1], j].Selected = 8;
                                        //eList[listNeighbors[2, 1], j].Used = 8;
                                    }

                                }
                            }

                            for (int i = 1; i <= this.DegList[listNeighbors[1, 1]]; i++)
                            {
                                if (this.EList[listNeighbors[1, 1], i].NodeId == w.Id)
                                {
                                    adjust = true;
                                    //check if it already exists in the neighbor list
                                    for (int check = 1; check <= this.DegList[listNeighbors[1, 1]]; check++) {
                                        if (this.EList[listNeighbors[1, 1], check].NodeId == listNeighbors[2, 1])
                                        {
                                            this.EList[listNeighbors[1, 1], check].Selected = this.EList[w.Id, listNeighbors[1, 2]].Selected;
                                            this.EList[listNeighbors[1, 1], check].Used = this.EList[w.Id, listNeighbors[1, 2]].Used;
                                            this.EList[listNeighbors[1, 1], i].Selected = 0;
                                            this.EList[listNeighbors[1, 1], i].Used = 0;
                                            adjust = false;
                                        }
                                    }

                                    if (adjust)
                                    {
                                        this.EList[listNeighbors[1, 1], i].NodeId = listNeighbors[2, 1];
                                        //eList[listNeighbors[1, 1], i].Selected = 8;
                                        //eList[listNeighbors[1, 1], i].Used = 8;
                                    }
                                }
                            }



                            //delete old edges

                            this.EList[w.Id, listNeighbors[1, 2]].Selected = 0;
                            this.EList[w.Id, listNeighbors[1, 2]].Used = 0;
                            this.EList[w.Id, listNeighbors[2, 2]].Selected = 0;
                            this.EList[w.Id, listNeighbors[2, 2]].Used = 0;

                            //remove the vertex                            
                            w.Invalid = true;
                            w.CId = 0;

                        }
                    }
                }
            }
#endif
        }

        public void MsaglDetour(Dictionary<int, Node> idToNode, bool omit)
        {
            this.NumOfnodesBeforeDetour = this.NumOfnodes;

            if(omit) {
                return;
            }

            for (int index = 0; index < this.N; index++)
            {
                Vertex w = this.VList[index];
                Vertex[] list = new Vertex[10];
                int separation = 30;
                int neighbor;


#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
                throw new InvalidOperationException();
#else
                int[,] removelist = new int[10, 2];
                int[,] addlist = new int[10, 4];

                int remove = 0;
                int add = 0;
                int newnode = 0;

                for (neighbor = 0; neighbor < this.DegList[index]; neighbor++)
                {

                    int a = this.NumOfnodes;
                    Vertex b = this.VList[this.EList[index, neighbor].NodeId];



                    if (w.YLoc == b.YLoc && w.XLoc > b.XLoc)
                    {
                        int exists = this.GetNode(w.XLoc - separation, w.YLoc);
                        if (exists == -1)
                        {
                            this.VList[a] = new Vertex(w.XLoc - separation, w.YLoc) { Id = a };
                            removelist[remove, 0] = w.Id; removelist[remove, 1] = b.Id; remove++;
                            addlist[add, 0] = w.Id; addlist[add, 1] = a;
                            addlist[add, 2] = b.Id; addlist[add, 3] = a; add++;
                            list[newnode++] = this.VList[a];
                            this.NumOfnodes++;
                        }
                        else { list[newnode++] = this.VList[exists]; }
                    }

                    if (w.YLoc == b.YLoc && w.XLoc < b.XLoc)
                    {
                        int exists = this.GetNode(w.XLoc + separation, w.YLoc);
                        if (exists == -1)
                        {
                            this.VList[a] = new Vertex(w.XLoc + separation, w.YLoc) { Id = a };
                            removelist[remove, 0] = w.Id; removelist[remove, 1] = b.Id; remove++;
                            addlist[add, 0] = w.Id; addlist[add, 1] = a;
                            addlist[add, 2] = b.Id; addlist[add, 3] = a; add++;
                            list[newnode++] = this.VList[a];
                            this.NumOfnodes++;
                        }
                        else {
                            list[newnode++] = this.VList[exists];
                        }
                    }

                    if (w.XLoc == b.XLoc && w.YLoc > b.YLoc)
                    {
                        int exists = this.GetNode(w.XLoc, w.YLoc - separation);
                        if (exists == -1)
                        {
                            this.VList[a] = new Vertex(w.XLoc, w.YLoc - separation) { Id = a };
                            removelist[remove, 0] = w.Id; removelist[remove, 1] = b.Id; remove++;
                            addlist[add, 0] = w.Id; addlist[add, 1] = a;
                            addlist[add, 2] = b.Id; addlist[add, 3] = a; add++;
                            list[newnode++] = this.VList[a];
                            this.NumOfnodes++;
                        }
                        else {
                            list[newnode++] = this.VList[exists];
                        }
                    }

                    if (w.XLoc == b.XLoc && w.YLoc < b.YLoc)
                    {
                        int exists = this.GetNode(w.XLoc, w.YLoc + separation);
                        if (exists == -1)
                        {
                            this.VList[a] = new Vertex(w.XLoc, w.YLoc + separation) { Id = a };
                            removelist[remove, 0] = w.Id; removelist[remove, 1] = b.Id; remove++;
                            addlist[add, 0] = w.Id; addlist[add, 1] = a;
                            addlist[add, 2] = b.Id; addlist[add, 3] = a; add++;
                            list[newnode++] = this.VList[a];
                            this.NumOfnodes++;
                        }
                        else {
                            list[newnode++] = this.VList[exists];
                        }
                    }
                }

                for (int i = 0; i < remove; i++) {
                    this.RemoveEdge(removelist[i, 0], removelist[i, 1]);
                }

                for (int i = 0; i < add; i++)
                {
                    this.AddEdge(addlist[i, 0], addlist[i, 1], 1, 0);
                    this.AddEdge(addlist[i, 2], addlist[i, 3]);
                }

                int removeA = 0, removeB = 0;
                for (int i = 0; i < newnode; i++)
                {
                    for (int j = i + 1; j < newnode; j++)
                    {
                        if (list[i] == null || list[j] == null) {
                            continue;
                        }

                        if (list[i].XLoc == list[j].XLoc || list[i].YLoc == list[j].YLoc) {
                            continue;
                        }

                        if (this.AddEdge(list[i].Id, list[j].Id))
                        {
                            removeA = list[i].Id;
                            removeB = list[j].Id;
                        }
                    }
                }
                //remove one edge
                if (removeA + removeB > 0) {
                    this.RemoveEdge(removeA, removeB);
                }
#endif
            }
        }

        public void CreateNodeTreeEdgeTree()
        {
            this.nodeTree.Clear();
            for (int index = 0; index < this.NumOfnodes; index++)
            {
                if (this.DegList[index] > 0) {
                    this.nodeTree.Add(new Rectangle(new Point(this.VList[index].XLoc, this.VList[index].YLoc)), index);
                }
            }

            this.edgeTree.Clear();
        }

        public void ComputeDetourAroundVertex(WeightedPoint[] pt, int numPoints)
        {
            for (int i = numPoints; i >= 1; i--)
            {
                var x = pt[i].X;
                var y = pt[i].Y;
                int neighb;

                if (x + 1 < this.N && y + 1 < this.N && this.NodeMap[x + 1, y + 1] > 0 &&
                    x + 2 < this.N && y < this.N && this.NodeMap[x + 2, y] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x + 1, y + 1]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x + 1, y + 1], neighb].NodeId == this.VList[this.NodeMap[x + 2, y]].Id)
                        {
                            this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x + 1, y + 1]], this.VList[this.EList[this.NodeMap[x + 1, y + 1], neighb].NodeId], 6);
                            break;
                        }
                    }
                }

                if (x + 1 > 0 && y - 1 > 0 && this.NodeMap[x + 1, y - 1] > 0 &&
                    x + 2 < this.N && y < this.N && this.NodeMap[x + 2, y] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x + 1, y - 1]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x + 1, y - 1], neighb].NodeId == this.VList[this.NodeMap[x + 2, y]].Id)
                        {
                            this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x + 1, y - 1]], this.VList[this.EList[this.NodeMap[x + 1, y - 1], neighb].NodeId], 6);
                            break;
                        }
                    }
                }

                if (x - 1 > 0 && y - 1 > 0 && this.NodeMap[x - 1, y - 1] > 0 &&
                    x - 2 > 0 && y > 0 && this.NodeMap[x - 2, y] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x - 1, y - 1]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x - 1, y - 1], neighb].NodeId == this.VList[this.NodeMap[x - 2, y]].Id)
                        {
                            this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x - 1, y - 1]], this.VList[this.EList[this.NodeMap[x - 1, y - 1], neighb].NodeId], 6);
                            break;
                        }
                    }
                }

                if (x - 1 > 0 && y + 1 < this.N && this.NodeMap[x - 1, y + 1] > 0 &&
                    x - 2 > 0 && y > 0 && this.NodeMap[x - 2, y] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x - 1, y + 1]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x - 1, y + 1], neighb].NodeId == this.VList[this.NodeMap[x - 2, y]].Id)
                        {
                            this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x - 1, y + 1]], this.VList[this.EList[this.NodeMap[x - 1, y + 1], neighb].NodeId], 6);
                            break;
                        }
                    }
                }

                if (x - 1 > 0 && y + 1 < this.N && this.NodeMap[x - 1, y + 1] > 0 &&
                   x + 1 < this.N && y + 1 < this.N && this.NodeMap[x + 1, y + 1] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x - 1, y + 1]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x - 1, y + 1], neighb].NodeId == this.VList[this.NodeMap[x + 1, y + 1]].Id)
                        {
                            this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x - 1, y + 1]], this.VList[this.EList[this.NodeMap[x - 1, y + 1], neighb].NodeId], 6);
                            break;
                        }
                    }
                }
            }
        }

        public void ComputeShortcutMesh(WeightedPoint[] pt, int numPoints)
        {
            //COMPUTE NEIGHBORHOOD SHORTCUTS
            for (int i = numPoints; i >= 1; i--)
            {
                var x = pt[i].X;
                var y = pt[i].Y;

                //if v_i has a neighbor in the first (top right) quadrant 
                int neighb;
                while (x + 1 < this.N && y + 1 < this.N && this.NodeMap[x + 1, y + 1] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y + 1]].Id) {
                            break;
                        }
                    }
                    if (this.EList[this.NodeMap[x, y], neighb].Selected == 0) {
                        break;
                    }

                    x = x + 1;
                    y = y + 1;
                }
                while (x + 1 < this.N && y + 1 < this.N && this.NodeMap[x + 1, y + 1] > 0 && this.VList[this.NodeMap[x + 1, y + 1]].CId == 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y + 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x + 1;
                    y = y + 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.V.Add(this.VList[this.NodeMap[x, y]]);
                }
                if (x + 1 < this.N && y + 1 < this.N && this.NodeMap[x + 1, y + 1] > 0 && this.VList[this.NodeMap[x + 1, y + 1]].CId > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y + 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x + 1;
                    y = y + 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }

                x = pt[i].X;
                y = pt[i].Y;

                //if v_i has a neighbor in the top left quadrant 
                while (x - 1 > 0 && y + 1 < this.N && this.NodeMap[x - 1, y + 1] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y + 1]].Id) {
                            break;
                        }
                    }
                    if (this.EList[this.NodeMap[x, y], neighb].Selected == 0) {
                        break;
                    }

                    x = x - 1;
                    y = y + 1;
                }
                while (x - 1 > 0 && y + 1 < this.N && this.NodeMap[x - 1, y + 1] > 0 && this.VList[this.NodeMap[x - 1, y + 1]].CId == 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y + 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x - 1;
                    y = y + 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }
                if (x - 1 > 0 && y + 1 < this.N && this.NodeMap[x - 1, y + 1] > 0 && this.VList[this.NodeMap[x - 1, y + 1]].CId > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y + 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x - 1;
                    y = y + 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }

                x = pt[i].X;
                y = pt[i].Y;

                //if v_i has a neighbor in the bottom right quadrant 
                while (x + 1 < this.N && y - 1 > 0 && this.NodeMap[x + 1, y - 1] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y - 1]].Id) {
                            break;
                        }
                    }
                    if (this.EList[this.NodeMap[x, y], neighb].Selected == 0) {
                        break;
                    }

                    x = x + 1;
                    y = y - 1;
                }
                while (x + 1 < this.N && y - 1 > 0 && this.NodeMap[x + 1, y - 1] > 0 && this.VList[this.NodeMap[x + 1, y - 1]].CId == 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y - 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x + 1;
                    y = y - 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }
                if (x + 1 < this.N && y - 1 > 0 && this.NodeMap[x + 1, y - 1] > 0 && this.VList[this.NodeMap[x + 1, y - 1]].CId > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x + 1, y - 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x + 1;
                    y = y - 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }

                x = pt[i].X;
                y = pt[i].Y;

                //if v_i has a neighbor in the bottom-left quadrant 
                while (x - 1 > 0 && y - 1 > 0 && this.NodeMap[x - 1, y - 1] > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y - 1]].Id) {
                            break;
                        }
                    }
                    if (this.EList[this.NodeMap[x, y], neighb].Selected == 0) {
                        break;
                    }

                    x = x - 1;
                    y = y - 1;
                }
                while (x - 1 > 0 && y - 1 > 0 && this.NodeMap[x - 1, y - 1] > 0 && this.VList[this.NodeMap[x - 1, y - 1]].CId == 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y - 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x - 1;
                    y = y - 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }
                if (x - 1 > 0 && y - 1 > 0 && this.NodeMap[x - 1, y - 1] > 0 && this.VList[this.NodeMap[x - 1, y - 1]].CId > 0)
                {
                    for (neighb = 1; neighb <= this.DegList[this.NodeMap[x, y]]; neighb++)
                    {
                        if (this.EList[this.NodeMap[x, y], neighb].NodeId == this.VList[this.NodeMap[x - 1, y - 1]].Id) {
                            break;
                        }
                    }
                    this.SelectEdge(this.EList, this.DegList, this.VList[this.NodeMap[x, y]], this.VList[this.EList[this.NodeMap[x, y], neighb].NodeId], 6);
                    x = x - 1;
                    y = y - 1;
                    this.VList[this.NodeMap[x, y]].CId = 1;
                    this._sNet.AddVertex(this.VList[this.NodeMap[x, y]]);
                }

            }
        }
        public int SelectEdge(Edge[,] eList, int[] degList, Vertex a, Vertex b, int givenLevel)
        {
            int temp = givenLevel;
            for (int neighb = 1; neighb <= degList[a.Id]; neighb++)
            {
                if (eList[a.Id, neighb].NodeId == b.Id)
                {
                    if (eList[a.Id, neighb].Selected == 0)
                    {
                        eList[a.Id, neighb].Selected = givenLevel;
                    }
                    else {
                        temp = eList[a.Id, neighb].Selected;
                    }

                    break;
                }
            }
            for (int neighb = 1; neighb <= degList[b.Id]; neighb++)
            {
                if (eList[b.Id, neighb].NodeId == a.Id)
                {
                    if (eList[b.Id, neighb].Selected == 0)
                    {
                        eList[b.Id, neighb].Selected = givenLevel;
                    }
                    else {
                        temp = eList[b.Id, neighb].Selected;
                    }

                    break;
                }
            }
            return temp;
        }

        public bool AddEdge(int a, int b)
        {
            for (int index = 0; index < this.DegList[a]; index++)
            {
                if (this.EList[a, index].NodeId == b) {
                    return false;
                }
            }
            for (int index = 0; index < this.DegList[b]; index++)
            {
                if (this.EList[b, index].NodeId == a) {
                    return false;
                }
            }
            this.EList[a, this.DegList[a]] = new Edge(b);
            this.DegList[a]++;
            this.EList[b, this.DegList[b]] = new Edge(a);
            this.DegList[b]++;
            return true;
        }

        public bool IsAnEdge(int a, int b)
        {
            for (int index = 0; index < this.DegList[a]; index++)
            {
                if (this.EList[a, index].NodeId == b) {
                    return true;
                }
            }
            return false;
        }

        public bool AddEdge(int a, int b, int select, int zoomLevel)
        {
            for (int index = 0; index < this.DegList[a]; index++)
            {
                if (this.EList[a, index].NodeId == b) {
                    return false;
                }
            }
            for (int index = 0; index < this.DegList[b]; index++)
            {
                if (this.EList[b, index].NodeId == a) {
                    return false;
                }
            }
            this.EList[a, this.DegList[a]] = new Edge(b) { Selected = select, Used = zoomLevel };
            this.DegList[a]++;
            this.EList[b, this.DegList[b]] = new Edge(a) { Selected = select, Used = zoomLevel };
            this.DegList[b]++;
            return true;
        }
        public int GetNode(int a, int b)
        {
            for (int index = 0; index < this.NumOfnodes; index++) {
                if (a == this.VList[index].XLoc && b == this.VList[index].YLoc && this.VList[index].Invalid == false) {
                    return index;
                }
            }

            return -1;
        }
        public int GetNodeOtherthanThis(int givenNodeId, int a, int b)
        {
            for (int index = 0; index < this.NumOfnodes; index++) {
                if (a == this.VList[index].XLoc && b == this.VList[index].YLoc && this.VList[index].Invalid == false && index != givenNodeId) {
                    return index;
                }
            }

            return -1;
        }
        public int GetNodeExceptTheGivenNode(Vertex w, int a, int b, int offset)
        {
            Microsoft.Msagl.Core.Geometry.Point p1 = new Microsoft.Msagl.Core.Geometry.Point(a - offset, b - offset);
            Microsoft.Msagl.Core.Geometry.Point p2 = new Microsoft.Msagl.Core.Geometry.Point(a + offset, b + offset);
            Rectangle queryRectangle = new Rectangle(p1, p2);
            int[] candidateList = this.nodeTree.GetAllIntersecting(queryRectangle);
            for (int index = 0; index < candidateList.Length; index++)
            {
                int candidate = candidateList[index];
                if (w.Id != candidate && a == this.VList[candidate].XLoc && b == this.VList[candidate].YLoc &&
                    this.VList[candidate].Invalid == false) {
                    return candidate;
                }
            }
            return -1;
        }
        public bool RemoveEdge(int a, int b)
        {
            int i = 0;
            for (int index = 0; index < this.DegList[a]; index++)
            {
                if (this.EList[a, index].NodeId == b)
                {
                    this.DegList[a]--;
                    i++;
                    for (; index < this.DegList[a]; index++) { this.EList[a, index] = this.EList[a, index + 1]; }
                }
            }
            for (int index = 0; index < this.DegList[b]; index++)
            {
                if (this.EList[b, index].NodeId == a)
                {
                    this.DegList[b]--;
                    i++;
                    for (; index < this.DegList[b]; index++) {
                        this.EList[b, index] = this.EList[b, index + 1];
                    }
                }
            }

            return i == 2;
        }
    }
    public class Vertex
    {
        public int Id;
        public int CId; //component ID for steiner tree
        public int XLoc;
        public int YLoc;

        public double PreciseX;
        public double PreciseY;
        public double TargetX;
        public double TargetY;

        public double LeftX;
        public double LeftY;
        public double RightX;
        public double RightY;

        public double Dist = 0;
        public double Weight = 0; // priority
        public int ZoomLevel = 0;
        public Vertex Parent = null;
        public bool Visited;
        public bool Invalid;

        public Ray topRay;
        public Ray bottomRay;
        public Ray leftRay;
        public Ray rightRay;

        public Dictionary<LineSegment, bool> SegmentList = new Dictionary<LineSegment, bool>();

        public Vertex(int a, int b)
        {
            this.XLoc = a;
            this.YLoc = b;
        }


    }

    public class Ray
    {
        public LineSegment L;
        public bool dead;
        public Ray(LineSegment segment)
        {
            this.L = segment;
        }
    }

    public class Edge
    {
        public double Weight = 1;
        //public double EDist;
        public int EdgeId;
        public int Cost = 0;
        public int NodeId;
        public int Selected;
        public int Used;
        public Edge(int z)
        {
            this.NodeId = z;
        }

        public double GetEDist(Vertex[] vList, int a, int b)
        {
            return Math.Sqrt((vList[a].XLoc - vList[b].XLoc) * (vList[a].XLoc - vList[b].XLoc) + (vList[a].YLoc - vList[b].YLoc) * (vList[a].YLoc - vList[b].YLoc));
        }
    }

    public class ShortestPathEdgeList
    {
        public List<VertexNeighbor> Edgelist = new List<VertexNeighbor>();
    }

}
