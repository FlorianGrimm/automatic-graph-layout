﻿using System.Collections.Generic;
using System.Linq;

namespace Microsoft.Msagl.GraphmapsWithMesh
{
    internal class SteinerTree
    {
        public Stack<Tuple> Edgelist = new Stack<Tuple>();
        public List<Twin> SpanningTree = new List<Twin>();
        //ComponentCollection _compCollection;
 

    }

    public class Component
    {
        public int CId = 0;
        public double Dist;
        public bool Dead = true;
        public List<Vertex> V = new List<Vertex>();
        public List<VertexNeighbor> Segments = new List<VertexNeighbor>();

        public void AddVertex(Vertex w)
        {
            foreach (Vertex z in this.V)
            {
                if (w.Id == z.Id) {
                    return;
                }
            }
            this.V.Add(w);
        }
    }

    public class Twin
    {
        public int A;
        public int B;
        public Twin(int x, int y) { this.A = x; this.B = y; }

    }
    public class Tuple
    {
        public Vertex A;
        public Vertex B;
        public int Value;
        public Tuple(Vertex x, Vertex y, int z)
        {
            this.A = x; this.B = y; this.Value = z;
        }
    }
}
