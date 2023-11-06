using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Visibility
{
    [DebuggerDisplay("({Point.X} {Point.Y})")]
    internal class VisibilityVertex : IComparer<VisibilityEdge>
    {

        // This member is accessed a lot.  Using a field instead of a property for performance.
        readonly internal Point Point;
        public bool isReal;
        private bool _isTerminal;
        private bool _isShortestPathTerminal;
        private readonly List<VisibilityEdge> _inEdges = new List<VisibilityEdge>();

     
        internal void AddInEdge(VisibilityEdge e) {
            this._inEdges.Add(e);
        }
        internal int InEdgesCount() {
            return this._inEdges.Count;
        }
        internal IEnumerable<VisibilityEdge> InEdges
        {
            get { return this._inEdges; }
        }

        private readonly RbTree<VisibilityEdge> _outEdges;
        /* VisibilityEdge prev; */

        /// <summary>
        /// this collection is sorted by the target point, in the lexicographical order
        /// </summary>
        internal RbTree<VisibilityEdge> OutEdges
        {
            get { return this._outEdges; }
        }

        internal int Degree
        {
            get { return this.InEdgesCount() + this.OutEdges.Count; }
        }
        /// <summary>
        /// needed for shortest path calculations
        /// </summary>
        internal double Distance { get; set; }

        internal bool IsTerminal
        {
            get { return this._isTerminal; }
            set { this._isTerminal = value; }
        }

        internal bool IsShortestPathTerminal
        {
            get { return this._isShortestPathTerminal; }
            set { this._isShortestPathTerminal = value; }
        }


        /*
                /// <summary>
                /// needed for shortest path calculations
                /// </summary>        
                internal VisibilityVertex Prev {
                    get {
                        if (prev == null) return null;
                        if(prev.Source==this)
                            return prev.Target;
                        return prev.Source;
                    }
                }

                internal void SetPreviousEdge(VisibilityEdge e) {
                    prev = e;
                }
                */
        internal VisibilityVertex(Point point)
        {
            this._outEdges = new RbTree<VisibilityEdge>(this);
            this.Point = point;
        }

        /// <summary>
        /// Rounded representation; DebuggerDisplay shows the unrounded form.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return this.Point.ToString();
        }

        /// <summary>
        /// These iterate from the end of the list because List.Remove is linear in
        /// the number of items, so callers have been optimized where possible to
        /// remove only the last or next-to-last edges (but in some cases such as
        /// rectilinear, this optimization isn't always possible).
        /// </summary>
        /// <param name="edge"></param>
        internal void RemoveOutEdge(VisibilityEdge edge)
        {
            this.OutEdges.Remove(edge);
        }

        internal void RemoveInEdge(VisibilityEdge edge)
        {
            for (int ii = this._inEdges.Count - 1; ii >= 0; --ii)
            {
                if (this._inEdges[ii] == edge)
                {
                    this._inEdges.RemoveAt(ii);
                    break;
                }
            }
        }

        /// <summary>
        /// avoiding using delegates in calling RBTree.FindFirst because of the memory allocations
        /// </summary>
        /// <param name="tree"></param>
        /// <param name="targetPoint"></param>
        /// <returns></returns>
        private static RBNode<VisibilityEdge> FindFirst(RbTree<VisibilityEdge> tree, Point targetPoint)
        {
            return FindFirst(tree.Root, tree, targetPoint);
        }

        private static RBNode<VisibilityEdge> FindFirst(RBNode<VisibilityEdge> n, RbTree<VisibilityEdge> tree, Point targetPoint)
        {
            if (n is null) {
                return null;
            }

            RBNode<VisibilityEdge> good = null;
            while (n != tree.Nil) {
                n = n.Item.TargetPoint >= targetPoint ? (good = n).Left : n.Right;
            }

            return good;
        }

        internal bool TryGetEdge(VisibilityVertex target, out VisibilityEdge visEdge)
        {
            var node = FindFirst(this.OutEdges, target.Point);// OutEdges.FindFirst(e => e.TargetPoint >= target.Point); 
            if (node != null)
            {
                if (node.Item.Target == target)
                {
                    visEdge = node.Item;
                    return true;
                }
            }
            node = FindFirst(target.OutEdges, this.Point);// target.OutEdges.FindFirst(e => e.TargetPoint >= Point);
            if (node != null)
            {
                if (node.Item.Target == this)
                {
                    visEdge = node.Item;
                    return true;
                }
            }
            visEdge = null;
            return false;
        }

        #region IComparer<VisibilityEdge>
        public int Compare(VisibilityEdge a, VisibilityEdge b)
        {
            return a.TargetPoint.CompareTo(b.TargetPoint);
        }
        #endregion // IComparer<VisibilityEdge>

        public void ClearEdges()
        {
            this._outEdges.Clear();
            this._inEdges.Clear();
        }
    }
}
