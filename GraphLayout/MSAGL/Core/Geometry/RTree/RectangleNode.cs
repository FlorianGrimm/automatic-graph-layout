#region Using directives

using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Diagnostics;

#endregion

namespace Microsoft.Msagl.Core.Geometry {
    /// <summary>
    /// Represents a node containing a box and some user data.
    /// Is used in curve intersections routines.
    /// </summary>
    public class RectangleNode<T, P> {
#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.Int32.ToString")]
        public override string ToString() {
            return this.IsLeaf ? (this.Count + " " + this.UserData) : this.Count.ToString();
        }
#endif
        /// <summary>
        /// 
        /// </summary>
        public int Count { get; set; }

        private RectangleNode<T, P>? _Left;
        private RectangleNode<T, P>? _Right;

        /// <summary>
        /// creates an empty node
        /// </summary>
        public RectangleNode() {
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="data"></param>
        /// <param name="rect"></param>
        public RectangleNode(T data, IRectangle<P> rect) {
            this.UserData = data;
            this.Rectangle = rect;
            this.Count = 1;
        }

        private RectangleNode(int count) {
            this.Count = count;
        }

        /// <summary>
        /// This field provides direct internal access to the value type Rectangle, which RTree and other callers
        /// modify directly with .Add(); the auto-property returns a temporary value-by-copy that is immediately discarded.
        /// </summary>
// ReSharper disable InconsistentNaming
        internal IRectangle<P>? IntenalRectangle;
        // ReSharper restore InconsistentNaming

        /// <summary>
        /// gets or sets the rectangle of the node
        /// </summary>
        public IRectangle<P>? Rectangle {
            get { return this.IntenalRectangle; }
            set { this.IntenalRectangle = value; }
        }

        /// <summary>
        /// false if it is an internal node and true if it is a leaf
        /// </summary>
        internal bool IsLeaf {
            get { return this._Left == null /*&& right==null*/; } //if left is a null then right is also a null
        }

        /// <summary>
        /// 
        /// </summary>
        public RectangleNode<T, P>? Left {
            get { return this._Left; }
            internal set {
                this._Left = value;
                if (this._Left != null) {
                    this._Left.Parent = this;
                }
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public RectangleNode<T, P>? Right {
            get { return this._Right; }
            internal set {
                this._Right = value;
                if (this._Right != null) {
                    this._Right.Parent = this;
                }
            }
        }

        /// <summary>
        /// The actual data if a leaf node, else null or a value-type default.
        /// </summary>
        public T? UserData { get; set; }

        /// <summary>
        /// Parent of this node.
        /// </summary>
        public RectangleNode<T, P>? Parent { get; private set; }

        internal bool IsLeftChild {
            get {
                Debug.Assert(this.Parent != null);
                if (this.Parent is null) {
                    // TODO: hack
                    // return false;
                    throw new Exception();
                } else {
                    return this.Equals(this.Parent.Left);
                }
            }
        }

        /// <summary>
        /// brings the first leaf which rectangle was hit and the delegate is happy with the object
        /// </summary>
        /// <param name="point"></param>
        /// <param name="hitTestForPointDelegate"></param>
        /// <returns></returns>
        public RectangleNode<T, P>? FirstHitNode(P point, Func<P, T, HitTestBehavior> hitTestForPointDelegate) {
            if (this.IntenalRectangle is not null && this.IntenalRectangle.Contains(point)) {
                if (this.IsLeaf) {
                    if (hitTestForPointDelegate != null && this.UserData is not null) {
                        return hitTestForPointDelegate(point, this.UserData) == HitTestBehavior.Stop ? this : null;
                    }
                    return this;
                }
                return this.Left?.FirstHitNode(point, hitTestForPointDelegate) ??
                        this.Right?.FirstHitNode(point, hitTestForPointDelegate);
            }
            return null;
        }


        /// <summary>
        /// brings the first leaf which rectangle was intersected
        /// </summary>
        /// <returns></returns>
        public RectangleNode<T, P>? FirstIntersectedNode(IRectangle<P> r) {
            if (this.IntenalRectangle is not null && r.Intersects(this.IntenalRectangle)) {
                if (this.IsLeaf) {
                    return this;
                }

                return this.Left?.FirstIntersectedNode(r) ?? this.Right?.FirstIntersectedNode(r);
            }
            return null;
        }



        /// <summary>
        /// brings the first leaf which rectangle was hit and the delegate is happy with the object
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public RectangleNode<T, P>? FirstHitNode(P point) {
            if (this.IntenalRectangle is not null && this.IntenalRectangle.Contains(point)) {
                if (this.IsLeaf) {
                    return this;
                }

                return this.Left?.FirstHitNode(point) ?? this.Right?.FirstHitNode(point);
            }
            return null;
        }


        /// <summary>
        /// returns all leaf nodes for which the rectangle was hit and the delegate is happy with the object
        /// </summary>
        /// <param name="rectanglePar"></param>
        /// <param name="hitTestAccept"></param>
        /// <returns></returns>
        public IEnumerable<T> AllHitItems(IRectangle<P> rectanglePar, Func<T, bool> hitTestAccept) {
            var stack = new Stack<RectangleNode<T, P>>();
            stack.Push(this);
            while (stack.Count > 0) {
                RectangleNode<T, P> node = stack.Pop();
                if (node.Rectangle is not null && node.Rectangle.Intersects(rectanglePar)) {
                    if (node.IsLeaf) {
                        if (node.UserData is T userData && ((null == hitTestAccept) || hitTestAccept(userData))) {
                            yield return userData;
                        }
                    } else {
                        if (node._Left is not null) {
                            stack.Push(node._Left);
                        }
                        if (node._Right is not null) {
                            stack.Push(node._Right);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// returns all items for which the rectangle contains the point
        /// </summary>
        /// <returns></returns>
        public IEnumerable<T> AllHitItems(P point) {
            var stack = new Stack<RectangleNode<T, P>>();
            stack.Push(this);
            while (stack.Count > 0) {
                var node = stack.Pop();
                if (node.Rectangle is not null
                    && node.Rectangle.Contains(point)) {
                    if (node.IsLeaf) {
                        if (node.UserData is T userData) {
                            yield return userData;
                        }
                    } else {
                        if (node._Left is not null) {
                            stack.Push(node._Left);
                        }
                        if (node._Right is not null) {
                            stack.Push(node._Right);
                        }
                    }
                }
            }
        }


        /// <summary>
        /// Returns all leaves whose rectangles intersect hitRectangle (or all leaves before hitTest returns false).
        /// </summary>
        /// <param name="hitTest"></param>
        /// <param name="hitRectangle"></param>
        /// <returns></returns>
        internal void VisitTree(Func<T, HitTestBehavior> hitTest, IRectangle<P> hitRectangle) {
            VisitTreeStatic(this, hitTest, hitRectangle);
        }

        private static HitTestBehavior VisitTreeStatic(RectangleNode<T, P> rectangleNode, Func<T, HitTestBehavior> hitTest, IRectangle<P> hitRectangle) {
            if (rectangleNode.Rectangle.Intersects(hitRectangle)) {
                if (hitTest(rectangleNode.UserData) == HitTestBehavior.Continue) {
                    if (rectangleNode.Left != null) {
                        // If rectangleNode.Left is not null, rectangleNode.Right won't be either.
                        if (VisitTreeStatic(rectangleNode.Left, hitTest, hitRectangle) == HitTestBehavior.Continue &&
                            VisitTreeStatic(rectangleNode.Right, hitTest, hitRectangle) == HitTestBehavior.Continue) {
                            return HitTestBehavior.Continue;
                        }
                        return HitTestBehavior.Stop;
                    }
                    return HitTestBehavior.Continue;
                }
                return HitTestBehavior.Stop;
            }
            return HitTestBehavior.Continue;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public RectangleNode<T, P> Clone() {
            var ret = new RectangleNode<T, P>(this.Count) { UserData = this.UserData, Rectangle = this.Rectangle };
            if (this.Left != null) {
                ret.Left = this.Left.Clone();
            }

            if (this.Right != null) {
                ret.Right = this.Right.Clone();
            }

            return ret;
        }

        /// <summary>
        /// yields all leaves which rectangles intersect the given one. We suppose that leaves are all nodes having UserData not a null.
        /// </summary>
        /// <param name="rectanglePar"></param>
        /// <returns></returns>
        public IEnumerable<T> GetNodeItemsIntersectingRectangle(IRectangle<P> rectanglePar) {
            return this.GetLeafRectangleNodesIntersectingRectangle(rectanglePar).Select(node => node.UserData);
        }

        /// <summary>
        /// yields all leaves whose rectangles intersect the given one. We suppose that leaves are all nodes having UserData not a null.
        /// </summary>
        /// <param name="rectanglePar"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        public IEnumerable<RectangleNode<T, P>> GetLeafRectangleNodesIntersectingRectangle(IRectangle<P> rectanglePar) {
            var stack = new Stack<RectangleNode<T, P>>();
            stack.Push(this);
            while (stack.Count > 0) {
                RectangleNode<T, P> node = stack.Pop();
                if (node.Rectangle is not null && node.Rectangle.Intersects(rectanglePar)) {
                    if (node.IsLeaf) {
                        yield return node;
                    } else {
                        stack.Push(node._Left);
                        stack.Push(node._Right);
                    }
                }
            }
        }

        /// <summary>
        /// Walk the tree and return the data from all leaves
        /// </summary>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1024:UsePropertiesWhereAppropriate")]
        public IEnumerable<T> GetAllLeaves() {
            return this.GetAllLeafNodes().Select(n => n.UserData);
        }

        internal IEnumerable<RectangleNode<T, P>> GetAllLeafNodes() {
            return this.EnumRectangleNodes(true /*leafOnly*/);
        }

        private IEnumerable<RectangleNode<T, P>> EnumRectangleNodes(bool leafOnly) {
            var stack = new Stack<RectangleNode<T, P>>();
            stack.Push(this);
            while (stack.Count > 0) {
                var node = stack.Pop();
                if (node.IsLeaf || !leafOnly) {
                    yield return node;
                }
                if (!node.IsLeaf) {
                    stack.Push(node._Left);
                    stack.Push(node._Right);
                }
            }
        }

        private const int GroupSplitThreshold = 2;


        /// <summary>
        /// calculates a tree based on the given nodes
        /// </summary>
        /// <param name="nodes"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1000:DoNotDeclareStaticMembersOnGenericTypes"), SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        public static RectangleNode<T, P>? CreateRectangleNodeOnEnumeration(IEnumerable<RectangleNode<T, P>> nodes) {
            if (nodes == null) {
                return null;
            }

            var nodeList = new List<RectangleNode<T, P>>(nodes);
            return CreateRectangleNodeOnListOfNodes(nodeList);
        }

        ///<summary>
        ///calculates a tree based on the given nodes
        ///</summary>
        ///<param name="dataEnumeration"></param>
        ///<param name="rectangleDelegate"></param>
        ///<returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1000:DoNotDeclareStaticMembersOnGenericTypes")]
        public static RectangleNode<T, P>? CreateRectangleNodeOnData(IEnumerable<T> dataEnumeration, Func<T, IRectangle<P>> rectangleDelegate) {
            if (dataEnumeration == null || rectangleDelegate == null) {
                return null;
            }

            var nodeList = new List<RectangleNode<T, P>>(dataEnumeration.Select(d => new RectangleNode<T, P>(d, rectangleDelegate(d))));
            return CreateRectangleNodeOnListOfNodes(nodeList);
        }


        /// <summary>
        /// 
        /// </summary>
        /// <param name="nodes"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1000:DoNotDeclareStaticMembersOnGenericTypes"), SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        static public RectangleNode<T, P>? CreateRectangleNodeOnListOfNodes(IList<RectangleNode<T, P>> nodes) {
            ValidateArg.IsNotNull(nodes, "nodes");
            if (nodes.Count == 0) {
                return null;
            }

            if (nodes.Count == 1) {
                return nodes[0];
            }

            //Finding the seeds

            //the first seed
            int seed0 = 1;

            int seed1 = ChooseSeeds(nodes, ref seed0);

            //We have two seeds at hand. Build two groups.
            var gr0 = new List<RectangleNode<T, P>>();
            var gr1 = new List<RectangleNode<T, P>>();

            gr0.Add(nodes[seed0]);
            gr1.Add(nodes[seed1]);

            var box0 = nodes[seed0].Rectangle;
            var box1 = nodes[seed1].Rectangle;
            //divide nodes on two groups
            DivideNodes(nodes, seed0, seed1, gr0, gr1, ref box0, ref box1, GroupSplitThreshold);

            var ret = new RectangleNode<T, P>(nodes.Count) {
                Rectangle = box0.Unite(box1),
                Left = CreateRectangleNodeOnListOfNodes(gr0),
                Right = CreateRectangleNodeOnListOfNodes(gr1)
            };

            Debug.Assert(RTree<T, P>.TreeIsCorrect(ret));

            return ret;

        }

        private static int ChooseSeeds(IList<RectangleNode<T, P>> nodes, ref int seed0) {
            var b0 = nodes[seed0].Rectangle;
            double area = b0.Area;
            for (int i = 0; i < nodes.Count; i++) {
                if (i == seed0) {
                    continue;
                }

                double area0 = b0.Unite(nodes[i].Rectangle).Area;
                if (area0 > area) {
                    seed0 = i;
                    area = area0;
                }
            }

            //Got the first seed seed0
            //Now looking for a seed for the second group
            int seed1 = 0; //the compiler forces me to init it

            //init seed1
            for (int i = 0; i < nodes.Count; i++) {
                if (i != seed0) {
                    seed1 = i;
                    break;
                }
            }

            area = nodes[seed0].Rectangle.Unite(nodes[seed1].Rectangle).Area;
            //Now try to improve the second seed

            for (int i = 0; i < nodes.Count; i++) {
                if (i == seed0) {
                    continue;
                }

                double area1 = nodes[seed0].Rectangle.Unite(nodes[i].Rectangle).Area;
                if (area1 > area) {
                    seed1 = i;
                    area = area1;
                }
            }

            return seed1;
        }

        private static void DivideNodes(IList<RectangleNode<T, P>> nodes, int seed0, int seed1, List<RectangleNode<T, P>> gr0, List<RectangleNode<T, P>> gr1,
            ref IRectangle<P> box0, ref IRectangle<P> box1, int groupSplitThreshold) {
            for (int i = 0; i < nodes.Count; i++) {

                if (i == seed0 || i == seed1) {
                    continue;
                }

                // ReSharper disable InconsistentNaming
                var box0_ = box0.Unite(nodes[i].Rectangle);
                double delta0 = box0_.Area - box0.Area;

                var box1_ = box1.Unite(nodes[i].Rectangle);
                double delta1 = box1_.Area - box1.Area;
                // ReSharper restore InconsistentNaming

                //keep the tree roughly balanced

                if (gr0.Count * groupSplitThreshold < gr1.Count) {
                    gr0.Add(nodes[i]);
                    box0 = box0_;
                } else if (gr1.Count * groupSplitThreshold < gr0.Count) {
                    gr1.Add(nodes[i]);
                    box1 = box1_;
                } else if (delta0 < delta1) {
                    gr0.Add(nodes[i]);
                    box0 = box0_;
                } else if (delta1 < delta0) {
                    gr1.Add(nodes[i]);
                    box1 = box1_;
                } else if (box0.Area < box1.Area) {
                    gr0.Add(nodes[i]);
                    box0 = box0_;
                } else {
                    gr1.Add(nodes[i]);
                    box1 = box1_;
                }
            }
        }



        /// <summary>
        /// Walk the tree from node down and apply visitor to all nodes
        /// </summary>
        /// <param name="node"></param>
        /// <param name="visitor"></param>
        [SuppressMessage("Microsoft.Design", "CA1000:DoNotDeclareStaticMembersOnGenericTypes"), SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        static public void TraverseHierarchy(RectangleNode<T, P> node, Action<RectangleNode<T, P>> visitor) {
            ValidateArg.IsNotNull(node, "node");
            ValidateArg.IsNotNull(visitor, "visitor");
            visitor(node);
            if (node.Left != null) {
                TraverseHierarchy(node.Left, visitor);
            }

            if (node.Right != null) {
                TraverseHierarchy(node.Right, visitor);
            }
        }
    }
}
