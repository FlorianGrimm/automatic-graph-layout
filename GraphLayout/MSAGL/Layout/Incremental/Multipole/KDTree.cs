using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Layout.Incremental
{
    /// <summary>
    /// A KDTree recursively divides particles in a 2D space into a balanced tree structure by doing horizontal splits for wide bounding boxes and vertical splits for tall bounding boxes.
    /// </summary>
    public class KDTree
    {
        private Particle[] particles;
        internal InternalKdNode root;
        private List<LeafKdNode> leaves;
        private Particle[] particlesBy(Particle.Dim d)
        {
            return (from Particle p in this.particles
                    orderby p.pos(d)
                    select p).ToArray();
        }

        /// <summary>
        /// Create a KDTree over the specified particles, with the leaf partitions each containing bucketSize particles.
        /// </summary>
        /// <param name="particles"></param>
        /// <param name="bucketSize"></param>
        public KDTree(Particle[] particles, int bucketSize)
        {
            this.particles = particles;
            Particle[][] ps = new Particle[][] {
                this.particlesBy(Particle.Dim.Horizontal),
                this.particlesBy(Particle.Dim.Vertical)};
            this.leaves = new List<LeafKdNode>();
            LeafKdNode l = new LeafKdNode(ps), r;
            this.leaves.Add(l);
            this.root = l.Split(out r);
            this.leaves.Add(r);
            var splitQueue = new SplitQueue(bucketSize);
            splitQueue.Enqueue(l, r);
            while (splitQueue.Count > 0)
            {
                l = splitQueue.Dequeue();
                l.Split(out r);
                this.leaves.Add(r);
                splitQueue.Enqueue(l, r);
            }
        }
        /// <summary>
        /// Compute forces between particles using multipole approximations.
        /// </summary>
        /// <param name="precision"></param>
        public void ComputeForces(int precision) {
            this.root.computeMultipoleCoefficients(precision);
            foreach (var l in this.leaves)
            {
                l.ComputeForces();
                var stack = new Stack<KdNode>();
                stack.Push(this.root);
                while (stack.Count > 0)
                {
                    KdNode v = stack.Pop();
                    if (!l.intersects(v))
                    {
                        foreach (var p in l.particles[0])
                        {
                            p.force -= v.multipoleCoefficients.ApproximateForce(p.point);
                        }
                    }
                    else
                    {
                        var leaf = v as LeafKdNode;    
                        if (leaf!=null)
                        {
                            foreach (var p in l.particles[0])
                            {
                                foreach (var q in leaf.particles[0])
                                {
                                    if(p!=q) {
                                        p.force += MultipoleCoefficients.Force(p.point, q.point);
                                    }
                                }
                            }
                        }
                        else
                        {
                            var n = v as InternalKdNode;
                            stack.Push(n.leftChild);
                            stack.Push(n.rightChild);
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Particles used in KDTree multipole force approximations
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1034:NestedTypesShouldNotBeVisible")]
        public class Particle
        {
            internal enum Dim { Horizontal = 0, Vertical = 1 };
            internal Point force;
            internal Point point;
            internal bool splitLeft;
            internal double pos(Dim d)
            {
                return d == Dim.Horizontal ? this.point.X : this.point.Y;
            }
            /// <summary>
            /// Create particle at point
            /// </summary>
            /// <param name="point"></param>
            public Particle(Point point)
            {
                this.point = point;
                this.force = new Point(0, 0);
            }
        }
        internal abstract class KdNode
        {
            internal InternalKdNode parent;
            internal Disc med;
            internal MultipoleCoefficients multipoleCoefficients;
            internal bool intersects(KdNode v)
            {
                Point d = v.med.Center - this.med.Center;
                double l = d.Length;
                return l < v.med.Radius + this.med.Radius;
            }
            internal abstract void computeMultipoleCoefficients(int precision);
        }
        internal class LeafKdNode : KdNode
        {
            internal Particle[][] particles;
            internal Point[] ps;
            internal LeafKdNode(Particle[][] particles)
            {
                Debug.Assert(particles[0].Length == particles[1].Length);
                this.particles = particles;
                this.ComputeMED();
            }
            internal override void computeMultipoleCoefficients(int precision)
            {
                this.multipoleCoefficients = new MultipoleCoefficients(precision, this.med.Center, this.ps);
            }
            internal Disc ComputeMED()
            {
                int n = this.Size();
                this.ps = new Point[n];
                for (int i = 0; i < n; ++i)
                {
                    this.ps[i] = this.particles[0][i].point;
                }
                return this.med = MinimumEnclosingDisc.LinearComputation(this.ps);
            }
            private double Min(Particle.Dim d)
            {
                return this.particles[(int)d][0].pos(d);
            }
            internal int Size()
            {
                return this.particles[0].Length;
            }
            private double Max(Particle.Dim d)
            {
                return this.particles[(int)d][this.Size() - 1].pos(d);
            }
            private double Dimension(Particle.Dim d)
            {
                return this.Max(d) - this.Min(d);
            }
            internal InternalKdNode Split(out LeafKdNode rightSibling)
            {
                Particle.Dim splitDirection =
                    this.Dimension(Particle.Dim.Horizontal) > this.Dimension(Particle.Dim.Vertical)
                    ? Particle.Dim.Horizontal : Particle.Dim.Vertical;
                Particle.Dim nonSplitDirection =
                    splitDirection == Particle.Dim.Horizontal
                    ? Particle.Dim.Vertical : Particle.Dim.Horizontal;
                int n = this.Size(), nLeft = n / 2, nRight = n - nLeft;
                Particle[][]
                    leftParticles = new Particle[][] { new Particle[nLeft], new Particle[nLeft] },
                    rightParticles = new Particle[][] { new Particle[nRight], new Particle[nRight] };
                int lCtr = 0, rCtr = 0;
                for (int i = 0; i < n; ++i)
                {
                    Particle p = this.particles[(int)splitDirection][i];
                    if (i < nLeft)
                    {
                        leftParticles[(int)splitDirection][i] = p;
                        p.splitLeft = true;
                    }
                    else
                    {
                        rightParticles[(int)splitDirection][i - nLeft] = p;
                        p.splitLeft = false;
                    }
                }
                for (int i = 0; i < n; ++i)
                {
                    Particle p = this.particles[(int)nonSplitDirection][i];
                    if (p.splitLeft)
                    {
                        leftParticles[(int)nonSplitDirection][lCtr++] = p;
                    }
                    else
                    {
                        rightParticles[(int)nonSplitDirection][rCtr++] = p;
                    }
                }
                Debug.Assert(lCtr == nLeft);
                Debug.Assert(rCtr == nRight);
                Disc parentMED = this.med;
                this.particles = leftParticles;
                this.ComputeMED();
                rightSibling = new LeafKdNode(rightParticles);
                return new InternalKdNode(parentMED, this, rightSibling);
            }
            internal void ComputeForces()
            {
                foreach (var u in this.particles[0])
                {
                    foreach (var v in this.particles[0])
                    {
                        if (u != v)
                        {
                            u.force += MultipoleCoefficients.Force(u.point, v.point);
                        }
                    }
                }
            }
        }
        internal class InternalKdNode : KdNode
        {
            internal KdNode leftChild;
            internal KdNode rightChild;
            internal InternalKdNode(Disc med, KdNode left, KdNode right)
            {
                this.med = med;
                this.parent = left.parent;
                if (this.parent != null)
                {
                    if (this.parent.leftChild == left)
                    {
                        this.parent.leftChild = this;
                    }
                    else
                    {
                        Debug.Assert(this.parent.rightChild == left);
                        this.parent.rightChild = this;
                    }
                }
                this.leftChild = left;
                this.rightChild = right;
                left.parent = this;
                right.parent = this;
            }
            internal override void computeMultipoleCoefficients(int precision)
            {
                this.leftChild.computeMultipoleCoefficients(precision);
                this.rightChild.computeMultipoleCoefficients(precision);
                this.multipoleCoefficients = new MultipoleCoefficients(this.med.Center, this.leftChild.multipoleCoefficients, this.rightChild.multipoleCoefficients);
            }
        }

        private class SplitQueue : Queue<LeafKdNode>
        {
            private int B;
            public SplitQueue(int B)
            {
                this.B = B;
            }
            public void Enqueue(LeafKdNode l, LeafKdNode r)
            {
                if (l.Size() > this.B)
                {
                    this.Enqueue(l);
                }
                if (r.Size() > this.B)
                {
                    this.Enqueue(r);
                }
            }
        }
    }
}
