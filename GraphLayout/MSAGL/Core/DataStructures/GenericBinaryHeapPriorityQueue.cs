using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace Microsoft.Msagl.Core.DataStructures {


    /// <summary>
    /// A generic version priority queue based on the binary heap algorithm where
    /// the priority of each element is passed as a parameter.
    /// </summary>
    [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1710:IdentifiersShouldHaveCorrectSuffix"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1711:IdentifiersShouldNotHaveIncorrectSuffix")]
    public class GenericBinaryHeapPriorityQueue<T> : IEnumerable<T> {
        private const int InitialHeapCapacity = 16;


        // ReSharper disable InconsistentNaming
        private GenericHeapElement<T>[] A;//array of heap elements
                                         // ReSharper restore InconsistentNaming


        /// <summary>
        /// it is a mapping from queue elements and their correspondent HeapElements
        /// </summary>
        private readonly Dictionary<T, GenericHeapElement<T>> cache;
        internal int Count { get { return this.heapSize; } }

        private int heapSize;

        internal bool ContainsElement(T key) {
            return this.cache.ContainsKey(key);
        }


        internal GenericBinaryHeapPriorityQueue() {
            this.cache = new Dictionary<T, GenericHeapElement<T>>();
            this.A = new GenericHeapElement<T>[InitialHeapCapacity + 1];
        }

        private void SwapWithParent(int i) {
            var parent = this.A[i >> 1];

            this.PutAtI(i >> 1, this.A[i]);
            this.PutAtI(i, parent);
        }



        internal void Enqueue(T element, double priority) {
            if (this.heapSize == this.A.Length - 1) {
                var newA = new GenericHeapElement<T>[this.A.Length * 2];
                Array.Copy(this.A, 1, newA, 1, this.heapSize);
                this.A = newA;
            }

            this.heapSize++;
            int i = this.heapSize;
            this.A[i] = this.cache[element] = new GenericHeapElement<T>(i, priority, element);
            while (i > 1 && this.A[i >> 1].priority.CompareTo(priority) > 0) {
                this.SwapWithParent(i);
                i >>= 1;
            }          
        }

        internal bool IsEmpty() {
            return this.heapSize == 0;
        }

        private void PutAtI(int i, GenericHeapElement<T> h) {
            this.A[i] = h;
            h.indexToA = i;
        }

        internal T Dequeue() {
            if (this.heapSize == 0) {
                throw new InvalidOperationException();
            }

            var ret = this.A[1].v;

            this.MoveQueueOneStepForward(ret);

            return ret;

        }

        internal T Dequeue(out double priority) {
            if (this.heapSize == 0) {
                throw new InvalidOperationException();
            }

            var ret = this.A[1].v;
            priority = this.A[1].priority;
            this.MoveQueueOneStepForward(ret);
            return ret;
        }

        private void MoveQueueOneStepForward(T ret) {
            this.cache.Remove(ret);
            this.PutAtI(1, this.A[this.heapSize]);
            int i = 1;
            while (true) {
                int smallest = i;
                int l = i << 1;

                if (l <= this.heapSize && this.A[l].priority.CompareTo(this.A[i].priority) < 0) {
                    smallest = l;
                }

                int r = l + 1;

                if (r <= this.heapSize && this.A[r].priority.CompareTo(this.A[smallest].priority) < 0) {
                    smallest = r;
                }

                if (smallest != i) {
                    this.SwapWithParent(smallest);
                } else {
                    break;
                }

                i = smallest;

            }

            this.heapSize--;
        }

        /// <summary>
        /// sets the object priority to c
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal void DecreasePriority(T element, double newPriority)
        {
            GenericHeapElement<T> h;
            //ignore the element if it is not in the queue
            if (!this.cache.TryGetValue(element, out h)) {
                return;
            }

            //var h = cache[element];
            h.priority = newPriority;
            int i = h.indexToA;
            while (i > 1) {
                if (this.A[i].priority.CompareTo(this.A[i >> 1].priority) < 0) {
                    this.SwapWithParent(i);
                } else {
                    break;
                }

                i >>= 1;
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        
        /// <summary>
        /// enumerator
        /// </summary>
        /// <returns></returns>
        public IEnumerator<T> GetEnumerator() {
            for (int i = 1; i <= this.heapSize; i++) {
                yield return this.A[i].v;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="priority"></param>
        /// <returns></returns>
        public T Peek(out double priority) {
            if (this.Count == 0) {
                priority = 0.0;
                return default(T);
            }
            priority = this.A[1].priority;
            return this.A[1].v;         
        }

        IEnumerator IEnumerable.GetEnumerator() {
            for (int i = 1; i <= this.heapSize; i++) {
                yield return this.A[i].v;
            }
        }
#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            StringBuilder sb=new StringBuilder();
            foreach (var i in this) {
                sb.Append(i + ",");
            }

            return sb.ToString();
        }
  
#endif
    }
}



