using System;

namespace Microsoft.Msagl.Core.DataStructures {


    /// <summary>
    /// A priority queue based on the binary heap algorithm
    /// </summary>
    internal class BinaryHeapPriorityQueue {
        //indexing for A starts from 1

        private readonly int[] _Heap; //array of heap elements
        private readonly int[] _ReverseHeap; // the map from [0,..., n-1] to their places in heap

        /// <summary>
        /// the array of priorities
        /// </summary>
        private readonly double[] _Priorities;

        private int _HeapSize;
        internal int Count { get { return this._HeapSize; } }

        /// <summary>
        /// the constructor
        /// we suppose that all integers inserted into the queue will be less then n
        /// </summary>
        /// <param name="n">it is the number of different integers that will be inserted into the queue </param>
        internal BinaryHeapPriorityQueue(int n) {
            this._Priorities = new double[n];
            this._Heap = new int[n + 1];//because indexing for A starts from 1
            this._ReverseHeap = new int[n];
        }

        private void SwapWithParent(int i) {
            int parent = this._Heap[i >> 1];
            this.PutAtI(i >> 1, this._Heap[i]);
            this.PutAtI(i, parent);
        }

        internal void Enqueue(int o, double priority) {
            this._HeapSize++;
            int i = this._HeapSize;
            this._Priorities[o] = priority;
            this.PutAtI(i, o);
            while (i > 1 && this._Priorities[this._Heap[i >> 1]] > priority) {
                this.SwapWithParent(i);
                i >>= 1;
            }
        }

        private void PutAtI(int i, int h) {
            this._Heap[i] = h;
            this._ReverseHeap[h] = i;
        }

        /// <summary>
        /// return the first element of the queue and removes it from the queue
        /// </summary>
        /// <returns></returns>
        internal int Dequeue() {
            if (this._HeapSize == 0) {
                throw new InvalidOperationException();
            }

            int ret = this._Heap[1];
            if (this._HeapSize > 1)
            {
                this.PutAtI(1, this._Heap[this._HeapSize]);
                int i = 1;
                while (true)
                {
                    int smallest = i;
                    int l = i << 1;

                    if (l <= this._HeapSize && this._Priorities[this._Heap[l]] < this._Priorities[this._Heap[i]]) {
                        smallest = l;
                    }

                    int r = l + 1;

                    if (r <= this._HeapSize && this._Priorities[this._Heap[r]] < this._Priorities[this._Heap[smallest]]) {
                        smallest = r;
                    }

                    if (smallest != i) {
                        this.SwapWithParent(smallest);
                    } else {
                        break;
                    }

                    i = smallest;

                }
            }
            this._HeapSize--;
            return ret;
        }

        /// <summary>
        /// sets the object priority to c
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal void DecreasePriority(int o, double newPriority) {

            //System.Diagnostics.Debug.WriteLine("delcrease "+ o.ToString()+" to "+ newPriority.ToString());

            this._Priorities[o] = newPriority;
            int i = this._ReverseHeap[o];
            while (i > 1) {
                if (this._Priorities[this._Heap[i]] < this._Priorities[this._Heap[i >> 1]]) {
                    this.SwapWithParent(i);
                } else {
                    break;
                }

                i >>= 1;
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        public static void Test() {
            var q = new BinaryHeapPriorityQueue(10);
            q.Enqueue(2, 2);
            q.Enqueue(1, 1);
            q.Enqueue(9, 9);
            q.Enqueue(8, 8);
            q.Enqueue(5, 5);
            q.Enqueue(3, 3);
            q.Enqueue(4, 4);
            q.Enqueue(7, 7);
            q.Enqueue(6, 6);
            q.Enqueue(0, 0);
            for (int i = 0; i < 10; i++) {
                System.Diagnostics.Debug.WriteLine(q.Dequeue());
            }
        }
    }
}


