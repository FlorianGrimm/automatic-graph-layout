using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;

namespace Microsoft.Msagl.Core.DataStructures {
    /// <summary>
    /// A priority queue based on the binary heap algorithm.
    /// This class needs a comparer object to compare elements of the queue.
    /// </summary>
    /// <typeparam name="T"></typeparam>
    internal class BinaryHeapWithComparer<T> 
        where T:notnull
        {
        private const int _InitialHeapCapacity = 16;
        private T[] _Array; //array of the heap elems starting at A[1]

        private int _HeapSize = 0;

        internal void Enqueue(T element) {
            if (this._HeapSize == this._Array.Length - 1) {
                var newA = new T[this._Array.Length*2];
                Array.Copy(this._Array, 1, newA, 1, this._HeapSize);
                this._Array = newA;
            }

            int i = this._HeapSize + 1;
            this._Array[i] = element;
            this._HeapSize++;
            int j = i >> 1;
            T parent, son;
            while (i > 1 && this.Less(son = this._Array[i], parent = this._Array[j])) {
                this._Array[j] = son;
                this._Array[i] = parent;
                i = j;
                j = i >> 1;
            }
        }


        internal T Dequeue() {
            if (this._HeapSize < 1) {
                throw new InvalidOperationException();
            }

            T ret = this._Array[1];

            T candidate = this._Array[this._HeapSize];

            this._HeapSize--;

            this.ChangeMinimum(candidate);

            return ret;
        }

        internal void ChangeMinimum(T candidate) {
            this._Array[1] = candidate;

            int j = 1;
            int i = 2;
            bool done = false;
            while (i < this._HeapSize && !done) {
                done = true;
                //both sons exist
                T leftSon = this._Array[i];
                T rigthSon = this._Array[i + 1];
                int compareResult = this.comparer.Compare(leftSon, rigthSon);
                if (compareResult < 0) {
                    //left son is the smallest
                    if (this.comparer.Compare(leftSon, candidate) < 0) {
                        this._Array[j] = leftSon;
                        this._Array[i] = candidate;
                        done = false;
                        j = i;
                        i = j << 1;
                    }
                } else {
                    //right son in not the greatest
                    if (this.comparer.Compare(rigthSon, candidate) < 0) {
                        this._Array[j] = rigthSon;
                        this._Array[i + 1] = candidate;
                        done = false;
                        j = i + 1;
                        i = j << 1;
                    }
                }
            }

            if (i == this._HeapSize) {
                //can we do one more step:
                T leftSon = this._Array[i];
                if (this.comparer.Compare(leftSon, candidate) < 0) {
                    this._Array[j] = leftSon;
                    this._Array[i] = candidate;
                }
            }
        }

        internal int Count {
            get { return this._HeapSize; }
        }

        internal bool Less(T a, T b) {
            return this.comparer.Compare(a, b) < 0;
        }

        private IComparer<T> comparer;

        internal BinaryHeapWithComparer(IComparer<T> comparer) {
            this._Array = new T[_InitialHeapCapacity + 1];
            this.comparer = comparer;
        }

#if TEST_MSAGL

        public override string ToString() {
            int i = 1;
            return "{" + this.Print(i) + "}";
        }

        private string Print(int i) {
            if (2*i + 1 <= this._HeapSize) {
                return String.Format(CultureInfo.InvariantCulture, "({0}->{1},{2})", this._Array[i], this._Array[i*2], this._Array[i*2 + 1]) +
                       this.Print(i*2) + this.Print(i*2 + 1);
            }

            if (2*i == this._HeapSize) {
                return String.Format(CultureInfo.InvariantCulture, "({0}->{1}", this._Array[i], this._Array[i*2]);
            }

            if (i == this._HeapSize && i == 1) {
                return " " + this._Array[i].ToString() + " ";
            }

            return "";
        }
#endif


        public T GetMinimum() {
            return this._Array[1];
        }

#if TEST_MSAGL
        //internal void UpdateMinimum() {
        //    throw new NotImplementedException();
        //}
#endif
    }
}