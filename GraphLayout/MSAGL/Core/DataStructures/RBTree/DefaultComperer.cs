using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.DataStructures {
    internal class DefaultComperer<T> : IComparer<T> {
        public int Compare(T? x, T? y) {
            if (x == null && y == null) return 0;
            if (x == null) return -1;
            if (y == null) return 1;
            return ((IComparable<T>)x).CompareTo(y);
        }
    }
}
