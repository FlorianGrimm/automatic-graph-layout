using System;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.DataStructures {
    internal class ComparerOnDelegate<T> : IComparer<T> {
        private readonly Func<T?, T?, int> comparer;

        public ComparerOnDelegate(Func<T?, T?, int> compare) {
            this.comparer = compare;
        }
     
        public int Compare(T? x, T? y) {
            return this.comparer(x, y);
        }
    }
}