﻿using System.Collections.Generic;
using Microsoft.Msagl.Core;

namespace Microsoft.Msagl.Layout.Layered {
    internal class EdgeComparerByTarget : IComparer<LayerEdge> {
        private int[] X;
        internal EdgeComparerByTarget(int[] X) {
            this.X = X;
        }

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203
        //SharpKit/Colin - https://code.google.com/p/sharpkit/issues/detail?id=290
        public int Compare(LayerEdge a, LayerEdge b) {
#else
        int System.Collections.Generic.IComparer<LayerEdge>.Compare(LayerEdge a, LayerEdge b) {
#endif
            ValidateArg.IsNotNull(a, "a");
            ValidateArg.IsNotNull(b, "b");
            int r = this.X[a.Target] - this.X[b.Target];
            if (r != 0) {
                return r;
            }

            return this.X[a.Source] - this.X[b.Source];
        }
    }

}
