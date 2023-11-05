using System;

namespace Microsoft.Msagl.Core.GraphAlgorithms {
    /// <summary>
    /// Represents a couple of integers.
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
#if TEST_MSAGL
    public
#else
    internal
#endif
 sealed class IntPair : IEdge {

        internal int x;
        internal int y;

        /// <summary>
        /// the first element of the pair
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811")]
        public int First {
            get { return this.x; }
            set
            {
                this.x = value;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
                UpdateHashKey();
#endif
            }
        }

        /// <summary>
        /// the second element of the pair
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811")]
        public int Second {
            get { return this.y; }
            set
            {
                this.y = value;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
                UpdateHashKey();
#endif
            }
        }
        /// <summary>
        ///  the less operator
        /// </summary>
        /// <param name="pair0"></param>
        /// <param name="pair1"></param>
        /// <returns></returns>
        public static bool operator <(IntPair pair0, IntPair pair1) {
            if (pair0 != null && pair1 != null) {
                return pair0.x < pair1.x || pair0.x == pair1.x && pair0.y < pair1.y;
            }

            throw new InvalidOperationException();
        }
        public override bool Equals(object obj) {
            if (!(obj is IntPair)) {
                return false;
            }
            return (IntPair)obj == this;
        }
        /// <summary>
        /// the greater operator
        /// </summary>
        /// <param name="pair0"></param>
        /// <param name="pair1"></param>
        /// <returns></returns>
        public static bool operator >(IntPair pair0, IntPair pair1) {
            return pair1 < pair0;
        }
        public static bool operator==(IntPair pair0, IntPair pair1) {
            return pair1.x == pair0.x && pair1.y == pair0.y;
        }
        public static bool operator !=(IntPair pair0, IntPair pair1) {
            return pair1.x != pair0.x || pair1.y != pair0.y;
        }


        /// <summary>
        /// Compares two pairs
        /// </summary>
        /// <param name="pair0"></param>
        /// <param name="pair1"></param>
        /// <returns></returns>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811")]
        public int CompareTo(IntPair other) {
            var r = this.x.CompareTo(other.x);
            return r != 0 ? r : this.y.CompareTo(other.y);            
        }

#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
        private SharpKit.JavaScript.JsString _hashKey;
        private void UpdateHashKey()
        {
            _hashKey = ""+x+","+y;
        }
#endif

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override int GetHashCode() {
            uint hc = (uint)this.x.GetHashCode();
            return (int)((hc << 5 | hc >> 27) + (uint)this.y);
        }

        /// <summary>
        /// the constructor
        /// </summary>
        /// <param name="first"></param>
        /// <param name="second"></param>
        public IntPair(int first, int second) {
            this.x = first;
            this.y = second;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
            UpdateHashKey();
#endif
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return "(" + this.x + "," + this.y + ")";
        }

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203 Explicitly implemented interfaces are not generate without any warning
        public int Source {
#else
        int IEdge.Source {
#endif
            get { return this.x; }
            set
            {
                this.x = value;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
                UpdateHashKey();
#endif
            }
        }

#if SHARPKIT //http://code.google.com/p/sharpkit/issues/detail?id=203 Explicitly implemented interfaces are not generate without any warning
        public int Target {
#else
        int IEdge.Target {
#endif
            get { return this.y; }
            set
            {
                this.y = value;
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=289 Support Dictionary directly based on object's GetHashCode
                UpdateHashKey();
#endif
            }
        }
    }
}
