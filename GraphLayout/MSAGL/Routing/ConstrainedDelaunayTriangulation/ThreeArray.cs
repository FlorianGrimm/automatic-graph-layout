using System;
using System.Collections;
using System.Collections.Generic;

namespace Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation {
    ///<summary>
    /// an efficient class to simulate a three element array
    ///</summary>
    ///<typeparam name="T"></typeparam>
#if TEST_MSAGL
    [Serializable]
#endif

    public class ThreeArray<T>:IEnumerable<T> {
        private T item0;
        private T item1;
        private T item2;

        internal bool Contains(T t) {
            return t.Equals(this.item0) || t.Equals(this.item1) || t.Equals(this.item2);
        }

        internal int Index(T t) {
            if(t.Equals(this.item0)) {
                return 0;
            }

            if (t.Equals(this.item1)) {
                return 1;
            }

            if (t.Equals(this.item2)) {
                return 2;
            }

            return -1;
        }

        ///<summary>
        ///</summary>
        ///<param name="item0"></param>
        ///<param name="item1"></param>
        ///<param name="item2"></param>
        public ThreeArray(T item0, T item1, T item2) {
            this.item0 = item0;
            this.item1 = item1;
            this.item2 = item2;
        }

        ///<summary>
        ///</summary>
        public ThreeArray() {}

        ///<summary>
        ///</summary>
        ///<param name="i"></param>
        ///<exception cref="InvalidOperationException"></exception>
        public T this[int i] {
            get {
                switch (i) {
                    case 0:
                    case 3:
                    case -3: return this.item0;
                    case 1:
                    case 4:
                    case -2: return this.item1;
                    case 2:
                    case 5: 
                    case -1: return this.item2;
                    default: throw new InvalidOperationException();
                }
            }
            set {
                switch (i) {
                    case 0:
                    case 3:
                    case -3: this.item0 = value;break;
                    case 1:
                    case 4:
                    case -2: this.item1 = value;break;
                    case 2:
                    case 5:
                    case -1: this.item2 = value;break;

                    default:
                        throw new InvalidOperationException();
                }
            }
        }

        /// <summary>
        /// Returns an enumerator that iterates through the collection.
        /// </summary>
        /// <returns>
        /// A <see cref="T:System.Collections.Generic.IEnumerator`1"/> that can be used to iterate through the collection.
        /// </returns>
        /// <filterpriority>1</filterpriority>
        public IEnumerator<T> GetEnumerator() {
            yield return this.item0;
            yield return this.item1;
            yield return this.item2;
        }

        IEnumerator IEnumerable.GetEnumerator() {
            yield return this.item0;
            yield return this.item1;
            yield return this.item2;
        }
    }
}