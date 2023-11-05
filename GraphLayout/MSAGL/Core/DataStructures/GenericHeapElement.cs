namespace Microsoft.Msagl.Core.DataStructures {
    internal class GenericHeapElement<T> {
        internal int indexToA;
        internal double priority;
        internal T v;//value
        internal GenericHeapElement(int index, double priority, T v) {
            this.indexToA = index;
            this.priority = priority;
            this.v = v;
        }
    }
}