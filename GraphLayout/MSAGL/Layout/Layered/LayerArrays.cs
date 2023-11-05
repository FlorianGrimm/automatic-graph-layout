using System;

namespace Microsoft.Msagl.Layout.Layered {
    /// <summary>
    /// Keeps all information about the hierarchy of layers
    /// </summary>
    internal class LayerArrays {

        internal int[] Y;
        private int[] verticesToX;
        private int[][] layers;

        internal LayerArrays(int[] verticesToLayers) {
            this.Initialize(verticesToLayers);
        }

        internal void Initialize(int[] verticesToLayers) {
            this.Y = verticesToLayers;
            this.verticesToX = null;
            this.layers = null;
        }

        /// <summary>
        /// Returns the same arrays but with no empty layers.
        /// </summary>
        /// <returns></returns>
        internal LayerArrays DropEmptyLayers() {
            int[] drop = new int[this.Layers.Length];
            int dropVal = 0;
            for (int i = 0; i < this.Layers.Length; i++) {
                drop[i] = dropVal;
                if (this.Layers[i].Length == 0) {
                    dropVal++;
                }
            }

            if (dropVal == 0) {
                return this;
            }

            //we do have empty layers
            int[] ny = new int[this.Y.Length];
            for (int i = 0; i < ny.Length; i++) {
                ny[i] = this.Y[i] - drop[this.Y[i]];
            }


            //copy the layers itself
            int[][] nls = new int[this.layers.Length - dropVal][];
            for (int i = 0; i < this.layers.Length; i++) {
                if (this.layers[i].Length > 0) {
                    nls[i - drop[i]] = (int[])this.layers[i].Clone();
                }
            }

            LayerArrays la = new LayerArrays(ny);
            la.layers = nls;

            return la;

        }

        internal void UpdateLayers(int[][] ulayers) {

            if (this.layers == null) {
                this.InitLayers();
            }

            for (int i = 0; i < this.layers.Length; i++) {
                ulayers[i].CopyTo(this.layers[i], 0);
            }

            this.UpdateXFromLayers();

        }

        internal void UpdateXFromLayers() {

            if (this.layers == null) {
                this.InitLayers();
            }

            if (this.verticesToX == null) {
                this.verticesToX = new int[this.Y.Length];
            }

            foreach (int[] layer in this.layers) {
                int i = 0;
                foreach (int v in layer) {
                    this.verticesToX[v] = i++;
                }
            }
        }


        /// <summary>
        /// gives the order of the vertices in the y-layer
        /// </summary>
        /// <value></value>
        internal int[] X {
            get {
                if (this.verticesToX != null) {
                    return this.verticesToX;
                }

                this.verticesToX = new int[this.Y.Length];

                this.UpdateXFromLayers();

                return this.verticesToX;
            }

        }



        /// <summary>
        /// returns the layer hierarchy where the order of the layers is reversed
        /// </summary>
        /// <returns></returns>
        internal LayerArrays ReversedClone() {
            int[] rv = new int[this.Y.Length];
            int lastLayer = this.Layers.Length - 1; //call Layers to ensure that the layers are calculated
            for (int i = 0; i < this.Y.Length; i++) {
                rv[i] = lastLayer - this.Y[i];
            }

            return new LayerArrays(rv);
        }



        /// <summary>
        /// Layers[i] is the array of vertices of i-th layer
        /// </summary>
        internal int[][] Layers {
            get {
                if (this.layers != null) {
                    return this.layers;
                }

                this.InitLayers();

                return this.layers;

            }

            set {
                this.layers = value;
            }


        }

        private void InitLayers() {
            //find the number of layers
            int nOfLayers = 0;

            foreach (int l in this.Y) {
                if (l + 1 > nOfLayers) {
                    nOfLayers = l + 1;
                }
            }

            int[] counts = new int[nOfLayers];


            //find the number of vertices in the layer
            foreach (int l in this.Y) {
                counts[l]++;
            }

            this.layers = new int[nOfLayers][];

            for (int i = 0; i < nOfLayers; i++) {
                this.layers[i] = new int[counts[i]];
                counts[i] = 0;//we reuse these counts later in the depth search
            }

            for (int i = 0; i < this.Y.Length; i++) {
                int l = this.Y[i];
                this.layers[l][counts[l]++] = i;
            }
        }

    }
}
