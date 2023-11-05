using System;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.Incremental {
    /// <summary>
    /// Wrapper for the MSAGL node to add force and velocity vectors
    /// </summary>
    internal class FiNode {
        internal Point desiredPosition;
        internal Point force;
        internal int index;
        internal Node mNode;
        internal OverlapRemovalNode mOlapNodeX, mOlapNodeY;
        internal Point previousCenter;
        private Point center;
        /// <summary>
        /// local cache of node center (which in the MSAGL node has to be computed from the bounding box)
        /// </summary>
        internal Point Center {
            get {
                return this.center;
            }
            set {
                this.center = this.mNode.Center = value;
            }
        }
        /// <summary>
        /// When mNode's bounds change we need to update our local
        /// previous and current center to MSAGL node center
        /// and update width and height
        /// </summary>
        internal void ResetBounds() {
            this.center = this.previousCenter = this.mNode.Center;
            this.Width = this.mNode.Width;
            this.Height = this.mNode.Height;
        }
        internal double stayWeight = 1;

        /// <summary>
        /// We also keep a local copy of Width and Height since it doesn't change and we don't want to keep going back to
        /// mNode.BoundingBox
        /// </summary>
        internal double Width;
        internal double Height;

        public FiNode(int index, Node mNode) {
            this.index = index;
            this.mNode = mNode;
            this.ResetBounds();
        }

        internal OverlapRemovalNode getOlapNode(bool horizontal) {
            return horizontal ? this.mOlapNodeX : this.mOlapNodeY;
        }

        internal void SetOlapNode(bool horizontal, OverlapRemovalNode olapNode) {
            if (horizontal) {
                this.mOlapNodeX = olapNode;
            } else {
                this.mOlapNodeY = olapNode;
            }
        }

        internal void SetVariableDesiredPos(bool horizontal) {
            if (horizontal) {
                this.mOlapNodeX.Variable.DesiredPos = this.desiredPosition.X;
            }
            else {
                this.mOlapNodeY.Variable.DesiredPos = this.desiredPosition.Y;
            }
        }
        /// <summary>
        /// Update the current X or Y coordinate of the node center from the result of a solve
        /// </summary>
        /// <param name="horizontal"></param>
        internal void UpdatePos(bool horizontal) {
            if (horizontal) {
                // Y has not yet been solved so reuse the previous position.
                this.Center = new Point(this.getOlapNode(true).Position, this.previousCenter.Y);
            } else {
                // Assumes X has been solved and set on prior pass.
                this.Center = new Point(this.Center.X, this.getOlapNode(false).Position);
            }
        }

        public override string ToString()
        {
            return "FINode(" + this.index + "):" + this.mNode;
        }
    }
}