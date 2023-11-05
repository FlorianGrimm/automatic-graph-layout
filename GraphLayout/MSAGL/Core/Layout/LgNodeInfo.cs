using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Layout.LargeGraphLayout;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// facilitates large graph browsing
    /// </summary>
    public partial class LgNodeInfo : LgInfoBase {
        //these needed for shortest path calculations
        internal bool Processed;
        public int PartiteSet;
        /// <summary>
        /// underlying geometry node
        /// </summary>
        public Node GeometryNode { get; set; }

        public bool Selected;
        public int SelectedNeighbor = 0;

        public ICurve BoundaryCurve {
            get { return this.BoundaryOnLayer; }
        }

        internal LgNodeInfo(Node geometryNode) {
            this.GeometryNode = geometryNode;
            //OriginalCurveOfGeomNode = geometryNode.BoundaryCurve.Clone();
        }

        /// <summary>
        /// the center of the node
        /// </summary>
        public Point Center {
            get { return this.GeometryNode.Center; }
        }

        
        /// <summary>
        /// 
        /// </summary>
        public Rectangle BoundingBox {
            get {
                return this.BoundaryOnLayer.BoundingBox;
            }
        }

        /// <summary>
        /// override the string method
        /// </summary>
        /// <returns></returns>
        public override string ToString() {
            return this.GeometryNode.ToString();
        }

        public double LabelVisibleFromScale = 0.0;

        public double LabelWidthToHeightRatio = 1.0;

        public LabelPlacement LabelPosition = LabelPlacement.Top;
        public object Color;

        public static Point GetLabelOffset(LabelPlacement placement)
        {
            switch (placement) {
                case LabelPlacement.Top:
                    return new Point(0, 0.5);
                case LabelPlacement.Bottom:
                    return new Point(0, -0.5);
                case LabelPlacement.Left:
                    return new Point(-0.5, 0);
                default:
                    return new Point(0.5, 0);
            }
        }

        public Point LabelOffset
        {
            get
            {
                switch (this.LabelPosition)
                {
                    case LabelPlacement.Top:
                        return new Point(0, 0.5);
                    case LabelPlacement.Bottom:
                        return new Point(0, -0.5);
                    case LabelPlacement.Left:
                        return new Point(-0.5, 0);
                    default:
                        return new Point(0.5, 0);
                }
            }
            set
            {
                if (Math.Abs(value.X) < 0.1 && value.Y > 0.4)
                {
                    this.LabelPosition = LabelPlacement.Top;
                }
                else if (Math.Abs(value.X) < 0.1 && value.Y < -0.4) {
                    this.LabelPosition = LabelPlacement.Bottom;
                } 
                else if (Math.Abs(value.Y) < 0.1 && value.X > 0.4) {
                    this.LabelPosition = LabelPlacement.Right;
                } 
                else {
                    this.LabelPosition = LabelPlacement.Left;
                }
            }
        }
        /// <summary>
        /// on each layer the node boundary is a different polyline, smaller on the layers with large zool levels
        /// </summary>
        public Polyline BoundaryOnLayer { get; set; }

        public void Translate(Point delta) {
            this.GeometryNode.Center += delta;
            this.BoundaryOnLayer.Translate(delta);
        }
    }
}
