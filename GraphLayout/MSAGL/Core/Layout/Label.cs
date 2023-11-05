using System;

using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using System.Collections.Generic;

namespace Microsoft.Msagl.Core.Layout {
    /// <summary>
    /// A class keeping the data about an edge label
    /// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
    public class Label : GeometryObject {

        /// <summary>
        /// The center of the label bounding box
        /// </summary>
        public Point Center {
            get { return this.boundingBox.Center; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.boundingBox.Center = value;
            }
        }

        /// <summary>
        /// Width of the label: set by the user.
        /// Label width could be different from the original width if the layer direction 
        /// of the layout is horizontal. This change is used only during the calcualations.
        /// </summary>
        public double Width {
            get { return this.boundingBox.Width; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.boundingBox.Width = value;
            }
        }

        /// <summary>
        /// Height of the label: set by the user
        /// Label height could be different from the original height if the layer direction 
        /// of the layout is horizontal. This change is used only during the calcualations 
        /// </summary>
        public double Height {
            get { return this.boundingBox.Height; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.boundingBox.Height = value;
            }
        }

        /// <summary>
        /// Constructor
        /// </summary>
        ///<param name="labelWidth">width</param>
        ///<param name="labelHeight">height</param>
        ///<param name="parentP">the corresponding edge</param>

        public Label(double labelWidth, double labelHeight, GeometryObject parentP) {
            this.Width = labelWidth;
            this.Height = labelHeight;
            this.GeometryParent = parentP;
            this.PlacementStrategyPriority = new[] {PlacementStrategy.AlongCurve, PlacementStrategy.Horizontal};
        }

        /// <summary>
        /// an empty constructor
        /// </summary>
        public Label() {
        }

        private Rectangle boundingBox;

        /// <summary>
        /// gets or sets the boundary box of the label
        /// </summary>
        public override Rectangle BoundingBox {
            get { return this.boundingBox; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.boundingBox = value;
            }
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal Label(GeometryObject parentPar) {
            this.GeometryParent = parentPar;
        }

        private Point attachmentSegmentStart;

        /// <summary>
        /// the start of the segment showing the connection between the label and the edge
        /// </summary>
        public Point AttachmentSegmentStart {
            get { return this.attachmentSegmentStart; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.attachmentSegmentStart = value;
            }
        }

        private Point edgeAttachmentPoint;

        /// <summary>
        /// the point on the edge closest to the label center
        /// </summary>
        public Point AttachmentSegmentEnd {
            get { return this.edgeAttachmentPoint; }
            set {
                this.RaiseLayoutChangeEvent(value);
                this.edgeAttachmentPoint = value;
            }
        }

        /// <summary>
        /// 0 is the start of the edge, 0.5 middle, 1 the end
        /// </summary>
        public double PlacementOffset {
            get { return this.placementOffset; }
            set { this.placementOffset = value; }
        }

        /// <summary>
        /// Options for which side of the edge the label should be placed on.
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1034:NestedTypesShouldNotBeVisible")]
        public enum PlacementSide {
            /// <summary>
            /// Places the label on any side
            /// </summary>
            Any,

            /// <summary>
            /// Places the label on the port side of the edge.
            /// Port is the left side of the edge if you were facing away from the source and towards the target.
            /// </summary>
            Port,

            /// <summary>
            /// Places the label on the starboard side of the edge.
            /// Starboard is the right side of the edge if you were facing away from the source and towards the target.
            /// </summary>
            Starboard,

            /// <summary>
            /// Places the label on the top side of the line.
            /// If the line is vertical, the label is placed on the left.
            /// </summary>
            Top,

            /// <summary>
            /// Places the label on the bottom side of the line.
            /// If the line is vertical, the label is placed on the right.
            /// </summary>
            Bottom,

            /// <summary>
            /// Places the label on the left side of the line.
            /// If the line is horizontal, the label is placed on the top.
            /// </summary>
            Left,

            /// <summary>
            /// Places the label on the right side of the line.
            /// If the line is horizontal, the label is placed on the bottom.
            /// </summary>
            Right,
        }

        /// <summary>
        /// which side of the edge to place the label
        /// </summary>
        public PlacementSide Side { get; set; }

        /// <summary>
        /// The various strategies we have for placing labels along edges
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1034:NestedTypesShouldNotBeVisible")]
        public enum PlacementStrategy {
            /// <summary>
            /// Try to place the label running along the curve path
            /// </summary>
            AlongCurve,

            /// <summary>
            /// Standard horizontal label
            /// </summary>
            Horizontal
        }

        private IEnumerable<PlacementStrategy> placementStrategyPriority = new[] {
                                                                             PlacementStrategy.Horizontal,
                                                                             PlacementStrategy.AlongCurve
                                                                         };
        private double placementOffset = 0.5;

        ///<summary>
        /// an array of placement strategies
        ///</summary>
        public IEnumerable<PlacementStrategy> PlacementStrategyPriority {
            get { return this.placementStrategyPriority; }
            set { this.placementStrategyPriority = value; }
        }

        /// <summary>
        /// Inner points of the label
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage",
            "CA2227:CollectionPropertiesShouldBeReadOnly")]
        public IList<Point> InnerPoints { get; set; }

        /// <summary>
        /// Outer points of the label
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage",
            "CA2227:CollectionPropertiesShouldBeReadOnly")]
        public IList<Point> OuterPoints { get; set; }

        /// <summary>
        /// The label placement algorithm sets this true if a "good" placement was found
        /// </summary>
        public LabelPlacementResult PlacementResult { get; set; }

        /// <summary>
        /// Translate the labels position state by the given delta
        /// </summary>
        /// <param name="delta"></param>
        public void Translate(Point delta) {
            this.RaiseLayoutChangeEvent(delta);
            if (this.InnerPoints != null) {
                for (int i = 0; i < this.InnerPoints.Count; ++i) {
                    this.InnerPoints[i] = this.InnerPoints[i] + delta;
                }
            }
            if (this.OuterPoints != null) {
                for (int i = 0; i < this.OuterPoints.Count; ++i) {
                    this.OuterPoints[i] = this.OuterPoints[i] + delta;
                }
            }
            this.boundingBox = Rectangle.Translate(this.boundingBox, delta);
            
        }
    }
}
