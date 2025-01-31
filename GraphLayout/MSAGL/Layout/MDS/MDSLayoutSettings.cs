using System.ComponentModel;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree;

namespace Microsoft.Msagl.Layout.MDS
{
    /// <summary>
    /// MDL layout settings
    /// </summary>
#if PROPERTY_GRID_SUPPORT
    [DisplayName("MDS layout settings")]
    [Description("Setting for Multi Dimensional Scaling algorithm"),
    TypeConverterAttribute(typeof(ExpandableObjectConverter))]
#endif
    public class MdsLayoutSettings : LayoutAlgorithmSettings {

        /// <summary>
        /// the setting of Multi-Dimensional Scaling layout
        /// </summary>
        public MdsLayoutSettings(){}
        // private double epsilon = Math.Pow(10,-8);
        private int pivotNumber = 50;
        private int iterationsWithMajorization = 30;
        private double scaleX = 200;
        private double scaleY = 200;
        private double exponent = -2;
        private double rotationAngle;
        private bool removeOverlaps = true;

        
        /// <summary>
        /// 
        /// </summary>
#if SHARPKIT // no multithreading in JS
        public bool RunInParallel = false;
#else
        public bool RunInParallel = true;
#endif
        private int _callIterationsWithMajorizationThreshold = 3000;

        /// remove overlaps between node boundaries
        public bool RemoveOverlaps {
            set { this.removeOverlaps = value; }
            get { return this.removeOverlaps; }
        }

        /// <summary>
        /// Number of pivots in Landmark Scaling (between 3 and number of objects).
        /// </summary>
#if PROPERTY_GRID_SUPPORT
        [DisplayName("Number of pivots")]
        [Description("Number of pivots in MDS")]
        [DefaultValue(50)]
#endif
        public int PivotNumber {
            set { this.pivotNumber = value; }
            get { return this.pivotNumber; }
        }

        /// <summary>
        /// Number of iterations in distance scaling
        /// </summary>


        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming",
            "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Majorization")]
#if PROPERTY_GRID_SUPPORT
        [DisplayName("Number of iterations with majorization")]
        [Description("If this number is positive then the majorization method will be used with the initial solution taken from the landmark method")]
        [DefaultValue(0)]
#endif
            public int IterationsWithMajorization {
            set { this.iterationsWithMajorization = value; }
            get { return this.iterationsWithMajorization; }
        }

        /// <summary>
        /// X Scaling Factor.
        /// </summary>
#if PROPERTY_GRID_SUPPORT
        [DisplayName("Scale by x-axis")]
        [Description("The resulting layout will be scaled in the x-axis by this number")]
        [DefaultValue(200.0)]
#endif
        public double ScaleX {
            set { this.scaleX = value; }
            get { return this.scaleX; }
        }

        /// <summary>
        /// Y Scaling Factor.
        /// </summary>
#if PROPERTY_GRID_SUPPORT
        [DisplayName("Scale by y-axis")]
        [Description("The resulting layout will be scaled by y-axis by this number")]
        [DefaultValue(200.0)]
#endif
        public double ScaleY {
            set { this.scaleY = value; }
            get { return this.scaleY; }
        }

        /// <summary>
        /// Weight matrix exponent.
        /// </summary>
#if PROPERTY_GRID_SUPPORT
        [Description("The power to raise the distances to in the majorization step")]
        [DefaultValue(-2.00)]
#endif
        public double Exponent {
            set { this.exponent = value; }
            get { return this.exponent; }
        }

        /// <summary>
        /// rotation angle
        /// </summary>
#if PROPERTY_GRID_SUPPORT
        [DisplayName("Rotation angle")]
        [Description("The resulting layout will be rotated by this angle")]
        [DefaultValue(0.0)]
#endif
        public double RotationAngle {
            set { this.rotationAngle = value%360; }
            get { return this.rotationAngle; }
        }

        /// <summary>
        /// Clones the object
        /// </summary>
        /// <returns></returns>
        public override LayoutAlgorithmSettings Clone() {
            return this.MemberwiseClone() as LayoutAlgorithmSettings;
        }

        
        /// <summary>
        /// Adjust the scale of the graph if there is not enough whitespace between nodes
        /// </summary>
        public bool AdjustScale { get; set; }


        
        public int GetNumberOfIterationsWithMajorization(int nodeCount) {
            if (nodeCount > this.CallIterationsWithMajorizationThreshold) {
                return 0;
            }
            return this.IterationsWithMajorization;
        }

        public int CallIterationsWithMajorizationThreshold {
            get { return this._callIterationsWithMajorizationThreshold; }
            set { this._callIterationsWithMajorizationThreshold = value; }
        }
    }
}
