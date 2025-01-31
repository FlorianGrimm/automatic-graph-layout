#region Using directives

using System;
using System.ComponentModel;

#endregion

namespace Microsoft.Msagl.Core.Routing {
    ///<summary>
    ///</summary>

    [Description("Specifies the edge bundling settings")]
    [TypeConverter(typeof(ExpandableObjectConverter))]
    [DisplayName("Edge bundling settings")]
    public sealed class BundlingSettings {

        ///<summary>
        ///the default value of CapacityOverflowCoefficient
        ///</summary>
        static public double DefaultCapacityOverflowCoefficientMultiplier = 1000;
        private double capacityOverflowCoefficient = DefaultCapacityOverflowCoefficientMultiplier;
        ///<summary>
        ///this number is muliplied by the overflow penalty cost and by the sum of the LengthImportanceCoefficient 
        ///and InkImportanceCoefficient, and added to the routing price
        ///</summary>
        public double CapacityOverflowCoefficient {
            get { return this.capacityOverflowCoefficient; }
            set { this.capacityOverflowCoefficient = value; }
        }

        /// <summary>
        /// the upper bound of the virtual node radius
        /// </summary>
        internal double MaxHubRadius = 50.0;
        /// <summary>
        /// the lower bound of the virtual node radius
        /// </summary>
        internal double MinHubRadius = 0.1;

        /// <summary>
        /// 
        /// </summary>
        public bool CreateUnderlyingPolyline { get; set; }

        ///<summary>
        ///the default path lenght importance coefficient
        ///</summary>
        static public double DefaultPathLengthImportance = 500;
        private double pathLengthImportance = DefaultPathLengthImportance;

        /// <summary>
        /// the importance of path lengths coefficient
        /// </summary>
        public double PathLengthImportance {
            get { return this.pathLengthImportance; }
            set { this.pathLengthImportance = value; }
        }

        ///<summary>
        ///the default ink importance
        ///</summary>
        static public double DefaultInkImportance = 0.01;
        private double inkImportance = DefaultInkImportance;

        ///<summary>
        ///</summary>
        public double InkImportance {
            get { return this.inkImportance; }
            set { this.inkImportance = value; }
        }

        private double edgeSeparation = DefaultEdgeSeparation;

        ///<summary>
        ///default edge separation
        ///</summary>
        static public double DefaultEdgeSeparation = 0.5;

        /// <summary>
        /// Separation between to neighboring edges within a bundle
        /// </summary>
        public double EdgeSeparation {
            get { return this.edgeSeparation; }
            set { this.edgeSeparation = value; }
        }

        private bool useCubicBezierSegmentsInsideOfHubs;

        ///<summary>
        ///if is set to true will be using Cubic Bezie Segments inside of hubs, otherwise will be using Biarcs
        ///</summary>
        public bool UseCubicBezierSegmentsInsideOfHubs {
            get { return this.useCubicBezierSegmentsInsideOfHubs; }
            set { this.useCubicBezierSegmentsInsideOfHubs = value; }
        }

        private bool useGreedyMetrolineOrdering = true;

        ///<summary>
        ///if is set to true will be using greedy ordering algorithm, otherwise will be using linear
        ///</summary>
        public bool UseGreedyMetrolineOrdering {
            get { return this.useGreedyMetrolineOrdering; }
            set { this.useGreedyMetrolineOrdering = value; }
        }

        private double angleThreshold = Math.PI / 180 * 45; //45 degrees;
        ///<summary>
        ///min angle for gluing edges
        ///</summary>
        public double AngleThreshold {
            get { return this.angleThreshold; }
            set { this.angleThreshold = value; }
        }

        private double hubRepulsionImportance = 100;

        /// <summary>
        /// the importance of hub repulsion coefficient
        /// </summary>
        public double HubRepulsionImportance {
            get { return this.hubRepulsionImportance; }
            set { this.hubRepulsionImportance = value; }
        }

        private double bundleRepulsionImportance = 100;

        /// <summary>
        /// the importance of bundle repulsion coefficient
        /// </summary>
        public double BundleRepulsionImportance {
            get { return this.bundleRepulsionImportance; }
            set { this.bundleRepulsionImportance = value; }
        }

        private double minimalRatioOfGoodCdtEdges = 0.9;

        /// <summary>
        /// minimal ration of cdt edges with satisfied capacity needed to perform bundling
        /// (otherwise bundling will not be executed)
        /// </summary>
        public double MinimalRatioOfGoodCdtEdges {
            get { return this.minimalRatioOfGoodCdtEdges; }
            set { this.minimalRatioOfGoodCdtEdges = value; }
        }

        private bool highestQuality = true;

        /// <summary>
        /// speed vs quality of the drawing
        /// </summary>
        public bool HighestQuality {
            get { return this.highestQuality; }
            set { this.highestQuality = value; }
        }
        /// <summary>
        /// if is set to true the original spline before the trimming should be kept under the corresponding EdgeGeometry
        /// </summary>
        public bool KeepOriginalSpline { get; set; }
        /// <summary>
        /// if set to true then the edges will be routed one on top of each other with no gap inside of a bundle
        /// </summary>
        public bool KeepOverlaps { get; set; }
        /// <summary>
        /// calculates the routes that just follow the visibility graph
        /// </summary>
        public bool StopAfterShortestPaths { get; set; }
        /// <summary>
        /// rotate bundles to diminish the cost: seems counter productive!
        /// </summary>
        public bool RotateBundles { get; internal set; }
    }
}
