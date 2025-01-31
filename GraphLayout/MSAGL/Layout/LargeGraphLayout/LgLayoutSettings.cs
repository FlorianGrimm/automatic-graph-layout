﻿using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;

namespace Microsoft.Msagl.Layout.LargeGraphLayout
{
    /// <summary>
    /// layout settings to handle a large graph
    /// </summary>
    public class LgLayoutSettings : LayoutAlgorithmSettings
    {

        public int maximumNumOfLayers;
        public double mainGeometryGraphWidth;
        public double mainGeometryGraphHeight;
        public GeometryGraph lgGeometryGraph;
        public bool hugeGraph;
        public bool flow;
        public int delta;


        internal readonly Func<PlaneTransformation> TransformFromGraphToScreen;
        internal readonly double DpiX;
        internal readonly double DpiY;

        /// <summary>
        /// size of drawn node
        /// </summary>
        //public double NodeDotWidthInInches = 0.1;
        public double NodeDotWidthInInches = 0.08; //jyoti made it small
        public double NodeDotWidthInInchesMinInImage = 0.06;
        //public double NodeLabelHeightInInches = 0.22;
        public double NodeLabelHeightInInches = 0.14; //jyoti made it small

        /// <summary>
        /// Func giving the maximal arrowhead length for the current viewport
        /// </summary>
        public readonly Func<double> MaximalArrowheadLength;



        /// <summary>
        /// delegate to be called by LgInteractor on a graph change
        /// </summary>
        public delegate void LgLayoutEvent();

        /// <summary>
        /// event handler to be raised when the graph and its transform change
        /// </summary>
        public event LgLayoutEvent ViewerChangeTransformAndInvalidateGraph;

        /// <summary>
        /// 
        /// </summary>
        public void OnViewerChangeTransformAndInvalidateGraph()
        {
            if (ViewerChangeTransformAndInvalidateGraph != null) {
                ViewerChangeTransformAndInvalidateGraph();
            }
        }

        /// <summary>
        /// this graph is shown in the viewer
        /// </summary>
        public RailGraph RailGraph
        {
            get { return this.Interactor != null ? this.Interactor.RailGraph : null; }
        }

        /// <summary>
        /// clones the object
        /// </summary>
        /// <returns></returns>
        public override LayoutAlgorithmSettings Clone()
        {
            return (LayoutAlgorithmSettings)this.MemberwiseClone();
        }

        /// <summary>
        /// the algorithm object
        /// </summary>
        public LgInteractor Interactor { get; set; }


        /// <summary>
        /// a delegate returning the current viewport of the viewer
        /// </summary>
        internal Func<Rectangle> ClientViewportFunc { get; set; }

        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="clientViewportFunc"></param>
        /// <param name="transformFromGraphToScreen"> </param>
        /// <param name="dpiX"> </param>
        /// <param name="dpiY"> </param>
        /// <param name="maximalArrowheadLength">the dynamic length of the arrowhead</param>
        public LgLayoutSettings(Func<Rectangle> clientViewportFunc,
                                Func<PlaneTransformation> transformFromGraphToScreen,
                                double dpiX, double dpiY,
                                Func<double> maximalArrowheadLength
            )
        {
            this.ClientViewportFunc = clientViewportFunc;
            this.TransformFromGraphToScreen = transformFromGraphToScreen;
            this.DpiX = dpiX;
            this.DpiY = dpiY;
            this.MaximalArrowheadLength = maximalArrowheadLength;
            this.EdgeRoutingSettings.Padding = this.NodeSeparation / 4;
            this.EdgeRoutingSettings.PolylinePadding = this.NodeSeparation / 6;
            this.InitDefaultRailColors();
            this.InitDefaultSelectionColors();
        }

        public Func<Rectangle> ClientViewportMappedToGraph { get; set; }

        private Dictionary<Node, LgNodeInfo> geometryNodesToLgNodeInfos = new Dictionary<Node, LgNodeInfo>();

        /// <summary>
        /// the mapping from Geometry nodes to LgNodes
        /// </summary>
        public Dictionary<Node, LgNodeInfo> GeometryNodesToLgNodeInfos
        {
            get { return this.geometryNodesToLgNodeInfos; }
            set { this.geometryNodesToLgNodeInfos = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        public IDictionary<Edge, LgEdgeInfo> GeometryEdgesToLgEdgeInfos
        {
            get { return this.Interactor.GeometryEdgesToLgEdgeInfos; }
        }



        /// <summary>
        /// 
        /// </summary>
        private Interval _scaleInterval = new Interval(0.00001, 100000000.0);
        private bool needToLayout = true;
        private int maxNumberOfNodesPerTile = 20;


        /// <summary>
        /// used for debugging mostly
        /// </summary>
        public bool ExitAfterInit;

        /// <summary>
        /// the range of scale 
        /// </summary>
        public Interval ScaleInterval
        {
            get { return this._scaleInterval; }
            set { this._scaleInterval = value; }
        }
        /// <summary>
        /// set this property to true if the graph comes with a given layout
        /// </summary>
        public bool NeedToLayout
        {
            get { return this.needToLayout; }
            set { this.needToLayout = value; }
        }

        /// <summary>
        /// the node quota per tile
        /// </summary>
        public int MaxNumberOfNodesPerTile
        {
            get { return this.maxNumberOfNodesPerTile; }
            set { this.maxNumberOfNodesPerTile = value; }
        }

        private double increaseNodeQuota;

        public double IncreaseNodeQuota
        {
            get { return this.increaseNodeQuota; }
            set { this.increaseNodeQuota = value; }
        }

        private int maxNumberOfRailsPerTile = 300;

        public int MaxNumberOfRailsPerTile
        {
            get { return this.maxNumberOfRailsPerTile; }
            set { this.maxNumberOfRailsPerTile = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public double GetMaximalZoomLevel()
        {
            return this.Interactor.GetMaximalZoomLevel();
        }

        private bool _simplifyRoutes = true;
        private int _numberOfNodeShapeSegs = 12; //16;

        public bool SimplifyRoutes
        {
            get { return this._simplifyRoutes; }
            set { this._simplifyRoutes = value; }
        }

        public int NumberOfNodeShapeSegs
        {
            get { return this._numberOfNodeShapeSegs; }
            set { this._numberOfNodeShapeSegs = value; }
        }

        public bool GenerateTiles
        {
            get { return this._generateTiles; }
            set { this._generateTiles = value; }
        }

        private bool _generateTiles = true;

        private String[] _railColors;

        public String[] RailColors
        {
            get { return this._railColors; }
            set { this._railColors = value; }
        }

        private String[] _selectionColors;

        public string[] SelectionColors
        {
            get { return this._selectionColors; }
            set { this._selectionColors = value; }
        }

        private void InitDefaultRailColors()
        {
            this._railColors = new String[3];

            this._railColors[0] = "#87CEFA";
            this._railColors[1] = "#FAFAD2";
            this._railColors[2] = "#FAFAD2";

            //jyoti changed colors
            //_railColors[0] = "#87CEFA";
            //_railColors[1] = "#FAFAD2";
            //_railColors[2] = "#F5F5F5";
        }

        private void InitDefaultSelectionColors()
        {
            // init red selection
            this._selectionColors = new String[3];


            this._selectionColors[0] = "#E60000";
            this._selectionColors[1] = "#E60000";
            this._selectionColors[2] = "#E60000";

            //jyoti changed colors
            //_selectionColors[0] = "#FF0000";
            //_selectionColors[1] = "#EB3044";
            //_selectionColors[2] = "#E55C7F";

            //orange: #FF6A00,#FF9047,FFB07F 
        }

        public String GetColorForZoomLevel(double zoomLevel)
        {
            int logZoomLevel = (int)Math.Log(zoomLevel, 2.0);
            logZoomLevel = Math.Min(logZoomLevel, this.RailColors.Count() - 1);
            logZoomLevel = Math.Max(logZoomLevel, 0);
            return this.RailColors[logZoomLevel];
        }

        public String GetSelColorForZoomLevel(double zoomLevel)
        {
            int logZoomLevel = (int)Math.Log(zoomLevel, 2.0);
            logZoomLevel = Math.Min(logZoomLevel, this.SelectionColors.Count() - 1);
            logZoomLevel = Math.Max(logZoomLevel, 0);
            return this.SelectionColors[logZoomLevel];
        }

        public String GetNodeSelColor()
        {
            return this.SelectionColors[0];
        }

    }
}