using Microsoft.Msagl.Layout.LargeGraphLayout;
using Microsoft.Msagl.Prototype.Ranking;
#if TEST_MSAGL
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Threading;
using System.Xml;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Layout.Incremental;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Layout.MDS;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;

namespace Microsoft.Msagl.DebugHelpers.Persistence
{

    /// <summary>
    /// reads the GeometryGraph from a file
    /// </summary>
    public class GeometryGraphReader : IDisposable
    {
        /// <summary>
        /// the list of edges, needed to match it with GraphReader edges
        /// </summary>
        public IList<Edge> EdgeList = new List<Edge>();
        private readonly Dictionary<string, Edge> idToEdges = new Dictionary<string, Edge>();
        private readonly Dictionary<string, Rail> idToRails = new Dictionary<string, Rail>();
        private readonly Dictionary<string, LgEdgeInfo> railIdsToTopRankedEdgeInfo = new Dictionary<string, LgEdgeInfo>();
        private readonly GeometryGraph _graph = new GeometryGraph();

        /// <summary>
        /// The deserialized settings.
        /// </summary>
        public LayoutAlgorithmSettings Settings { get; set; }

        private readonly Dictionary<string, ClusterWithChildLists> stringToClusters =
            new Dictionary<string, ClusterWithChildLists>();
        private readonly Dictionary<string, Node> nodeIdToNodes = new Dictionary<string, Node>();
        private readonly XmlTextReader xmlTextReader;

        /// <summary>
        /// an empty constructor
        /// </summary>
        public GeometryGraphReader()
        {
        }

        /// <summary>
        /// constructor witha given stream
        /// </summary>
        /// <param name="streamP"></param>
        public GeometryGraphReader(Stream streamP)
        {
            var settings = new XmlReaderSettings { IgnoreComments = false, IgnoreWhitespace = true };
            this.xmlTextReader = new XmlTextReader(streamP);
            this.XmlReader = XmlReader.Create(this.xmlTextReader, settings);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public Cluster FindClusterById(string id)
        {
            ClusterWithChildLists cwl;
            if (this.stringToClusters.TryGetValue(id, out cwl)) {
                return cwl.Cluster;
            }

            return null;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="id"></param>
        /// <returns></returns>
        public Node FindNodeById(string id)
        {
            Node node;
            if (this.nodeIdToNodes.TryGetValue(id, out node)) {
                return node;
            }

            return null;
        }

        /// <summary>
        /// creates the graph from a given file
        /// </summary>
        /// <returns></returns>
        public static GeometryGraph CreateFromFile(string fileName)
        {
            LayoutAlgorithmSettings settings;
            return CreateFromFile(fileName, out settings);
        }

        /// <summary>
        /// creates the graph and settings from a given file
        /// </summary>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Design", "CA1021:AvoidOutParameters", MessageId = "1#")]
        public static GeometryGraph CreateFromFile(string fileName, out LayoutAlgorithmSettings settings)
        {
#if TEST_MSAGL
            if (FirstCharacter(fileName) != '<') {
                settings = null;
                return null;
            }
#endif
            using (Stream stream = File.OpenRead(fileName))
            {
                var graphReader = new GeometryGraphReader(stream);
                GeometryGraph graph = graphReader.Read();
                settings = graphReader.Settings;
                return graph;
            }
        }

#if TEST_MSAGL
        private static char FirstCharacter(string fileName) {
            using (TextReader reader = File.OpenText(fileName))
            {
                var first = (char)reader.Peek();
                return first;
            }
        }
#endif

        /// <summary>
        /// Reads the graph from the stream
        /// </summary>
        /// <returns></returns>
        public GeometryGraph Read()
        {
            CultureInfo currentCulture = Thread.CurrentThread.CurrentCulture;
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
            try
            {
                this.ReadGraph();
                return this._graph;
            }
            finally
            {
                Thread.CurrentThread.CurrentCulture = currentCulture;
            }
        }

        /// <summary>
        /// reads the layout algorithm settings
        /// </summary>
        private LayoutAlgorithmSettings ReadLayoutAlgorithmSettings(XmlReader reader)
        {
            LayoutAlgorithmSettings layoutSettings = null;
            this.CheckToken(GeometryToken.LayoutAlgorithmSettings);
            if (reader.IsEmptyElement)
            {
                reader.Read();
                return null;
            }
            //reader.Read();

            var edgeRoutingMode =
                (EdgeRoutingMode)this.GetIntAttributeOrDefault(GeometryToken.EdgeRoutingMode, (int)EdgeRoutingMode.Spline);
            var str = this.GetAttribute(GeometryToken.LayoutAlgorithmType);
            if (this.XmlReader.NodeType == XmlNodeType.EndElement)
            {
                //todo - support fastincremental settings
                layoutSettings = new FastIncrementalLayoutSettings();
                EdgeRoutingSettings routingSettings = layoutSettings.EdgeRoutingSettings;
                routingSettings.EdgeRoutingMode = edgeRoutingMode;
            }
            else
            {
                if (str != null)
                {
                    var token =
                        (GeometryToken)Enum.Parse(typeof(GeometryToken), str, false);
                    if (token == GeometryToken.SugiyamaLayoutSettings)
                    {
                        layoutSettings = this.ReadSugiyamaLayoutSettings(edgeRoutingMode);
                    }
                    else if (token == GeometryToken.MdsLayoutSettings)
                    {
                        var mds = new MdsLayoutSettings();
                        EdgeRoutingSettings routingSettings = mds.EdgeRoutingSettings;
                        routingSettings.EdgeRoutingMode = edgeRoutingMode;

                        layoutSettings = mds;
                        if (this.XmlReader.IsStartElement(GeometryToken.Reporting.ToString()))
                        {
#if TEST_MSAGL
                            mds.Reporting =
#endif
 this.ReadBooleanElement(GeometryToken.Reporting);
                        }
                        mds.Exponent = ReadDoubleElement(reader);
                        mds.IterationsWithMajorization = this.ReadIntElement(GeometryToken.IterationsWithMajorization);
                        mds.PivotNumber = this.ReadIntElement(GeometryToken.PivotNumber);
                        mds.RotationAngle = ReadDoubleElement(reader);
                        mds.ScaleX = ReadDoubleElement(reader);
                        mds.ScaleY = ReadDoubleElement(reader);
                    }
                    else //todo - write a reader and a writer for FastIncrementalLayoutSettings 
{
                        throw new NotImplementedException();
                    }
                }
            }
            reader.ReadEndElement();

            return layoutSettings;
        }

        private LayoutAlgorithmSettings ReadSugiyamaLayoutSettings(EdgeRoutingMode edgeRoutingMode)
        {
            var sugiyama = new SugiyamaLayoutSettings();
            EdgeRoutingSettings routingSettings = sugiyama.EdgeRoutingSettings;
            routingSettings.EdgeRoutingMode = edgeRoutingMode;

            LayoutAlgorithmSettings layoutSettings = sugiyama;

            sugiyama.MinNodeWidth = this.GetDoubleAttributeOrDefault(GeometryToken.MinNodeWidth, sugiyama.MinNodeWidth);
            sugiyama.MinNodeHeight = this.GetDoubleAttributeOrDefault(GeometryToken.MinNodeHeight, sugiyama.MinimalHeight);
            sugiyama.AspectRatio = this.GetDoubleAttributeOrDefault(GeometryToken.AspectRatio, sugiyama.AspectRatio);
            sugiyama.NodeSeparation = this.GetDoubleAttributeOrDefault(GeometryToken.NodeSeparation, sugiyama.NodeSeparation);
            sugiyama.ClusterMargin = sugiyama.NodeSeparation;

#if TEST_MSAGL
            sugiyama.Reporting = this.GetBoolAttributeOrDefault(GeometryToken.Reporting, false);
#endif

            sugiyama.RandomSeedForOrdering = this.GetIntAttributeOrDefault(GeometryToken.RandomSeedForOrdering,
                sugiyama.RandomSeedForOrdering);
            sugiyama.NoGainAdjacentSwapStepsBound = this.GetIntAttributeOrDefault(GeometryToken.NoGainStepsBound,
                sugiyama.NoGainAdjacentSwapStepsBound);
            sugiyama.MaxNumberOfPassesInOrdering = this.GetIntAttributeOrDefault(GeometryToken.MaxNumberOfPassesInOrdering,
                sugiyama.MaxNumberOfPassesInOrdering);
            sugiyama.RepetitionCoefficientForOrdering = this.GetIntAttributeOrDefault(GeometryToken.
                RepetitionCoefficientForOrdering, sugiyama.RepetitionCoefficientForOrdering);
            sugiyama.GroupSplit = this.GetIntAttributeOrDefault(GeometryToken.GroupSplit, sugiyama.GroupSplit);
            sugiyama.LabelCornersPreserveCoefficient = this.GetDoubleAttribute(GeometryToken.LabelCornersPreserveCoefficient);
            sugiyama.BrandesThreshold = this.GetIntAttributeOrDefault(GeometryToken.BrandesThreshold,
                sugiyama.BrandesThreshold);
            sugiyama.LayerSeparation = this.GetDoubleAttributeOrDefault(GeometryToken.LayerSeparation,
                sugiyama.LayerSeparation);
            var transform = new PlaneTransformation();
            this.ReadTransform(transform);
            return layoutSettings;
        }

        private void ReadTransform(PlaneTransformation transform)
        {
            this.XmlRead();
            if (this.TokenIs(GeometryToken.Transform))
            {
                this.XmlRead();
                for (int i = 0; i < 2; i++) {
                    for (int j = 0; j < 3; j++)
                    {
                        this.CheckToken(GeometryToken.TransformElement);
                        this.MoveToContent();
                        transform[i, j] = this.ReadElementContentAsDouble();
                    }
                }

                this.XmlRead();
            }
            else
            {
                //set the unit transform
                transform[0, 0] = 1;
                transform[0, 1] = 0;
                transform[0, 2] = 0;
                transform[1, 0] = 0;
                transform[1, 1] = 1;
                transform[1, 2] = 0;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="tokens"></param>
        /// <returns></returns>
        [SuppressMessage("Microsoft.Usage", "CA1801:ReviewUnusedParameters", MessageId = "token")]
        private bool ReadBooleanElement(GeometryToken tokens)
        {
            this.CheckToken(tokens);
            return this.ReadElementContentAsBoolean();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private bool ReadElementContentAsBoolean()
        {
            return this.XmlReader.ReadElementContentAsBoolean();
        }

        private int ReadIntElement(GeometryToken token)
        {
            this.CheckToken(token);
            return this.ReadElementContentAsInt();
        }

        private static double ReadDoubleElement(XmlReader r)
        {
            return r.ReadElementContentAsDouble();
        }

        [SuppressMessage("Microsoft.Globalization", "CA1304:SpecifyCultureInfo", MessageId = "System.String.ToLower"),
         SuppressMessage("Microsoft.Globalization", "CA1308:NormalizeStringsToUppercase")]
        private void ReadGraph()
        {
            this.MoveToContent();
            this._graph.Margins = this.GetDoubleAttributeOrDefault(GeometryToken.Margins, 10);
            if (this.XmlReader.Name.ToLower() != GeometryToken.Graph.ToString().ToLower()) {
                this.Error("expecting element \"graph\"");
            }

            bool done = false;
            do
            {
                switch (this.GetElementTag())
                {
                    case GeometryToken.Nodes:
                        this.ReadNodes();
                        break;
                    case GeometryToken.Edges:
                        this.ReadEdges();
                        break;
                    case GeometryToken.Clusters:
                        this.ReadClusters();
                        break;
                    case GeometryToken.LayoutAlgorithmSettings:
                        this.Settings = this.ReadLayoutAlgorithmSettings(this.XmlReader); //todo . not tested
                        break;
                    case GeometryToken.LgLevels:
                        this.ReadLgLevels();
                        break;
                    case GeometryToken.LgSkeletonLevels:
                        this.ReadLgSkeletonLevels();
                        break;
                    case GeometryToken.End:
                    case GeometryToken.Graph:
                        if (this.XmlReader.NodeType == XmlNodeType.EndElement)
                        {
                            done = true;
                            this.ReadEndElement();
                            break;
                        }

                        //jyoti - added this if block for reloading msagl
                        if (this.XmlReader.NodeType == XmlNodeType.None)
                        {
                            done = true;
                            break;
                        }
                        this.XmlRead();
                        break;
                    default: //ignore this element
                        this.XmlReader.Skip();
                        break;

                    //                        XmlReader.Skip();
                    //                        ReadHeader();
                    //                        if (TokenIs(GeometryToken.LayoutAlgorithmSettings))
                    //                            this.Settings = ReadLayoutAlgorithmSettings(XmlReader);
                    //                        ReadNodes();
                    //                        ReadClusters();
                    //                        ReadEdges();
                }
            } while (!done);
            this._graph.BoundingBox = this._graph.PumpTheBoxToTheGraphWithMargins();
        }

        private void ReadLgLevels()
        {
            LgData lgData = new LgData(this._graph);
            this._graph.LgData = lgData;
            this.FillLgData(lgData);
            this.ReadEndElement();
        }

        private void ReadLgSkeletonLevels()
        {
            this.XmlRead();
            this.ReadSkeletonLevels(this._graph.LgData);
            this.ReadEndElement();
        }

        private void FillLgData(LgData lgData)
        {

            this.XmlRead();
            if (this.TokenIs(GeometryToken.LgEdgeInfos)) {
                this.ReadLgEdgeInfos(lgData);
            }

            if (this.TokenIs(GeometryToken.LgNodeInfos))
            {
                this.ReadLgNodeInfos(lgData);
            }
            this.ReadLevels(lgData);
        }

        private void ReadLgEdgeInfos(LgData lgData)
        {
            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }

            this.XmlRead();
            while (this.TokenIs(GeometryToken.LgEdgeInfo)) {
                this.ReadLgEdgeInfo(lgData);
            }

            this.ReadEndElement();
        }

        private void ReadLgEdgeInfo(LgData lgData)
        {
            string edgeId = this.GetAttribute(GeometryToken.EdgeId);
            Edge edge = this.idToEdges[edgeId];
            lgData.GeometryEdgesToLgEdgeInfos[edge] = new LgEdgeInfo(edge)
            {
                Rank = this.GetDoubleAttribute(GeometryToken.Rank),
                ZoomLevel = this.GetDoubleAttribute(GeometryToken.Zoomlevel)
            };
            this.XmlRead();
        }

        private void ReadLevels(LgData lgData)
        {
            int zoomLevel = 1;
            while (this.GetElementTag() == GeometryToken.Level)
            {
                var dZoomLevel = this.GetDoubleAttributeOrDefault(GeometryToken.Zoomlevel, zoomLevel);
                this.ReadLevel(lgData, (int)dZoomLevel);
                zoomLevel = 2 * (int)dZoomLevel;

            }
        }

        private void ReadSkeletonLevels(LgData lgData)
        {
            int zoomLevel = 1;
            while (this.GetElementTag() == GeometryToken.SkeletonLevel)
            {
                var dZoomLevel = this.GetDoubleAttributeOrDefault(GeometryToken.Zoomlevel, zoomLevel);
                this.ReadSkeletonLevel(lgData, (int)dZoomLevel);
                zoomLevel = 2 * (int)dZoomLevel;
            }
        }

        private void ReadLevel(LgData lgData, int zoomLevel)
        {
            int levelNodeCount = this.GetIntAttribute(GeometryToken.NodeCountOnLevel);
            if (lgData.LevelNodeCounts == null)
            {
                lgData.LevelNodeCounts = new List<int>();
            }
            lgData.LevelNodeCounts.Add(levelNodeCount);
            LgLevel level = new LgLevel(zoomLevel, this._graph);
            lgData.Levels.Add(level);
            this.XmlRead();
            Dictionary<string, Set<string>> edgeIdToEdgeRailsSet = new Dictionary<string, Set<string>>();
            this.ReadRailIdsPerEdgeIds(lgData, edgeIdToEdgeRailsSet);
            this.ReadRails(level);
            this.ReadEndElement();
            this.FillRailsOfEdges(level, edgeIdToEdgeRailsSet);
        }

        private void ReadSkeletonLevel(LgData lgData, int zoomLevel)
        {
            LgSkeletonLevel level = new LgSkeletonLevel() { ZoomLevel = zoomLevel };
            lgData.SkeletonLevels.Add(level);

            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }

            this.XmlRead();
            //ReadSkeletonRails(level);            
            this.ReadEndElement();
            //level.CreateRailTree();
        }

        private void FillRailsOfEdges(LgLevel level, Dictionary<string, Set<string>> edgeIdToEdgeRailsSet)
        {
            foreach (var edgeRails in edgeIdToEdgeRailsSet)
            {
                var edge = this.idToEdges[edgeRails.Key];
                var railSet = new Set<Rail>(edgeRails.Value.Where(s => s != "").Select(r => this.idToRails[r]));
                level._railsOfEdges[edge] = railSet;
            }
        }

        private void ReadRailIdsPerEdgeIds(LgData lgData, Dictionary<string, Set<string>> edgeIdToEdgeRailsSet)
        {
            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }

            this.XmlRead();
            while (this.TokenIs(GeometryToken.EdgeRails)) {
                this.ReadEdgeRailIds(lgData, edgeIdToEdgeRailsSet);
            }

            this.ReadEndElement();
        }

        private void ReadEdgeRailIds(LgData lgData, Dictionary<string, Set<string>> edgeIdToEdgeRailsSet)
        {
            string edgeId = this.GetAttribute(GeometryToken.EdgeId);
            Set<string> railIdSet;
            edgeIdToEdgeRailsSet[edgeId] = railIdSet = new Set<string>();
            string edgeRailsString = this.GetAttribute(GeometryToken.EdgeRails);
            LgEdgeInfo edgeInfo = lgData.GeometryEdgesToLgEdgeInfos[this.idToEdges[edgeId]];
            foreach (var railId in edgeRailsString.Split(' '))
            {
                this.UpdateToRankedEdgeInfoForRail(railId, edgeInfo);
                railIdSet.Insert(railId);
            }
            this.XmlRead();
        }

        private void UpdateToRankedEdgeInfoForRail(string railId, LgEdgeInfo edgeInfo)
        {
            LgEdgeInfo topRankeEdgeInfo;
            if (this.railIdsToTopRankedEdgeInfo.TryGetValue(railId, out topRankeEdgeInfo))
            {
                if (topRankeEdgeInfo.Rank < edgeInfo.Rank) {
                    this.railIdsToTopRankedEdgeInfo[railId] = edgeInfo;
                }
            }
            else {
                this.railIdsToTopRankedEdgeInfo[railId] = edgeInfo;
            }
        }

        private void ReadRails(LgLevel level)
        {

            this.CheckToken(GeometryToken.Rails);
            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }
            this.XmlRead();
            while (this.TokenIs(GeometryToken.Rail))
            {
                this.ReadRail(level);
            }
            this.ReadEndElement();
        }

        private void ReadSkeletonRails(LgSkeletonLevel level)
        {
            this.CheckToken(GeometryToken.Rails);
            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }
            this.XmlRead();
            while (this.TokenIs(GeometryToken.Rail))
            {
                this.ReadSkeletonRail(level);
            }
            this.ReadEndElement();
        }

        private void ReadRail(LgLevel level)
        {
            string railId = this.GetAttribute(GeometryToken.Id);
            int zoomLevel = (int)this.GetDoubleAttribute(GeometryToken.Zoomlevel);
            double minPassigEdgeZoomLevel =
                (double)this.GetDoubleAttributeOrDefault(GeometryToken.MinPassingEdgeZoomLevel, zoomLevel);
            var topRankedEdgoInfo = this.GetTopRankedEdgeInfoOfRail(railId);
            Rail rail = this.ContinueReadingRail(topRankedEdgoInfo, zoomLevel, level);
            rail.MinPassingEdgeZoomLevel = minPassigEdgeZoomLevel;
            this.idToRails[railId] = rail;

        }

        private void ReadSkeletonRail(LgSkeletonLevel level)
        {
            // do not save rails in skeleton level;
            return;
        }

        private Rail ContinueReadingRail(LgEdgeInfo topRankedEdgoInfo, int zoomLevel, LgLevel level)
        {
            this.XmlRead();
            string pointString;
            if (this.TokenIs(GeometryToken.Arrowhead))
            {
                Point arrowheadPosition = this.TryGetPointAttribute(GeometryToken.ArrowheadPosition);
                Point attachmentPoint = this.TryGetPointAttribute(GeometryToken.CurveAttachmentPoint);
                Arrowhead ah = new Arrowhead
                {
                    TipPosition = arrowheadPosition,
                    Length = (attachmentPoint - arrowheadPosition).Length
                };
                this.XmlRead();
                this.ReadEndElement();
                var rail = new Rail(ah, attachmentPoint, topRankedEdgoInfo, zoomLevel);
                var tuple = new SymmetricSegment(arrowheadPosition, attachmentPoint);
                level._railDictionary[tuple] = rail;
                return rail;
            }

            if (this.TokenIs(GeometryToken.LineSegment))
            {
                pointString = this.GetAttribute(GeometryToken.Points);
                var linePoints = this.ParsePoints(pointString);
                Debug.Assert(linePoints.Length == 2);
                LineSegment ls = new LineSegment(linePoints[0], linePoints[1]);
                this.XmlRead();
                this.ReadEndElement();
                var rail = new Rail(ls, topRankedEdgoInfo, zoomLevel);
                var tuple = new SymmetricSegment(ls.Start, ls.End);
                level._railDictionary[tuple] = rail;
                level._railTree.Add(ls.BoundingBox, rail);
                return rail;
            }
            if (this.TokenIs(GeometryToken.CubicBezierSegment))
            {
                pointString = this.GetAttribute(GeometryToken.Points);
                var controlPoints = this.ParsePoints(pointString);
                Debug.Assert(controlPoints.Length == 4);
                var bs = new CubicBezierSegment(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3]);
                this.XmlRead();
                this.ReadEndElement();
                var rail = new Rail(bs, topRankedEdgoInfo, zoomLevel);
                var tuple = new SymmetricSegment(bs.Start, bs.End);
                level._railDictionary[tuple] = rail;
                return rail;
            }
            throw new Exception();
        }

        private LgEdgeInfo GetTopRankedEdgeInfoOfRail(string railId)
        {
            if (!this.railIdsToTopRankedEdgeInfo.ContainsKey(railId)) {
                return null;
            }

            return this.railIdsToTopRankedEdgeInfo[railId];
        }

        private LgEdgeInfo GetTopRankedEdgeInfoOfSkeletonRail(string railId)
        {
            if (this.railIdsToTopRankedEdgeInfo.ContainsKey(railId)) {
                return this.railIdsToTopRankedEdgeInfo[railId];
            }

            return null;
        }

        private void ReadLgNodeInfos(LgData lgData)
        {
            if (this.XmlReader.IsEmptyElement) {
                return;
            }

            lgData.GeometryNodesToLgNodeInfos = new Dictionary<Node, LgNodeInfo>();
            lgData.SortedLgNodeInfos = new List<LgNodeInfo>();
            this.XmlRead();
            while (this.TokenIs(GeometryToken.LgNodeInfo)) {
                this.ReadLgNodeInfo(lgData);
            }

            this.ReadEndElement();
        }

        private void ReadLgNodeInfo(LgData lgData)
        {
            var nodeId = this.GetAttribute(GeometryToken.Id);
            var nodeInfo = new LgNodeInfo(this.nodeIdToNodes[nodeId])
            {
                Rank = this.GetDoubleAttribute(GeometryToken.Rank),
                ZoomLevel = this.GetDoubleAttribute(GeometryToken.Zoomlevel),
                LabelVisibleFromScale = this.GetDoubleAttributeOrDefault(GeometryToken.LabelVisibleFromScale, 1.0),
                LabelWidthToHeightRatio = this.GetDoubleAttributeOrDefault(GeometryToken.LabelWidthToHeightRatio, 1.0),
                LabelOffset = this.TryGetPointAttribute(GeometryToken.LabelOffset)
            };
            lgData.SortedLgNodeInfos.Add(nodeInfo);
            lgData.GeometryNodesToLgNodeInfos[this.nodeIdToNodes[nodeId]] = nodeInfo;
            this.XmlRead();
        }

        private GeometryToken GetElementTag()
        {
            if (this.XmlReader.NodeType == XmlNodeType.EndElement &&
                this.XmlReader.Name == "graph") {
                return GeometryToken.End;
            }

            GeometryToken token;
            if (this.XmlReader.ReadState == ReadState.EndOfFile) {
                return GeometryToken.Graph;
            }

            if (Enum.TryParse(this.XmlReader.Name, true, out token)) {
                return token;
            }

            return GeometryToken.Unknown;
        }

        private void ReadClusters()
        {
            this.XmlRead();
            while (this.TokenIs(GeometryToken.Cluster)) {
                this.ReadCluster();
            }

            this.FleshOutClusters();
            var rootClusterSet = new Set<Cluster>();
            foreach (var cluster in this.stringToClusters.Values.Select(c => c.Cluster)) {
                if (cluster.ClusterParent == null) {
                    rootClusterSet.Insert(cluster);
                }
            }

            if (rootClusterSet.Count == 1) {
                this._graph.RootCluster = rootClusterSet.First();
            } else
            {
                this._graph.RootCluster.AddRangeOfCluster(rootClusterSet);
            }

            if (!this.XmlReader.IsStartElement()) {
                this.ReadEndElement();
            }
        }

        private void FleshOutClusters()
        {
            foreach (var clusterWithLists in this.stringToClusters.Values)
            {
                var cl = clusterWithLists.Cluster;
                foreach (var i in clusterWithLists.ChildClusters) {
                    cl.AddCluster(this.stringToClusters[i].Cluster);
                }

                foreach (var i in clusterWithLists.ChildNodes) {
                    cl.AddNode(this.nodeIdToNodes[i]);
                }
            }
        }

        [SuppressMessage("Microsoft.Globalization", "CA1304:SpecifyCultureInfo", MessageId = "System.String.ToLower")]
        private void ReadCluster()
        {
            var cluster = new Cluster { RectangularBoundary = new RectangularClusterBoundary() };
            var clusterWithChildLists = new ClusterWithChildLists(cluster);
            cluster.Barycenter = this.TryGetPointAttribute(GeometryToken.Barycenter);
            var clusterId = this.GetAttribute(GeometryToken.Id);
            this.stringToClusters[clusterId] = clusterWithChildLists;
            this.ReadChildClusters(clusterWithChildLists.ChildClusters);
            this.ReadChildNodes(clusterWithChildLists.ChildNodes);
            this.XmlRead();
            switch (this.NameToToken())
            {
                case GeometryToken.ICurve:
                    cluster.BoundaryCurve = this.ReadICurve();
                    break;
                case GeometryToken.Curve:
                    cluster.BoundaryCurve = this.ReadCurve();
                    this.XmlRead();
                    break;
                case GeometryToken.Rect:
                    cluster.BoundaryCurve = this.ReadRect();
                    this.XmlRead();
                    break;
            }

            if (this.XmlReader.NodeType != XmlNodeType.EndElement) {
                cluster.RectangularBoundary = this.ReadClusterRectBoundary();
            }

            this.ReadEndElement();
        }

        private RectangularClusterBoundary ReadClusterRectBoundary()
        {
            RectangularClusterBoundary recClBnd = new RectangularClusterBoundary
            {
                LeftMargin = this.GetDoubleAttribute(GeometryToken.LeftMargin),
                RightMargin = this.GetDoubleAttribute(GeometryToken.RightMargin),
                TopMargin = this.GetDoubleAttribute(GeometryToken.TopMargin),
                BottomMargin = this.GetDoubleAttribute(GeometryToken.BottomMargin)
            };

            if (this.GetAttribute(GeometryToken.DefaultLeftMargin) != null)
            {
                var defaultLeftMargin = this.GetDoubleAttribute(GeometryToken.DefaultLeftMargin);
                var defaultRightMargin = this.GetDoubleAttribute(GeometryToken.DefaultRightMargin);
                var defaultTopMargin = this.GetDoubleAttribute(GeometryToken.DefaultBottomMargin);
                var defaultBottomMargin = this.GetDoubleAttribute(GeometryToken.DefaultBottomMargin);
                recClBnd.StoreDefaultMargin(defaultLeftMargin, defaultRightMargin, defaultBottomMargin, defaultTopMargin);
            }

            recClBnd.GenerateFixedConstraints = this.GetBoolAttributeOrDefault(GeometryToken.GenerateFixedConstraints, false);
            recClBnd.GenerateFixedConstraintsDefault =
                this.GetBoolAttributeOrDefault(GeometryToken.GenerateFixedConstraintsDefault, false);

            recClBnd.MinHeight = this.GetDoubleAttribute(GeometryToken.MinNodeHeight);
            recClBnd.MinWidth = this.GetDoubleAttribute(GeometryToken.MinNodeWidth);

            double ry;
            double left;
            double bottom;
            double w;
            double h;
            double rx;

            this.XmlRead();
            this.ReadRectParams(out left, out bottom, out w, out h, out rx, out ry);
            recClBnd.Rect = new Rectangle(left, bottom, new Point(w, h));
            recClBnd.RadiusX = rx;
            recClBnd.RadiusY = ry;


            recClBnd.RightBorderInfo = this.ReadBorderInfo(GeometryToken.RightBorderInfo);
            recClBnd.LeftBorderInfo = this.ReadBorderInfo(GeometryToken.LeftBorderInfo);
            recClBnd.TopBorderInfo = this.ReadBorderInfo(GeometryToken.TopBorderInfo);
            recClBnd.BottomBorderInfo = this.ReadBorderInfo(GeometryToken.BottomBorderInfo);
            this.XmlRead();
            this.ReadEndElement();
            return recClBnd;
        }

        private BorderInfo ReadBorderInfo(GeometryToken token)
        {
            this.XmlRead();
            this.CheckToken(token);
            var bi = new BorderInfo
            {
                InnerMargin = this.GetDoubleAttribute(GeometryToken.InnerMargin),
                FixedPosition = this.GetDoubleAttribute(GeometryToken.FixedPosition),
                Weight = this.GetDoubleAttribute(GeometryToken.Weight)
            };
            return bi;
        }

        private void ReadChildClusters(List<string> childClusters)
        {
            var clusterIds = this.GetAttribute(GeometryToken.ChildClusters);
            if (string.IsNullOrEmpty(clusterIds)) {
                return;
            }

            childClusters.AddRange(clusterIds.Split(' '));
        }

        private void ReadChildNodes(List<string> childNodes)
        {
            var nodeIds = this.GetAttribute(GeometryToken.ChildNodes);
            if (string.IsNullOrEmpty(nodeIds)) {
                return;
            }

            childNodes.AddRange(nodeIds.Split(' '));
        }

        private void ReadEdges()
        {
            this.CheckToken(GeometryToken.Edges);

            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlRead();
                return;
            }

            this.XmlRead();
            while (this.TokenIs(GeometryToken.Edge)) {
                this.ReadEdge();
            }

            this.ReadEndElement();
        }

        private void ReadEdge()
        {
            this.CheckToken(GeometryToken.Edge);
            Node s = this.ReadSourceNode();
            Node t = this.ReadTargetNode();
            var edge = new Edge(s, t)
            {
                Separation = (int)this.GetDoubleAttributeOrDefault(GeometryToken.Separation, 1),
                LineWidth = this.GetDoubleAttributeOrDefault(GeometryToken.LineWidth, 1),
                Weight = (int)this.GetDoubleAttributeOrDefault(GeometryToken.Weight, 1),

            };
            string id = this.GetAttribute(GeometryToken.Id);
            if (id != null)
            {
                Debug.Assert(this.idToEdges.ContainsKey(id) == false);
                this.idToEdges[id] = edge;
            }
            else
            {
                Debug.Assert(this.idToEdges.Count == 0); // we consistently should have no ids or unique id per edge
            }

            this.EdgeList.Add(edge);
            this.ReadArrowheadAtSource(edge);
            this.ReadArrowheadAtTarget(edge);
            this.ReadLabelFromAttribute(edge);
            bool breakTheLoop = false;
            //edge.UnderlyingPolyline = ReadUnderlyingPolyline();
            this._graph.Edges.Add(edge);
            if (this.XmlReader.IsEmptyElement)
            {
                this.XmlReader.Skip();
                return;
            }
            this.XmlRead();
            do
            {
                GeometryToken token = this.GetElementTag();
                switch (token)
                {
                    case GeometryToken.Curve:
                        edge.Curve = this.ReadICurve();
                        break;
                    case GeometryToken.LineSegment:
                        edge.Curve = this.ReadLineSeg();
                        break;
                    case GeometryToken.Edge:
                        this.ReadEndElement();
                        breakTheLoop = true;
                        break;
                    case GeometryToken.CubicBezierSegment:
                        edge.Curve = this.ReadCubucBezierSegment();
                        break;
                    case GeometryToken.Polyline:

                        break;
                    default:
                        breakTheLoop = true;
                        break;
                }
                if (!breakTheLoop) {
                    this.XmlRead();
                }
            } while (!breakTheLoop);
            //XmlReader.Skip();
        }

        private CubicBezierSegment ReadCubucBezierSegment() {
            var str = this.GetAttribute(GeometryToken.Points);
            var ss = str.Split(' ');

            var nonEmptySs = ss.Where(s => !string.IsNullOrEmpty(s)).ToList();
            if (nonEmptySs.Count != 8) {
                this.Error("wrong number of points in LineSegment");
            }

            var ds = nonEmptySs.Select(this.ParseDouble).ToArray();
            return new CubicBezierSegment(new Point(ds[0], ds[1]), new Point(ds[2], ds[3]), new Point(ds[4], ds[5]), new Point(ds[6], ds[7]) );
        }

        private void ReadArrowheadAtSource(Edge edge)
        {
            var str = this.GetAttribute(GeometryToken.As);
            var arrowhead =
                edge.EdgeGeometry.SourceArrowhead = str != null ? new Arrowhead { TipPosition = this.ParsePoint(str) } : null;
            if (arrowhead != null) {
                arrowhead.Length = this.GetDoubleAttributeOrDefault(GeometryToken.Asl, Arrowhead.DefaultArrowheadLength);
            } else
            {
                str = this.GetAttribute(GeometryToken.Asl);
                if (str != null) {
                    edge.EdgeGeometry.SourceArrowhead = new Arrowhead { Length = this.ParseDouble(str) };
                }
            }
        }

        private Point ParsePoint(string str)
        {
            var xy = str.Split(' ').ToArray();
            Debug.Assert(xy.Length == 2);
            double x, y;
            if (double.TryParse(xy[0], out x) && double.TryParse(xy[1], out y)) {
                return new Point(x, y);
            }

            this.Error("invalid point format" + str);
            return new Point();
        }

        private Point[] ParsePoints(string str)
        {
            var tokens = str.Split(' ');
            Debug.Assert(tokens.Length % 2 == 0);
            Point[] ret = new Point[tokens.Length / 2];
            for (int i = 0; i < tokens.Length - 1; i += 2)
            {
                double x, y;
                if (double.TryParse(tokens[i], out x) && double.TryParse(tokens[i + 1], out y)) {
                    ret[i / 2] = new Point(x, y);
                } else {
                    this.Error("invalid point format" + str);
                }
            }
            return ret;
        }

        private void ReadArrowheadAtTarget(Edge edge)
        {
            var str = this.GetAttribute(GeometryToken.At);
            var arrowhead =
                edge.EdgeGeometry.TargetArrowhead = str != null ? new Arrowhead { TipPosition = this.ParsePoint(str) } : null;
            if (arrowhead != null) {
                arrowhead.Length = this.GetDoubleAttributeOrDefault(GeometryToken.Atl, Arrowhead.DefaultArrowheadLength);
            } else
            {
                str = this.GetAttribute(GeometryToken.Atl);
                if (str != null) {
                    edge.EdgeGeometry.TargetArrowhead = new Arrowhead { Length = this.ParseDouble(str) };
                }
            }
        }

        private void ReadLabelFromAttribute(GeometryObject geomObj)
        {
            string str;
            if (!this.TryGetAttribute(GeometryToken.Label, out str)) {
                return;
            }

            var label = new Label(geomObj);
            Point center;
            double width, height;
            this.ParseLabel(str, out center, out width, out height);
            label.Center = center;
            label.Width = width;
            label.Height = height;

            var edge = geomObj as Edge;
            if (edge != null)
            {
                edge.Label = label;
            }
        }

        private void ParseLabel(string str, out Point center, out double width, out double height)
        {
            var ss = str.Split(' ');
            Debug.Assert(ss.Length == 4);

            center = new Point(this.ParseDouble(ss[0]), this.ParseDouble(ss[1]));
            width = this.ParseDouble(ss[2]);
            height = this.ParseDouble(ss[3]);
        }

        private double ParseDouble(string s)
        {
            double ret;
            if (double.TryParse(s, out ret)) {
                return ret;
            }

            this.Error(" cannot parse double " + s);
            return 0;
        }

        private Node ReadTargetNode()
        {
            var targetId = this.GetMustAttribute(GeometryToken.T);
            return this.GetNodeOrClusterById(targetId);
        }

        private Node ReadSourceNode()
        {
            var id = this.GetMustAttribute(GeometryToken.S);
            return this.GetNodeOrClusterById(id);
        }

        private Node GetNodeOrClusterById(string id)
        {
            Node ret;
            if (this.nodeIdToNodes.TryGetValue(id, out ret)) {
                return ret;
            }

            return this.stringToClusters[id].Cluster;

        }

        private void ReadNodes()
        {
            if (this.XmlReader.IsEmptyElement) {
                return;
            }

            this.XmlRead();
            while (this.TokenIs(GeometryToken.Node)) {
                this.ReadNode();
            }

            this.ReadEndElement();
        }

        internal const double NodeDefaultPadding = 1;

        private void ReadNode()
        {
            string nodeId = this.GetMustAttribute(GeometryToken.Id);
            double nodePadding = this.GetDoubleAttributeOrDefault(GeometryToken.Padding, NodeDefaultPadding);
            this.XmlRead();
            var node = new Node(this.ReadICurve()) { Padding = nodePadding, UserData = nodeId };
            if (node.BoundaryCurve == null) {
                throw new InvalidOperationException();
            }

            this._graph.Nodes.Add(node);
            this.XmlReader.Skip();
            this.ReadEndElement();
            this.nodeIdToNodes[nodeId] = node;
        }

        private string GetAttribute(GeometryToken token)
        {
            return this.XmlReader.GetAttribute(GeometryGraphWriter.FirstCharToLower(token));
        }

        private bool TryGetAttribute(GeometryToken token, out string val)
        {
            return (val = this.GetAttribute(token)) != null;
        }

        private string GetMustAttribute(GeometryToken token)
        {
            var s = GeometryGraphWriter.FirstCharToLower(token);
            var ret = this.XmlReader.GetAttribute(GeometryGraphWriter.FirstCharToLower(token));
            if (ret != null) {
                return ret;
            }

            this.Error("attribute " + s + " not found");
            return null;
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider",
            MessageId = "System.String.Format(System.String,System.Object)")]
        private int GetIntAttribute(GeometryToken token)
        {
            var val = this.GetAttribute(token);
            if (val == null) {
                this.Error(String.Format("attribute {0} not found", token));
            }

            int ret;
            if (int.TryParse(val, out ret)) {
                return ret;
            }

            this.Error("cannot parse an int value " + val);
            return 0;
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider",
            MessageId = "System.String.Format(System.String,System.Object)")]
        private double GetDoubleAttribute(GeometryToken token)
        {
            var val = this.GetAttribute(token);
            if (val == null) {
                this.Error(String.Format("attribute {0} not found", token));
            }

            double ret;
            if (double.TryParse(val, out ret)) {
                return ret;
            }

            this.Error("cannot parse an int value " + val);
            return 0;
        }

        private double GetDoubleAttributeOrDefault(GeometryToken token, double defaultVal)
        {
            string val = this.GetAttribute(token);
            if (val == null) {
                return defaultVal;
            }

            double ret;
            if (double.TryParse(val, out ret)) {
                return ret;
            }

            this.Error("cannot parse a double value " + val);
            return 0;
        }

        private bool GetBoolAttributeOrDefault(GeometryToken token, bool defaultVal)
        {
            string val = this.GetAttribute(token);
            if (val == null) {
                return defaultVal;
            }

            bool ret;
            if (bool.TryParse(val, out ret)) {
                return ret;
            }

            this.Error("cannot parse a bool value " + val);
            return false;
        }

        private int GetIntAttributeOrDefault(GeometryToken token, int defaultVal)
        {
            string val = this.GetAttribute(token);
            if (val == null) {
                return defaultVal;
            }

            int ret;
            if (int.TryParse(val, out ret)) {
                return ret;
            }

            this.Error("cannot parse a bool value " + val);
            return 0;
        }

        private Point TryGetPointAttribute(GeometryToken token)
        {
            string val = this.GetAttribute(token);
            return val == null ? new Point() : this.ParsePoint(val);
        }

        private void Error(string msg)
        {
            throw new InvalidOperationException(msg + ";" + this.GetPositionInfo());
        }

        private ICurve ReadICurve()
        {
            switch (this.NameToToken())
            {
                case GeometryToken.Curve:
                    return this.ReadCurve();
                case GeometryToken.Ellipse:
                    return this.ReadEllipse();
                case GeometryToken.Rect:
                    return this.ReadRect();
                case GeometryToken.Polygon:
                    return this.ReadPolygon();
                //            if (hasCurve) {
                //                XmlRead();
                //                ICurve ret = null;
                //                if (TokenIs(GeometryToken.Ellipse))
                //                    ret = ReadEllipse();
                //                else if (TokenIs(GeometryToken.Curve))
                //                    ret = ReadCurve();
                //                else if (TokenIs(GeometryToken.LineSegment))
                //                    ret = ReadLineSeg();
                //                else if (TokenIs(GeometryToken.CubicBezierSegment))
                //                    ret = ReadCubicBezierSeg();
                //                else if (TokenIs(GeometryToken.Polyline))
                //                    ret = ReadPolyline();
            }
            return null;
        }

        private ICurve ReadPolygon()
        {
            var pointString = this.GetMustAttribute(GeometryToken.Points);
            var t = pointString.Split(' ');
            if (t.Length == 0 || t.Length % 2 != 0) {
                this.Error("invalid input for the polygon");
            }

            var poly = new Polyline { Closed = true };
            for (int i = 0; i < t.Length; i += 2) {
                poly.AddPoint(new Point(this.ParseDouble(t[i]), this.ParseDouble(t[i + 1])));
            }

            return poly;
        }

        private ICurve ReadRect()
        {
            double y;
            double width;
            double height;
            double rx;
            double ry;
            double x;
            this.ReadRectParams(out x, out y, out width, out height, out rx, out ry);
            var box = new Rectangle(x, y, x + width, y + height);
            return new RoundedRect(box, rx, ry);
        }

        private void ReadRectParams(out double x, out double y, out double width, out double height, out double rx,
            out double ry)
        {
            x = this.GetDoubleAttributeOrDefault(GeometryToken.X, 0);
            y = this.GetDoubleAttributeOrDefault(GeometryToken.Y, 0);
            width = this.ParseDouble(this.GetMustAttribute(GeometryToken.Width));
            height = this.ParseDouble(this.GetMustAttribute(GeometryToken.Height));
            rx = this.GetDoubleAttributeOrDefault(GeometryToken.Rx, 0);
            ry = this.GetDoubleAttributeOrDefault(GeometryToken.Ry, 0);
        }

        private ICurve ReadCurve()
        {
            if (this.XmlReader.MoveToFirstAttribute())
            {
                var token = this.NameToToken();
                switch (token)
                {
                    case GeometryToken.CurveData:
                        return this.ParseCurve(this.XmlReader.Value);
                }
                throw new InvalidOperationException();
            }
            this.Error("No boundary curve is defined");
            return null;
        }

        private GeometryToken NameToToken()
        {
            GeometryToken token;
            if (Enum.TryParse(this.XmlReader.Name, true, out token)) {
                return token;
            }

            this.Error("cannot parse " + this.XmlReader.Name);
            return GeometryToken.Error;
        }

        private ICurve ParseCurve(string curveData)
        {
            var curve = new Curve();
            var curveStream = new CurveStream(curveData);
            var currentPoint = new Point();
            do
            {
                var curveStreamElement = curveStream.GetNextCurveStreamElement();
                if (curveStreamElement == null) {
                    return curve;
                }

                var charStreamElement = curveStreamElement as CharStreamElement;
                if (charStreamElement == null)
                {
                    this.Error("wrong formatted curve string " + curveStreamElement);
                    return null;
                }
                this.AddCurveSegment(curveStream, charStreamElement.Char, curve, ref currentPoint);
            } while (true);
        }

        [SuppressMessage("Microsoft.Maintainability", "CA1502:AvoidExcessiveComplexity")]
        private void AddCurveSegment(CurveStream curveStream, char c, Curve curve, ref Point currentPoint)
        {
            switch (c)
            {
                case 'M': //moveto
                    currentPoint = this.GetNextPointFromCurveData(curveStream);
                    break;
                case 'm': //relative moveto
                    throw new NotImplementedException();
                case 'Z':
                case 'z': //closepath
                    if (curve.Segments.Count == 0) {
                        this.Error("the curve is too short");
                    }

                    curve.AddSegment(new LineSegment(currentPoint, currentPoint = curve.Start));
                    break;
                case 'L': //lineto
                    this.ProceedWithLines(curve, curveStream, ref currentPoint);
                    break;
                case 'l': //lineto relative
                    throw new NotImplementedException();
                case 'H': //lineto horizontal
                    throw new NotImplementedException();
                case 'h': //lineto horizontal relative
                    throw new NotImplementedException();
                case 'V': //lineto vertical
                    throw new NotImplementedException();
                case 'v': //lineto vertical relative
                    throw new NotImplementedException();
                case 'C': //cubic Bezier
                    this.ProceedWithCubicBeziers(curve, curveStream, ref currentPoint);
                    break;
                case 'c': //cubic Bezier relative
                    throw new NotImplementedException();
                case 'S': //cubic Bezier shorthand
                    throw new NotImplementedException();
                case 's': //cubic Bezier relative shorthand
                    throw new NotImplementedException();
                case 'Q': //quadratic Bezier
                    throw new NotImplementedException();
                case 'q': //quadratic Bezier relative
                    throw new NotImplementedException();
                case 'T': //quadratic Bezier shorthand
                    throw new NotImplementedException();
                case 't': //quadratic Bezier relative shorthand
                    throw new NotImplementedException();
                case 'A': //elleptical arc
                    this.ReadEllepticalArc(curve, curveStream, ref currentPoint);
                    break;
                case 'a': //eleptical arc relative
                    throw new NotImplementedException();
                default:
                    this.Error("unknown character " + c);
                    break;
            }
        }

        private void ProceedWithCubicBeziers(Curve curve, CurveStream curveStream, ref Point currentPoint)
        {
            do
            {
                curve.AddSegment(new CubicBezierSegment(currentPoint, this.GetNextPointFromCurveData(curveStream),
                    this.GetNextPointFromCurveData(curveStream), currentPoint = this.GetNextPointFromCurveData(curveStream)));
            } while (curveStream.PickNextCurveStreamElement() is DoubleStreamElement);
        }

        private void ReadEllepticalArc(Curve curve, CurveStream curveStream, ref Point currentPoint)
        {
            curve.AddSegment(this.ReadEllepticalArc(curveStream, ref currentPoint));
        }

        private ICurve ReadEllepticalArc(CurveStream curveStream, ref Point currentPoint)
        {
            var rx = this.GetNextDoubleFromCurveData(curveStream);
            var ry = this.GetNextDoubleFromCurveData(curveStream);
            var xAxisRotation = this.GetNextDoubleFromCurveData(curveStream) / 180 * Math.PI;
            var largeArcFlag = (int)this.GetNextDoubleFromCurveData(curveStream);
            var sweepFlag = (int)this.GetNextDoubleFromCurveData(curveStream);
            var endPoint = this.GetNextPointFromCurveData(curveStream);
            //figure out the transform to the circle
            //then solve this problem on the circle
            if (ApproximateComparer.Close(rx, 0) || ApproximateComparer.Close(ry, 0)) {
                this.Error("ellipseArc radius is too small");
            }

            var yScale = rx / ry;
            var rotationMatrix = PlaneTransformation.Rotation(-xAxisRotation);
            var scaleMatrix = new PlaneTransformation(1, 0, 0, 0, yScale, 0);
            var transform = scaleMatrix * rotationMatrix;
            var start = transform * currentPoint;
            currentPoint = endPoint;
            var end = transform * endPoint;
            Point center;
            double startAngle;
            double endAngle;
            Point axisY;
            this.GetArcCenterAndAngles(rx, largeArcFlag, sweepFlag, start, end, out center, out startAngle, out endAngle,
                out axisY);
            var inverted = transform.Inverse;
            center = inverted * center;
            var rotation = PlaneTransformation.Rotation(xAxisRotation);
            var axisX = rotation * new Point(rx, 0);
            axisY = rotation * (axisY / yScale);
            var ret = new Ellipse(startAngle, endAngle, axisX, axisY, center);

            Debug.Assert(ApproximateComparer.Close(ret.End, endPoint));
            return ret;
        }

        private void GetArcCenterAndAngles(double r, int largeArcFlag, int sweepFlag, Point start, Point end, out Point center,
            out double startAngle, out double endAngle, out Point axisY)
        {
            var d = end - start;
            var dLenSquared = d.LengthSquared; //use it to get more precision
            var dLen = d.Length;
            //            if(dLen<r-ApproximateComparer.DistanceEpsilon)
            //                Error("arc radius is too small");

            var middle = (start + end) / 2;

            //the circle center belongs to the perpendicular to d passing through 'middle'
            d /= dLen;
            var perp = new Point(d.Y, -d.X) * Math.Sqrt(r * r - dLenSquared / 4);
            center = sweepFlag == 1 && largeArcFlag == 0 || sweepFlag == 0 && largeArcFlag == 1
                ? middle - perp
                : middle + perp;
            var axisX = new Point(r, 0);
            axisY = sweepFlag == 1 ? new Point(0, r) : new Point(0, -r);
            startAngle = Point.Angle(axisX, start - center);
            if (sweepFlag == 0) {
                startAngle = 2 * Math.PI - startAngle;
            }

            endAngle = Point.Angle(axisX, end - center);
            if (sweepFlag == 0) {
                endAngle = 2 * Math.PI - endAngle;
            }

            if (ApproximateComparer.Close(endAngle, startAngle) && largeArcFlag == 1) {
                endAngle += 2 * Math.PI;
            } else if (endAngle < startAngle) {
                endAngle += 2 * Math.PI;
            }
        }

        private void ProceedWithLines(Curve curve, CurveStream curveStream, ref Point currentPoint)
        {
            do
            {
                curve.AddSegment(new LineSegment(currentPoint, currentPoint = this.GetNextPointFromCurveData(curveStream)));
            } while (curveStream.PickNextCurveStreamElement() is DoubleStreamElement);
        }

        private Point GetNextPointFromCurveData(CurveStream curveStream)
        {
            return new Point(this.GetNextDoubleFromCurveData(curveStream), this.GetNextDoubleFromCurveData(curveStream));
        }

        private double GetNextDoubleFromCurveData(CurveStream curveStream)
        {
            var a = curveStream.GetNextCurveStreamElement();
            if (a == null) {
                this.Error("cannot parse curveData");
            }

            var d = a as DoubleStreamElement;
            if (d == null) {
                this.Error("cannot parse curveData");
            }
            // ReSharper disable PossibleNullReferenceException
            return d.Double;
            // ReSharper restore PossibleNullReferenceException
        }

        private ICurve ReadLineSeg()
        {
            this.CheckToken(GeometryToken.LineSegment);
            var str = this.GetAttribute(GeometryToken.Points);
            var ss = str.Split(' ');
            if (ss.Length != 4) {
                this.Error("wrong number of points in LineSegment");
            }

            var ds = ss.Select(this.ParseDouble).ToArray();
            return new LineSegment(new Point(ds[0], ds[1]), new Point(ds[2], ds[3]));
        }

        private ICurve ReadEllipse()
        {
            var cx = this.ParseDouble(this.GetMustAttribute(GeometryToken.Cx));
            var cy = this.ParseDouble(this.GetMustAttribute(GeometryToken.Cy));
            var rx = this.ParseDouble(this.GetMustAttribute(GeometryToken.Rx));
            var ry = this.ParseDouble(this.GetMustAttribute(GeometryToken.Ry));
            return new Ellipse(rx, ry, new Point(cx, cy));
        }

        private bool TokenIs(GeometryToken t)
        {
            return this.XmlReader.IsStartElement(GeometryGraphWriter.FirstCharToLower(t)) ||
                   this.XmlReader.IsStartElement(t.ToString());
        }

        private void MoveToContent()
        {
            this.XmlReader.MoveToContent();
        }

        /// <summary>
        /// the xml reader
        ///</summary>
        private XmlReader XmlReader { get; set; }

        /// <summary>
        /// the xml reader
        ///<parameter>the reader</parameter>
        /// </summary>
        public void SetXmlReader(XmlReader reader)
        {
            this.XmlReader = reader;
        }

        ///<summary>
        ///used only in Debug configuration
        ///<param name="token">the token that should be here</param>
        ///</summary>
        private void CheckToken(GeometryToken token)
        {
            if (!this.XmlReader.IsStartElement(GeometryGraphWriter.FirstCharToLower(token)) &&
                !this.XmlReader.IsStartElement(token.ToString()))
            {
                string positionInfo = this.GetPositionInfo();
                throw new InvalidDataException(
                    String.Format(CultureInfo.InvariantCulture,
                        "expected {0}, {1}", token, positionInfo));
            }
        }

        private string GetPositionInfo()
        {
            if (this.xmlTextReader != null) {
                return String.Format(CultureInfo.InvariantCulture, "line {0} col {1}", this.xmlTextReader.LineNumber,
                    this.xmlTextReader.LinePosition);
            }

            return String.Empty;
        }

        ///<summary>
        ///reads the end element
        ///</summary>
        private void ReadEndElement()
        {
            this.XmlReader.ReadEndElement();
        }


        ///<summary>
        /// reads a double
        ///</summary>        
        private double ReadElementContentAsDouble()
        {
            return this.XmlReader.ReadElementContentAsDouble();
        }

        ///<summary>
        ///reads the line?
        ///</summary>
        private void XmlRead()
        {
            this.XmlReader.Read();
        }


        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private int ReadElementContentAsInt()
        {
            return this.XmlReader.ReadElementContentAsInt();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="disposing"></param>
        protected virtual void Dispose(bool disposing)
        {
            if (disposing) {
                this.xmlTextReader.Close();
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public void Dispose()
        {
            this.Dispose(true);
            GC.SuppressFinalize(this);
        }
    }
}

#endif