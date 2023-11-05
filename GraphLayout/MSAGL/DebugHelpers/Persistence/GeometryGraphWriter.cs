using Microsoft.Msagl.Layout.LargeGraphLayout;
#if TEST_MSAGL
using System;
using System.Linq;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Xml;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Layout.Layered;
using Microsoft.Msagl.Layout.MDS;

namespace Microsoft.Msagl.DebugHelpers.Persistence
{
    /// <summary>
    /// writes a GeometryGraph to a stream
    /// </summary>
    [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1702:CompoundWordsShouldBeCasedCorrectly", MessageId = "GraphWriter")]
    public class GeometryGraphWriter
    {
        private const string FileExtension = ".msagl.geom";
        private Dictionary<Node, string> nodeIds = new Dictionary<Node, string>();
        private Dictionary<Edge, int> edgeIds = new Dictionary<Edge, int>();

        /// <summary>
        /// 
        /// </summary>
        private GeometryGraph graph;
        private bool needToCloseXmlWriter = true;
        private Stream stream;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="streamPar">the stream to write the graph into</param>
        /// <param name="graphP">the graph</param>
        /// <param name="settings">The settings to be written.</param>
        public GeometryGraphWriter(Stream streamPar, GeometryGraph graphP, LayoutAlgorithmSettings settings)
        {
            this.stream = streamPar;
            this.Graph = graphP;
            this.Settings = settings;
            var xmlWriterSettings = new XmlWriterSettings { Indent = true };
            this.XmlWriter = XmlWriter.Create(this.stream, xmlWriterSettings);
            this.EdgeEnumeration = graphP.Edges;
        }

        /// <summary>
        /// an empty constructor
        /// </summary>
        public GeometryGraphWriter() { }

        /// <summary>
        /// if set to true then the XmlWriter will be closed after the graph writing
        /// </summary>
        [SuppressMessage("Microsoft.Design", "CA1044:PropertiesShouldNotBeWriteOnly")]
        public bool NeedToCloseXmlWriter
        {
            get { return this.needToCloseXmlWriter; }
            set { this.needToCloseXmlWriter = value; }
        }

        /// <summary>
        /// the stream to write the graph into
        /// </summary>
        public Stream Stream
        {
            get { return this.stream; }
            set { this.stream = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        [SuppressMessage("Microsoft.Design", "CA1044:PropertiesShouldNotBeWriteOnly")]
        public XmlWriter XmlWriter { get; set; }

        /// <summary>
        /// the graph
        /// </summary>
        public GeometryGraph Graph
        {
            get { return this.graph; }
            set { this.graph = value; }
        }

        /// <summary>
        /// The settings
        /// </summary>
        public LayoutAlgorithmSettings Settings { get; set; }

        /// <summary>
        /// saves the graph to a file
        /// </summary>
        public static void Write(GeometryGraph graph, string fileName)
        {
            Write(graph, null, fileName);
        }

        /// <summary>
        /// saves the graph and settings to a file
        /// </summary>
        [SuppressMessage("Microsoft.Globalization", "CA1309:UseOrdinalStringComparison",
            MessageId = "System.String.EndsWith(System.String,System.StringComparison)"),
         SuppressMessage("Microsoft.Globalization", "CA1309:UseOrdinalStringComparison",
             MessageId = "System.String.EndsWith(System.String,System.Boolean,System.Globalization.CultureInfo)")]
        public static void Write(GeometryGraph graph, LayoutAlgorithmSettings settings, string fileName)
        {
            if (fileName == null) {
                return;
            }

            if (!fileName.EndsWith(FileExtension, StringComparison.InvariantCultureIgnoreCase)) {
                fileName += FileExtension;
            }

            using (Stream stream = File.Open(fileName, FileMode.Create))
            {
                var graphWriter = new GeometryGraphWriter(stream, graph, settings);
                graphWriter.Write();
            }
        }

        /// <summary>
        /// Writes the graph to a file
        /// </summary>
        public void Write()
        {
            var currentCulture = Thread.CurrentThread.CurrentCulture;
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
            try
            {
                this.Open();
                this.WriteLayoutSettings();

                this.InitEdgeIds();
                this.WriteNodes();
                this.WriteClusters();
                this.WriteEdges();
                this.WriteLayers();
                this.Close();
            }
            finally
            {
                Thread.CurrentThread.CurrentCulture = currentCulture;
            }
        }

        private void WriteLayers()
        {
            if (this.graph.LgData == null) {
                return;
            }

            this.WriteStartElement(GeometryToken.LgLevels);
            this.WriteLgEdgeInfos();
            this.WriteSortedLgInfos();
            Dictionary<Rail, int> railIds = this.CreateRailIds();

            for (int i = 0; i < this.graph.LgData.Levels.Count; i++)
            {
                this.WriteLevel(this.graph.LgData.Levels[i], railIds, this.graph.LgData.LevelNodeCounts[i]);
            }
            this.WriteEndElement();

            this.WriteStartElement(GeometryToken.LgSkeletonLevels);
            for (int i = 0; i < this.graph.LgData.SkeletonLevels.Count; i++)
            {
                this.WriteSkeletonLevel(this.graph.LgData.SkeletonLevels[i], railIds);
            }
            this.WriteEndElement();

        }

        private void WriteLgEdgeInfos()
        {
            this.WriteStartElement(GeometryToken.LgEdgeInfos);
            foreach (var t in this.graph.LgData.GeometryEdgesToLgEdgeInfos)
            {
                var edge = t.Key;
                var ei = t.Value;
                this.WriteLgEdgeInfo(edge, ei);
            }
            this.WriteEndElement();
        }

        private void WriteLgEdgeInfo(Edge edge, LgEdgeInfo ei)
        {
            this.WriteStartElement(GeometryToken.LgEdgeInfo);
            this.WriteAttribute(GeometryToken.EdgeId, this.edgeIds[edge]);
            this.WriteAttribute(GeometryToken.Rank, ei.Rank);
            this.WriteAttribute(GeometryToken.Zoomlevel, ei.ZoomLevel);
            this.WriteEndElement();
        }

        private void WriteLevel(LgLevel level, Dictionary<Rail, int> railsToIds, int nodeCountOnLevel)
        {
            this.WriteStartElement(GeometryToken.Level);
            this.WriteAttribute(GeometryToken.NodeCountOnLevel, nodeCountOnLevel);
            this.WriteAttribute(GeometryToken.Zoomlevel, level.ZoomLevel);
            this.WriteLevelRails(level, railsToIds);
            this.WriteEndElement();
        }

        private void WriteSkeletonLevel(LgSkeletonLevel level, Dictionary<Rail, int> railsToIds)
        {
            this.WriteStartElement(GeometryToken.SkeletonLevel);
            //WriteAttribute(GeometryToken.NodeCountOnLevel, nodeCountOnLevel);
            this.WriteAttribute(GeometryToken.Zoomlevel, level.ZoomLevel);
            this.WriteEndElement();
        }

        private void WriteLevelRails(LgLevel level, Dictionary<Rail, int> railIds)
        {
            this.WriteStartElement(GeometryToken.RailsPerEdge);
            foreach (var t in level._railsOfEdges)
            {
                this.WriteEdgeRails(t.Key, t.Value, railIds);
            }
            this.WriteEndElement();
            this.WriteRailsGeometry(level, railIds);
        }

        private void WriteRailsGeometry(LgLevel level, Dictionary<Rail, int> railIds)
        {
            this.WriteStartElement(GeometryToken.Rails);
            foreach (var rail in level._railDictionary.Values) {
                this.WriteRail(rail, railIds[rail]);
            }

            this.WriteEndElement();
        }

        private void WriteRail(Rail rail, int railId)
        {
            this.WriteStartElement(GeometryToken.Rail);
            this.WriteAttribute(GeometryToken.Id, railId);
            this.WriteAttribute(GeometryToken.Zoomlevel, rail.ZoomLevel);
            if (rail.MinPassingEdgeZoomLevel != Double.MaxValue) {
                this.WriteAttribute(GeometryToken.MinPassingEdgeZoomLevel, rail.MinPassingEdgeZoomLevel);
            }

            Arrowhead ah = rail.Geometry as Arrowhead;
            if (ah != null)
            {
                this.WriteStartElement(GeometryToken.Arrowhead);
                this.WriteAttribute(GeometryToken.ArrowheadPosition, ah.TipPosition);
                this.WriteAttribute(GeometryToken.CurveAttachmentPoint, rail.CurveAttachmentPoint);
                this.WriteEndElement();
            }
            else
            {
                ICurve curve = rail.Geometry as ICurve;
                if (curve != null) {
                    this.WriteICurve(curve);
                } else {
                    throw new InvalidOperationException();
                }
            }
            this.WriteEndElement();
        }

        private void WriteEdgeRails(Edge edge, Set<Rail> rails, Dictionary<Rail, int> railIds)
        {
            this.WriteStartElement(GeometryToken.EdgeRails);
            this.WriteAttribute(GeometryToken.EdgeId, this.edgeIds[edge]);
            List<string> railIdStrings = new List<string>();
            foreach (var rail in rails)
            {
                railIdStrings.Add(railIds[rail].ToString());
            }

            this.WriteAttribute(GeometryToken.EdgeRails, String.Join(" ", railIdStrings));
            this.WriteEndElement();
        }

        private Dictionary<Rail, int> CreateRailIds()
        {
            var ret = new Dictionary<Rail, int>();
            int id = 0;
            foreach (var level in this.graph.LgData.Levels)
            {
                foreach (var rail in level._railDictionary.Values)
                {
                    if (ret.ContainsKey(rail))
                    {
                        continue;
                    }
                    ret[rail] = id++;
                }
            }
            return ret;
        }

        private void WriteSortedLgInfos()
        {
            this.WriteStartElement(GeometryToken.LgNodeInfos);
            foreach (var lgNodeInfo in this.graph.LgData.SortedLgNodeInfos)
            {
                this.WriteLgNodeInfo(lgNodeInfo);
            }
            this.WriteEndElement();
        }

        private void WriteLgNodeInfo(LgNodeInfo lgNodeInfo)
        {
            this.WriteStartElement(GeometryToken.LgNodeInfo);
            this.WriteAttribute(GeometryToken.Id, this.nodeIds[lgNodeInfo.GeometryNode]);
            this.WriteAttribute(GeometryToken.Rank, lgNodeInfo.Rank);
            this.WriteAttribute(GeometryToken.Zoomlevel, lgNodeInfo.ZoomLevel);
            this.WriteAttribute(GeometryToken.LabelVisibleFromScale, lgNodeInfo.LabelVisibleFromScale);
            this.WriteAttribute(GeometryToken.LabelOffset, lgNodeInfo.LabelOffset);
            this.WriteAttribute(GeometryToken.LabelWidthToHeightRatio, lgNodeInfo.LabelWidthToHeightRatio);
            this.WriteEndElement();
        }

        private void InitEdgeIds()
        {
            int id = 0;
            foreach (var e in this.graph.Edges) {
                this.edgeIds[e] = id++;
            }
        }

        /// <summary>
        /// roman: Writes the graph to an Ipe file
        /// todo
        /// </summary>
        public void WriteIpe()
        {
            var currentCulture = Thread.CurrentThread.CurrentCulture;
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
            try
            {
                this.Open();
                this.WriteNodesIpe();
                this.Close();
            }
            finally
            {
                Thread.CurrentThread.CurrentCulture = currentCulture;
            }
        }

        private void WriteClusters()
        {
            if (this.graph.RootCluster == null) {
                return;
            }

            this.WriteStartElement(GeometryToken.Clusters);

            this.MapClustersToIds(this.graph.RootCluster);

            foreach (var cluster in this.graph.RootCluster.AllClustersDepthFirstExcludingSelf()) {
                this.WriteCluster(cluster, this.nodeIds[cluster]);
            }

            this.WriteEndElement();
        }

        private void WriteCluster(Cluster cluster, string clusterId)
        {
            this.WriteStartElement(GeometryToken.Cluster);
            this.WriteAttribute(GeometryToken.Id, clusterId);
            this.WriteAttribute(GeometryToken.Barycenter, cluster.Barycenter);
            this.WriteChildClusters(cluster);
            this.WriteChildNodes(cluster);
            if (cluster.BoundaryCurve != null) {
                this.WriteICurve(cluster.BoundaryCurve);
            }

            this.WriteClusterRectBoundary(cluster.RectangularBoundary);
            this.WriteEndElement();
        }

        private void WriteClusterRectBoundary(RectangularClusterBoundary recClBnd)
        {
            if (recClBnd == null) {
                return;
            }

            this.WriteStartElement(GeometryToken.RectangularClusterBoundary);

            this.WriteAttribute(GeometryToken.LeftMargin, recClBnd.LeftMargin);
            this.WriteAttribute(GeometryToken.RightMargin, recClBnd.RightMargin);
            this.WriteAttribute(GeometryToken.TopMargin, recClBnd.TopMargin);
            this.WriteAttribute(GeometryToken.BottomMargin, recClBnd.BottomMargin);
            if (recClBnd.DefaultMarginIsSet)
            {
                this.WriteAttribute(GeometryToken.DefaultLeftMargin, recClBnd.DefaultLeftMargin);
                this.WriteAttribute(GeometryToken.DefaultRightMargin, recClBnd.DefaultRightMargin);
                this.WriteAttribute(GeometryToken.DefaultTopMargin, recClBnd.DefaultTopMargin);
                this.WriteAttribute(GeometryToken.DefaultBottomMargin, recClBnd.DefaultBottomMargin);
            }
            this.WriteAttribute(GeometryToken.GenerateFixedConstraints, recClBnd.GenerateFixedConstraints);
            this.WriteAttribute(GeometryToken.GenerateFixedConstraintsDefault,
                recClBnd.GenerateFixedConstraintsDefault);
            this.WriteAttribute(GeometryToken.MinNodeHeight, recClBnd.MinHeight);
            this.WriteAttribute(GeometryToken.MinNodeWidth, recClBnd.MinWidth);
            this.WriteRect(recClBnd.Rect.Left, recClBnd.Rect.Bottom, recClBnd.Rect.Width, recClBnd.Rect.Height, recClBnd.RadiusX, recClBnd.RadiusY);
            this.WriteBorderInfo(GeometryToken.RightBorderInfo, recClBnd.RightBorderInfo);
            this.WriteBorderInfo(GeometryToken.LeftBorderInfo, recClBnd.LeftBorderInfo);
            this.WriteBorderInfo(GeometryToken.TopBorderInfo, recClBnd.TopBorderInfo);
            this.WriteBorderInfo(GeometryToken.BottomBorderInfo, recClBnd.BottomBorderInfo);
            this.WriteEndElement();
        }

        private void WriteBorderInfo(GeometryToken token, BorderInfo borderInfo)
        {
            this.WriteStartElement(token);
            this.WriteAttribute(GeometryToken.InnerMargin, borderInfo.InnerMargin);
            this.WriteAttribute(GeometryToken.FixedPosition, borderInfo.FixedPosition);
            this.WriteAttribute(GeometryToken.Weight, borderInfo.Weight);
            this.WriteEndElement();
        }

        private void WriteChildNodes(Cluster cluster)
        {
            this.WriteAttribute(GeometryToken.ChildNodes,
                string.Join(" ",
                cluster.nodes.Select(child => this.nodeIds[child].ToString(CultureInfo.InvariantCulture))));
        }

        private void WriteChildClusters(Cluster cluster)
        {
            this.WriteAttribute(GeometryToken.ChildClusters,
            String.Join(" ", cluster.Clusters.Select(child => this.NodeToIds[child])));
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.Int32.ToString"
            )]
        private void MapClustersToIds(Cluster cluster)
        {
            string id;
            var setOfIds = new Set<string>(this.nodeIds.Values);
            foreach (Cluster child in cluster.AllClustersDepthFirst())
            {
                if (!this.nodeIds.TryGetValue(child, out id))
                {
                    id = this.FindNewId(setOfIds);
                    this.nodeIds[child] = id;
                    setOfIds.Insert(id);
                }
            }
        }

        private string FindNewId(Set<string> setOfIds)
        {
            int i = this.nodeIds.Count;
            do
            {
                var s = i.ToString();
                if (!setOfIds.Contains(s)) {
                    return s;
                }

                i++;
            } while (true);
        }


        [SuppressMessage("Microsoft.Globalization", "CA1304:SpecifyCultureInfo", MessageId = "System.String.ToLower")]
        private void Open()
        {
            this.XmlWriter.WriteStartElement(GeometryToken.Graph.ToString().ToLower());
            this.WriteAttribute(GeometryToken.Margins, this.graph.Margins);
        }

        private void Close()
        {
            this.XmlWriter.WriteEndElement();
            if (this.NeedToCloseXmlWriter)
            {
                this.XmlWriter.WriteEndDocument();
                this.XmlWriter.Flush();
                this.XmlWriter.Close();
            }
        }

        private void WriteEdges()
        {
            this.WriteStartElement(GeometryToken.Edges);
            foreach (Edge edge in this.EdgeEnumeration) {
                this.WriteEdge(edge);
            }

            this.WriteEndElement();
        }

        private string NodeOrClusterId(Node node)
        {
            return this.nodeIds[node];
        }

        private void WriteEdge(Edge edge)
        {
            this.WriteStartElement(GeometryToken.Edge);
            this.WriteAttribute(GeometryToken.Id, this.edgeIds[edge]);
            this.WriteAttribute(GeometryToken.S, this.NodeOrClusterId(edge.Source).ToString(CultureInfo.InvariantCulture));
            this.WriteAttribute(GeometryToken.T, this.NodeOrClusterId(edge.Target).ToString(CultureInfo.InvariantCulture));
            if (edge.LineWidth != 1) {
                this.WriteAttribute(GeometryToken.LineWidth, edge.LineWidth);
            }

            if (edge.ArrowheadAtSource)
            {
                this.WriteAttribute(GeometryToken.As, edge.EdgeGeometry.SourceArrowhead.TipPosition);
                this.WriteDefaultDouble(GeometryToken.Asl, edge.EdgeGeometry.SourceArrowhead.Length,
                                   Arrowhead.DefaultArrowheadLength);
            }
            if (edge.ArrowheadAtTarget)
            {
                this.WriteAttribute(GeometryToken.At, edge.EdgeGeometry.TargetArrowhead.TipPosition);
                this.WriteDefaultDouble(GeometryToken.Atl, edge.EdgeGeometry.TargetArrowhead.Length,
                                   Arrowhead.DefaultArrowheadLength);
            }

            if (edge.Weight != 1) {
                this.WriteAttribute(GeometryToken.Weight, edge.Weight);
            }

            if (edge.Separation != 1) {
                this.WriteAttribute(GeometryToken.Separation, edge.Separation);
            }

            if (edge.Label != null) {
                this.WriteLabel(edge.Label);
            }

            this.WriteICurve(edge.Curve);
            this.WriteEndElement();
        }

        private void WriteDefaultDouble(GeometryToken geometryToken, double val, double defaultValue)
        {
            if (val != defaultValue) {
                this.WriteAttribute(geometryToken, this.DoubleToString(val));
            }
        }

        private void WriteAttribute(GeometryToken attrKind, object val)
        {
            var attrString = FirstCharToLower(attrKind);
            if (val is Point) {
                this.XmlWriter.WriteAttributeString(attrString, this.PointToString((Point)val));
            } else if (val is Double) {
                this.XmlWriter.WriteAttributeString(attrString, this.DoubleToString((double)val));
            } else {
                this.XmlWriter.WriteAttributeString(attrString, val.ToString());
            }
        }

        [SuppressMessage("Microsoft.Globalization", "CA1308:NormalizeStringsToUppercase")]
        static internal string FirstCharToLower(GeometryToken attrKind)
        {
            var attrString = attrKind.ToString();
            attrString = attrString.Substring(0, 1).ToLower(CultureInfo.InvariantCulture) + attrString.Substring(1, attrString.Length - 1);
            return attrString;
        }

        private string PointToString(Point start)
        {
            return this.DoubleToString(start.X) + " " + this.DoubleToString(start.Y);
        }

        private string formatForDoubleString = "#.###########";
        private int precision = 11;
        private IEnumerable<Edge> edgeEnumeration;

        ///<summary>
        ///</summary>
        public int Precision
        {
            get { return this.precision; }
            set
            {
                this.precision = Math.Max(1, value);
                var s = new char[this.precision + 2];
                s[0] = '#';
                s[1] = '.';
                for (int i = 0; i < this.precision; i++) {
                    s[2 + i] = '#';
                }

                this.formatForDoubleString = new string(s);
            }

        }

        ///<summary>
        /// a mapping from nodes to their ids
        ///</summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA2227:CollectionPropertiesShouldBeReadOnly")]

        public Dictionary<Node, string> NodeToIds
        {
            get { return this.nodeIds; }
            set { this.nodeIds = value; }
        }

        /// <summary>
        /// this enumeration is used in a combination with GraphWriter, to dictate the order of edges
        /// </summary>
        public IEnumerable<Edge> EdgeEnumeration
        {
            get { return this.edgeEnumeration; }
            set { this.edgeEnumeration = value; }
        }

        private string DoubleToString(double d)
        {
            return (Math.Abs(d) < 1e-11) ? "0" : d.ToString(this.formatForDoubleString, CultureInfo.InvariantCulture);
        }

        private void WriteLabel(Label label)
        {
            this.WriteAttribute(GeometryToken.Label, this.LabelToString(label));
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.String.Format(System.String,System.Object,System.Object,System.Object)")]
        private string LabelToString(Label label)
        {
            return String.Format("{0} {1} {2}", this.PointToString(label.Center), this.DoubleToString(label.Width), this.DoubleToString(label.Height));
        }

        private void WriteNodes()
        {
            this.WriteStartElement(GeometryToken.Nodes);
            foreach (Node node in this.Graph.Nodes) {
                this.WriteNode(node);
            }

            this.WriteEndElement();
        }

        private void WriteNodesIpe()
        {
            foreach (Node node in this.Graph.Nodes) {
                this.WriteNodeIpe(node);
            }
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.Int32.ToString")]
        private void WriteNode(Node node)
        {
            string id;
            if (!this.nodeIds.TryGetValue(node, out id)) {
                this.nodeIds[node] = id = this.nodeIds.Count.ToString();
            }

            this.WriteStartElement(GeometryToken.Node);
            this.WriteAttribute(GeometryToken.Id, id);
            if (node.Padding != GeometryGraphReader.NodeDefaultPadding) {
                this.WriteAttribute(GeometryToken.Padding, node.Padding);
            }

            this.WriteICurve(node.BoundaryCurve);
            this.WriteEndElement();
        }

        private void WriteNodeIpe(Node node)
        {
            string id;
            if (!this.nodeIds.TryGetValue(node, out id)) {
                this.nodeIds[node] = id = this.nodeIds.Count.ToString();
            }
            // todo: add Id as text
            if (node.BoundaryCurve != null)
            {
                this.WriteICurveIpe(node.BoundaryCurve);
                this.WriteLabelIpe(node.BoundaryCurve.BoundingBox.Center, "" + id);
            }
        }

        private void WriteLabelIpe(Point p, string label)
        {
            this.XmlWriter.WriteStartElement("text");
            this.XmlWriter.WriteAttributeString("pos", p.X + " " + p.Y);
            this.XmlWriter.WriteAttributeString("halign", "center");
            this.XmlWriter.WriteAttributeString("valign", "center");
            this.XmlWriter.WriteString(label);
            this.XmlWriter.WriteEndElement();
        }

        private void WriteICurve(ICurve iCurve)
        {
            if (iCurve == null) {
                return;
            }

            var rect = iCurve as RoundedRect;
            if (rect != null) {
                this.WriteRect(rect.BoundingBox.Left, rect.BoundingBox.Bottom, rect.BoundingBox.Width,
                          rect.BoundingBox.Height, rect.RadiusX, rect.RadiusY);
            } else
            {
                var c = iCurve as Curve;
                if (c != null) {
                    this.WriteCurveInSvgStyle(c);
                } else
                {
                    var ellipse = iCurve as Ellipse;
                    if (ellipse != null) {
                        this.WriteEllipseInSvgStyle(ellipse);
                    } else
                    {
                        var poly = iCurve as Polyline;
                        if (poly != null) {
                            this.WritePolylineInSvgStyle(poly);
                        } else
                        {
                            var ls = iCurve as LineSegment;
                            if (ls != null) {
                                this.WriteLineSeg(ls);
                            } else
                            {
                                var bs = iCurve as CubicBezierSegment;
                                if (bs != null)
                                {
                                    this.WriteBezierSegment(bs);
                                }
                                else {
                                    throw new InvalidOperationException();
                                }
                            }
                        }

                    }
                }
            }
        }

        private void WriteBezierSegment(CubicBezierSegment bs)
        {
            this.WriteStartElement(GeometryToken.CubicBezierSegment);
            this.WriteAttribute(GeometryToken.Points, this.PointsToString(bs.B(0), bs.B(1), bs.B(2), bs.B(3)));
            this.WriteEndElement();
        }

        private void WriteICurveIpe(ICurve iCurve)
        {
            if (iCurve == null) {
                return;
            }

            var rect = iCurve as RoundedRect;
            if (rect != null) {
                this.WriteRectIpe(rect.BoundingBox.Left, rect.BoundingBox.Bottom, rect.BoundingBox.Width,
                          rect.BoundingBox.Height, rect.RadiusX, rect.RadiusY);
            }

            return;
        }

        private void WriteRect(double x, double y, double width, double height, double rx, double ry)
        {
            this.WriteStartElement(GeometryToken.Rect);
            this.WriteAttribute(GeometryToken.X, x);
            this.WriteAttribute(GeometryToken.Y, y);
            this.WriteAttribute(GeometryToken.Width, width);
            this.WriteAttribute(GeometryToken.Height, height);
            if (rx > 0) {
                this.WriteAttribute(GeometryToken.Rx, rx);
            }

            if (ry > 0) {
                this.WriteAttribute(GeometryToken.Ry, ry);
            }

            this.WriteEndElement();
        }

        private void WriteRectIpe(double x, double y, double width, double height, double rx, double ry)
        {
            this.XmlWriter.WriteStartElement("path");
            this.XmlWriter.WriteString("\n" + x + " " + y + " m\n");
            this.XmlWriter.WriteString((x + width) + " " + y + " l\n");
            this.XmlWriter.WriteString((x + width) + " " + (y + height) + " l\n");
            this.XmlWriter.WriteString(x + " " + (y + height) + " l\nh\n");
            this.XmlWriter.WriteEndElement();
        }

        private void WritePolylineInSvgStyle(Polyline poly)
        {
            this.WriteStartElement(poly.Closed ? GeometryToken.Polygon : GeometryToken.Polyline);
            this.WriteAttribute(GeometryToken.Points, this.PointsToString(poly.ToArray()));
            this.WriteEndElement();
        }

        private void WriteEllipseInSvgStyle(Ellipse ellipse)
        {
            if (ApproximateComparer.Close(ellipse.ParStart, 0) && ApproximateComparer.Close(ellipse.ParEnd, 2 * Math.PI)) { this.WriteFullEllipse(ellipse); }
            else
            {
                WriteEllepticalArc(ellipse);
            }
        }

        private void WriteFullEllipse(Ellipse ellipse)
        {
            this.WriteStartElement(GeometryToken.Ellipse);
            this.WriteAttribute(GeometryToken.Cx, ellipse.Center.X);
            this.WriteAttribute(GeometryToken.Cy, ellipse.Center.Y);
            this.WriteAttribute(GeometryToken.Rx, ellipse.AxisA.Length);
            this.WriteAttribute(GeometryToken.Ry, ellipse.AxisB.Length);
            this.WriteEndElement();
        }


        // ReSharper disable UnusedParameter.Local
        [SuppressMessage("Microsoft.Usage", "CA1801:ReviewUnusedParameters", MessageId = "ellipse")]
        private static void WriteEllepticalArc(Ellipse ellipse)
        {
            // ReSharper restore UnusedParameter.Local
            throw new NotImplementedException();
        }

        private void WriteCurveInSvgStyle(Curve curve)
        {
            this.WriteStartElement(GeometryToken.Curve);
            this.WriteAttribute(GeometryToken.CurveData, this.CurveString(curve));
            this.WriteEndElement();
        }

        private string CurveString(ICurve iCurve)
        {
            return String.Join(" ", this.CurveStringTokens(iCurve));
        }

        private IEnumerable<string> CurveStringTokens(ICurve iCurve)
        {
            yield return "M";
            yield return this.PointToString(iCurve.Start);
            var curve = iCurve as Curve;
            var previousInstruction = 'w'; //a character that is not used by the SVG curve
            if (curve != null) {
                for (int i = 0; i < curve.Segments.Count; i++)
                {
                    var segment = curve.Segments[i];
                    if (i != curve.Segments.Count - 1) {
                        yield return this.SegmentString(segment, ref previousInstruction);
                    } else
                    { //it is the last seg
                        if (segment is LineSegment && ApproximateComparer.Close(segment.End, iCurve.Start)) {
                            yield return "Z";
                        } else {
                            yield return this.SegmentString(segment, ref previousInstruction);
                        }
                    }
                }
            }
        }

        private string SegmentString(ICurve segment, ref char previousInstruction)
        {
            var ls = segment as LineSegment;
            if (ls != null)
            {
                var str = this.LineSegmentString(ls, previousInstruction);
                previousInstruction = 'L';
                return str;
            }
            var cubic = segment as CubicBezierSegment;
            if (cubic != null)
            {
                var str = this.CubicBezierSegmentToString(cubic, previousInstruction);
                previousInstruction = 'C';
                return str;
            }
            var ellipseArc = segment as Ellipse;
            if (ellipseArc != null)
            {
                previousInstruction = 'A';
                return this.EllipticalArcToString(ellipseArc);

            }
            throw new NotImplementedException();
        }

        [SuppressMessage("Microsoft.Globalization", "CA1305:SpecifyIFormatProvider", MessageId = "System.Int32.ToString")]
        private string EllipticalArcToString(Ellipse ellipse)
        {
            /*
             * rx ry x-axis-rotation large-arc-flag sweep-flag x y
             * */
            //In general in an Msagl ellipse the axes don't have to be orthogonal: we have a possible bug here
            var rx = "A" + this.DoubleToString(ellipse.AxisA.Length);
            var ry = this.DoubleToString(ellipse.AxisB.Length);
            var xAxisRotation = this.DoubleToString(180 * Point.Angle(new Point(1, 0), ellipse.AxisA) / Math.PI);
            var largeArcFlag = Math.Abs(ellipse.ParEnd - ellipse.ParStart) >= Math.PI ? "1" : "0";
            var sweepFlagInt = ellipse.ParEnd > ellipse.ParStart ? 1 : 0; //it happens because of the y-axis orientation down in SVG
            if (AxesSwapped(ellipse.AxisA, ellipse.AxisB))
            {
                sweepFlagInt = sweepFlagInt == 1 ? 0 : 1;
            }
            var endPoint = this.PointToString(ellipse.End);
            return string.Join(" ", new[] { rx, ry, xAxisRotation, largeArcFlag, sweepFlagInt.ToString(), endPoint });
        }

        private static bool AxesSwapped(Point axisA, Point axisB)
        {
            return axisA.X * axisB.Y - axisB.X * axisA.Y < 0;
        }

        private string CubicBezierSegmentToString(CubicBezierSegment cubic, char previousInstruction)
        {
            var str = this.PointsToString(cubic.B(1), cubic.B(2), cubic.B(3));
            return previousInstruction == 'C' ? str : "C" + str;
        }

        private string PointsToString(params Point[] points)
        {
            return String.Join(" ", points.Select(this.PointToString));
        }

        private string LineSegmentString(LineSegment ls, char previousInstruction)
        {
            var str = this.PointToString(ls.End);
            return previousInstruction == 'L' ? str : "L" + str;
        }

        private void WriteLineSeg(LineSegment ls)
        {
            this.WriteStartElement(GeometryToken.LineSegment);
            this.WriteAttribute(GeometryToken.Points, this.PointsToString(ls.Start, ls.End));
            this.WriteEndElement();
        }

        [SuppressMessage("Microsoft.Globalization", "CA1303:DoNotPassLiteralsAsLocalizedParameters",
            MessageId = "System.Xml.XmlWriter.WriteComment(System.String)")]
        private void WriteTransformation(PlaneTransformation transformation)
        {
            this.WriteStartElement(GeometryToken.Transform);
            this.XmlWriter.WriteComment("the order of elements is [0,0],[0,1],[0,2],[1,0],[1,1],[1,2]");
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 3; j++) {
                    this.WriteTransformationElement(transformation[i, j]);
                }
            }

            this.WriteEndElement();
        }

        private void WriteTransformationElement(double t)
        {
            this.WriteStringElement(GeometryToken.TransformElement, t);
        }


        /// <summary>
        /// writes the starte element with the token
        /// </summary>
        /// <param name="token"></param>
        private void WriteStartElement(GeometryToken token)
        {
            this.XmlWriter.WriteStartElement(FirstCharToLower(token));
        }

        //static  void WriteStartElement(XmlWriter writer, Tokens token) {
        //    writer.WriteStartElement(token.ToString());
        //}

        /// <summary>
        /// WriteStringElement with double
        /// </summary>
        /// <param name="tokens"></param>
        /// <param name="element"></param>
        private void WriteStringElement(GeometryToken tokens, double element)
        {
            this.XmlWriter.WriteElementString(tokens.ToString(), XmlConvert.ToString(element));
        }

        /// <summary>
        /// writes the end element
        /// </summary>
        private void WriteEndElement()
        {
            this.XmlWriter.WriteEndElement();
        }

        /// <summary>
        /// 
        /// </summary>
        private void WriteLayoutSettings()
        {
            if (this.Settings != null)
            {
                LayoutAlgorithmSettings settings = this.Settings;
                EdgeRoutingSettings routingSettings = settings.EdgeRoutingSettings;

                this.WriteStartElement(GeometryToken.LayoutAlgorithmSettings);
                this.WriteAttribute(GeometryToken.EdgeRoutingMode, (int)routingSettings.EdgeRoutingMode);

                var sugiyama = settings as SugiyamaLayoutSettings;
                if (sugiyama != null) {
                    this.WriteSugiyamaSettings(sugiyama);
                } else
                {
                    var mds = settings as MdsLayoutSettings;
                    if (mds != null)
                    {
                        this.WriteAttribute(GeometryToken.LayoutAlgorithmType, GeometryToken.MdsLayoutSettings);
#if TEST_MSAGL
                        this.WriteAttribute(GeometryToken.Reporting, mds.Reporting);
#endif
                        this.WriteAttribute(GeometryToken.Exponent, mds.Exponent);
                        this.WriteAttribute(GeometryToken.IterationsWithMajorization, mds.IterationsWithMajorization);
                        this.WriteAttribute(GeometryToken.PivotNumber, mds.PivotNumber);
                        this.WriteAttribute(GeometryToken.RotationAngle, mds.RotationAngle);
                        this.WriteAttribute(GeometryToken.ScaleX, mds.ScaleX);
                        this.WriteAttribute(GeometryToken.ScaleY, mds.ScaleY);
                    }
                }

                this.XmlWriter.WriteEndElement();
            }
        }

        private void WriteSugiyamaSettings(SugiyamaLayoutSettings sugiyama)
        {
            this.WriteAttribute(GeometryToken.LayoutAlgorithmType, GeometryToken.SugiyamaLayoutSettings);
            this.WriteAttribute(GeometryToken.MinNodeWidth, sugiyama.MinNodeWidth);
            this.WriteAttribute(GeometryToken.MinNodeHeight, sugiyama.MinNodeHeight);
            this.WriteAttribute(GeometryToken.AspectRatio, sugiyama.AspectRatio);
            this.WriteAttribute(GeometryToken.NodeSeparation, sugiyama.NodeSeparation);
#if TEST_MSAGL
            this.WriteAttribute(GeometryToken.Reporting, sugiyama.Reporting);
#endif
            this.WriteAttribute(GeometryToken.RandomSeedForOrdering, sugiyama.RandomSeedForOrdering);
            this.WriteAttribute(GeometryToken.NoGainStepsBound, sugiyama.NoGainAdjacentSwapStepsBound);
            this.WriteAttribute(GeometryToken.MaxNumberOfPassesInOrdering, sugiyama.MaxNumberOfPassesInOrdering);
            this.WriteAttribute(GeometryToken.RepetitionCoefficientForOrdering,
                               sugiyama.RepetitionCoefficientForOrdering);
            this.WriteAttribute(GeometryToken.GroupSplit, sugiyama.GroupSplit);
            this.WriteAttribute(GeometryToken.LabelCornersPreserveCoefficient,
                               sugiyama.LabelCornersPreserveCoefficient);
            this.WriteAttribute(GeometryToken.BrandesThreshold, sugiyama.BrandesThreshold);
            this.WriteAttribute(GeometryToken.LayerSeparation, sugiyama.LayerSeparation);
            this.WriteTransformation(sugiyama.Transformation);
        }
    }
}

#endif