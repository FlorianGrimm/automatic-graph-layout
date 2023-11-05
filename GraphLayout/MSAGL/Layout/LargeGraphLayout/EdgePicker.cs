using Microsoft.Msagl.Core.Layout;

namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    internal class EdgePicker {
        private readonly LgData lgData;
        private readonly IZoomLevelCalculator nodeZoomLevelCalculator;
        private readonly int _nodeCountOnLevel;
        internal static void SetEdgeInfosZoomLevelsAndIcreaseRanks(LgData lgData,
                                                    IZoomLevelCalculator nodeZoomLevelCalculator,
                                                    int nodeCountOnLevel) {
            var edgePicker = new EdgePicker(lgData, nodeZoomLevelCalculator, nodeCountOnLevel);
            edgePicker.Run();
        }

        private void Run() {
            if (this._nodeCountOnLevel == this.lgData.GeometryNodesToLgNodeInfos.Count) {
                foreach (var e in this.lgData.GeometryEdgesToLgEdgeInfos.Keys) {
                    this.UpdateEdgeInfoZoomLevel(e);
                }

                return;
            }

            this.FillShortRoutes();
        }

        private readonly int zoomLevel;

        private EdgePicker(LgData lgData, IZoomLevelCalculator nodeZoomLevelCalculator,
                   int nodeCountOnLevel) {
            this.lgData = lgData;
            this.nodeZoomLevelCalculator = nodeZoomLevelCalculator;
            this._nodeCountOnLevel = nodeCountOnLevel;
            this.zoomLevel = (int) nodeZoomLevelCalculator.SortedLgNodeInfos[nodeCountOnLevel - 1].ZoomLevel;
        }

        private void FillShortRoutes() {
            //creating one edge long routes between the nodes of the level
             for (int i = 0; i < this._nodeCountOnLevel; i++) {
                this.FillShortRoutesOfNode(this.nodeZoomLevelCalculator.SortedLgNodeInfos[i].GeometryNode);
            }
        }

        private void FillShortRoutesOfNode(Node node) {
            foreach (var e in node.OutEdges) {
                this.TryPickingEdge(e);
            }
        }

        private void TryPickingEdge(Edge edge) {
            if (this.lgData.GeometryNodesToLgNodeInfos[edge.Target].ZoomLevel <= this.zoomLevel) {
                //connected to a node from the same level
                this.UpdateEdgeInfoZoomLevel(edge);
            }
        }

        private void UpdateEdgeInfoZoomLevel(Edge edge) {
            var sourceNodeInfo = this.lgData.GeometryNodesToLgNodeInfos[edge.Source];
            var targetNodeInfo = this.lgData.GeometryNodesToLgNodeInfos[edge.Target];
            this.UpdateEdgeInfoZoomLevel(edge, sourceNodeInfo.Rank + targetNodeInfo.Rank);
        }

        private void UpdateEdgeInfoZoomLevel(Edge edge, double edgeRank) {
            var edgeInfo = this.lgData.GeometryEdgesToLgEdgeInfos[edge];
            this.TryToDecreaseZoomLevel(edgeInfo);
            TryToIncreaseRank(edgeInfo, edgeRank);
        }

        private static void TryToIncreaseRank(LgEdgeInfo edgeInfo, double edgeRank) {
            if (edgeInfo.Rank < edgeRank) {
                edgeInfo.Rank = edgeRank;
            }
        }

        private void TryToDecreaseZoomLevel(LgEdgeInfo edgeInfo) {
            if (edgeInfo.ZoomLevel > this.zoomLevel) {
                edgeInfo.ZoomLevel = this.zoomLevel;
            }
        }
    }
}