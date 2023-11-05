using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Layout.LargeGraphLayout
{
    public class LgCellInfo
    {
        public Rectangle CellRectangle;
        public double ZoomLevel;
        public int NumberNodesLeqItsLevelInside = 0;
        public int MaxNumberNodesPerTile = 0;

        public override string ToString()
        {
            return "ZoomLevel: " + this.ZoomLevel + "\nnodes of ZoomLevel <= " + this.ZoomLevel + " inside: " + this.NumberNodesLeqItsLevelInside + "/" + this.MaxNumberNodesPerTile;
        }
    }
}
