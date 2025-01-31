﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.ProjectionSolver;

namespace Microsoft.Msagl.Miscellaneous.RegularGrid
{
    /// <summary>
    /// Traversal of regular grid
    /// </summary>
    public class GridTraversal
    {
        private readonly Rectangle _boundingBox;
        private readonly int _iLevel;
        private readonly ulong _numberOfTilesOnSide;

        public GridTraversal(Rectangle boundingBox, int iLevel)
        {
            this._boundingBox = boundingBox;
            this._iLevel = iLevel;
            this._numberOfTilesOnSide = (ulong)Math.Pow(2, iLevel);
            this.TileWidth = boundingBox.Width / this._numberOfTilesOnSide;
            this.TileHeight = boundingBox.Height / this._numberOfTilesOnSide;
        }

        public double TileWidth { get; private set; }

        public double TileHeight{ get; private set; }

        public int ILevel {
            get { return this._iLevel; }
        }

        public Tuple<int, int> PointToTuple(Point point)
        {
            var dx = (int)(point.X - this._boundingBox.Left);
            var dy = (int)(point.Y - this._boundingBox.Bottom);
            var ix = (int)(dx / this.TileWidth);
            var iy = (int)(dy / this.TileHeight);
            return new Tuple<int, int>(ix, iy);
        }


        public List<Tuple<int, int>> GetTilesIntersectedByLineSeg(Point p1, Point p2)
        {
            List<Tuple<int, int>> tiles = new List<Tuple<int, int>>();


            var tile1 = this.PointToTuple(p1);
            var tile2 = this.PointToTuple(p2);

            tiles.Add(tile1);
            if (tile1.Equals(tile2))
            {
                return tiles;
            }

            tiles.Add(tile2);
            var midPoint = (p1 + p2) / 2;
            this.InsertTilesLeftHalf(tile1, p1, midPoint, tiles);
            this.InsertTilesRightHalf(midPoint, p2, tile2, tiles);
            return tiles;
        }

        private void InsertTilesLeftHalf(Tuple<int, int> leftTile, Point p1, Point p2, List<Tuple<int, int>> tiles)
        {
            Debug.Assert(this.PointToTuple(p1).Equals(leftTile));
            var rightTile = this.PointToTuple(p2);
            if (rightTile.Equals(leftTile) || ApproximateComparer.Close(p1, p2, ((double)this.TileWidth) / 10)) {
                return;
            }

            tiles.Add(rightTile);
            var midPoint = (p1 + p2) / 2;
            this.InsertTilesLeftHalf(leftTile, p1, midPoint, tiles);
            this.InsertTilesRightHalf(midPoint, p2, rightTile, tiles);
        }

        private void InsertTilesRightHalf(Point p1, Point p2, Tuple<int, int> rightTile, List<Tuple<int, int>> tiles)
        {
            var leftTile = this.PointToTuple(p1);
            if (leftTile.Equals(rightTile) || ApproximateComparer.Close(p1, p2, ((double)this.TileWidth) / 10)) {
                return;
            }

            tiles.Add(leftTile);
            var midPoint = (p1 + p2) / 2;
            this.InsertTilesLeftHalf(leftTile, p1, midPoint, tiles);
            this.InsertTilesRightHalf(midPoint, p2, rightTile, tiles);
        }

        public Rectangle GetTileRect(int ix, int iy)
        {
            var lb = this._boundingBox.LeftBottom + new Point(ix * this.TileWidth, iy * this.TileHeight);
            return new Rectangle(lb, lb + new Point(this.TileWidth, this.TileHeight));
        }

        public List<string> SplitTileNameOnDirectories(int ix, int iy) {
            List<string> ret = new List<string>();
            ulong tileIndex = (ulong) iy* this._numberOfTilesOnSide + (ulong) ix;
            const ulong maxInDirectory = 500; // there will be no more than 500 files in each sub-directory
            do {
                if (tileIndex < maxInDirectory) {
                    ret.Add(tileIndex.ToString());
                    break;
                }
                var k = tileIndex/maxInDirectory;
                var remainder = tileIndex - k*maxInDirectory;
                ret.Add(remainder.ToString());
                tileIndex = k;
            } while (true);

            ret.Add(this.ILevel.ToString());
            return ret;
        }

        public Point GetTileCenter(int ix, int iy) {
            return this._boundingBox.LeftBottom + new Point(ix* this.TileWidth + 0.5* this.TileWidth, iy* this.TileHeight + 0.5* this.TileHeight);

        }
    }
}
