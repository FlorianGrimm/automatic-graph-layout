using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;
using Point = Microsoft.Msagl.Core.Geometry.Point;

using Rectangle = Microsoft.Msagl.Core.Geometry.Rectangle;
using Microsoft.Msagl.Core.Geometry.Curves;
using SymmetricSegment = Microsoft.Msagl.Core.DataStructures.SymmetricTuple<Microsoft.Msagl.Core.Geometry.Point>;


namespace Microsoft.Msagl.Layout.OverlapRemovalFixedSegments {
    public class OverlapRemovalFixedSegmentsBitmap {
        private Rectangle[] _moveableRectangles;
        private Rectangle[] _fixedRectangles;
        private SymmetricSegment[] _fixedSegments;
        private Rectangle[] movedRectangles;
        private FakeBitmap _bitmap;
        private const int MaxSize = 1024*2; //512
        private int _mapWidth, _mapHeight;
        private double _pixelSize;
        private Rectangle _bbox;
        private PlaneTransformation _worldToMap;
        private PlaneTransformation _mapToWorld;
        private double _insertRectSize;
        
        internal int NumPixelsPadding = 5;
        
        public void InitInsertRectSize(Rectangle box) {
            this._insertRectSize = box.Width;
            this._insertRectSize += this.NumPixelsPadding * this._pixelSize;
        }

        public OverlapRemovalFixedSegmentsBitmap(Rectangle[] moveableRectangles, Rectangle[] fixedRectangles,
            SymmetricSegment[] fixedSegments) {
            this._moveableRectangles = moveableRectangles;
            this._fixedRectangles = fixedRectangles;
            this._fixedSegments = fixedSegments;

            this._bbox = this.GetInitialBoundingBox();
            this._bbox.ScaleAroundCenter(1.25);

            this.InitBitmap();
            this.InitTransform();

            this.movedRectangles = new Rectangle[moveableRectangles.Length];
        }

        public void ScaleBbox(double scale) {
            this._bbox.ScaleAroundCenter(1.25);
            this.InitBitmap();
            this.InitTransform();
        }

        public OverlapRemovalFixedSegmentsBitmap(Rectangle bbox) {
            this._bbox = bbox;
            this.InitBitmap();
            this.InitTransform();
        }

        public Rectangle GetInitialBoundingBox() {
            Rectangle bbox = new Rectangle();
            bbox.SetToEmpty();
            foreach (var rect in this._fixedRectangles) {
                bbox.Add(rect);
            }
            foreach (var rect in this._moveableRectangles) {
                bbox.Add(rect);
            }
            return bbox;
        }

        public void InitBitmap() {
            var maxDim = Math.Max(this._bbox.Width, this._bbox.Height);
            this._mapWidth = (int) (this._bbox.Width/maxDim*MaxSize);
            this._mapHeight = (int) (this._bbox.Height/maxDim*MaxSize);

            this._pixelSize = this._bbox.Width/ this._mapWidth;
            this._bitmap = new FakeBitmap(this._mapWidth, this._mapHeight);
        }

        internal bool FindClosestFreePixel(Point p, out PixelPoint found) {
            var pPixel = this.PointToPixel(p);

            for (int r = 0; r < MaxSize/2; r++) {
                if (!this.GetFreePixelInRadius(ref pPixel, r)) {
                    continue;
                }

                found = pPixel;
                return true;
            }

            found = new PixelPoint();
            return false;
        }

        public bool IsPositionFree(Point p) {
            return this.IsFree(this.PointToPixel(p));
        }

        public int PositionAllMoveableRectsSameSize(int startInd, RTree<Rectangle, Point> fixedRectanglesTree,
            RTree<SymmetricSegment, Point> fixedSegmentsTree) {
            int i;
            if (this._moveableRectangles.Length == 0) {
                return 0;
            }

            this.InitInsertRectSize(this._moveableRectangles[startInd]);

            this.DrawFixedRectsSegments(fixedRectanglesTree, fixedSegmentsTree);

            for (i = startInd; i < this._moveableRectangles.Length; i++) {
                var rect = this._moveableRectangles[i];

                bool couldInsert;

                if (this.IsPositionFree(rect.Center)) {
                    this.movedRectangles[i] = rect;
                    couldInsert = true;
                }
                else {
                    Point newPos;
                    couldInsert = this.FindClosestFreePos(rect.Center, out newPos);
                    this.movedRectangles[i] = Translate(rect, newPos - rect.Center);
                }

                if (!couldInsert) {
                    return i;
                }

                fixedRectanglesTree.Add(this.movedRectangles[i], this.movedRectangles[i]);
                this.DrawRectDilated(this.movedRectangles[i]);
            }
            return this._moveableRectangles.Length;
        }

        private void DrawFixedRectsSegments(RTree<Rectangle, Point> fixedRectanglesTree,
            RTree<SymmetricSegment, Point> fixedSegmentsTree) {
            foreach (var fr in fixedRectanglesTree.GetAllLeaves()) {
                this.DrawRectDilated(fr);
            }

            foreach (var seg in fixedSegmentsTree.GetAllLeaves()) {
                this.DrawLineSegDilated(seg.A, seg.B);
            }
        }
        
        public bool FindClosestFreePos(Point p, out Point newPos) {
            PixelPoint iPos;
            bool found = this.FindClosestFreePixel(p, out iPos);
            newPos = this.GetWorldCoord(iPos);
            return found;
        }

        public Point[] GetTranslations() {
            Point[] translation = new Point[this._moveableRectangles.Length];

            for (int i = 0; i < this._moveableRectangles.Length; i++) {
                translation[i] = this.movedRectangles[i].Center - this._moveableRectangles[i].Center;
            }

            return translation;
        }

        private static Rectangle Translate(Rectangle rect, Point delta) {
            return new Rectangle(rect.LeftBottom + delta, rect.RightTop + delta);
        }

        private bool GetFreePixelInRadius(ref PixelPoint source, int rad) {
            for (int x = source.X - rad; x <= source.X + rad; x++) {
                PixelPoint p1 = new PixelPoint(x, source.Y - rad);
                if (this.IsFree(p1)) {
                    source = p1;
                    return true;
                }
                PixelPoint p2 = new PixelPoint(x, source.Y + rad);
                if (rad != 0 && this.IsFree(p2)) {
                    source = p2;
                    return true;
                }
            }
            for (int y = source.Y - rad + 1; y < source.Y + rad; y++) {
                PixelPoint p1 = new PixelPoint(source.X - rad, y);
                if (this.IsFree(p1)) {
                    source = p1;
                    return true;
                }
                PixelPoint p2 = new PixelPoint(source.X + rad, y);
                if (this.IsFree(p2)) {
                    source = p2;
                    return true;
                }
            }
            return false;
        }

        private bool IsFree(PixelPoint p) {
            return this._bitmap.GetPixelValue(p.X, p.Y) == 0;
        }

        private void DrawRectDilated(Rectangle rect) {
            Point d = new Point(this._insertRectSize /2, this._insertRectSize /2);
            var leftBottom = this._worldToMap *(rect.LeftBottom - d);
            var rightTop = this._worldToMap *(rect.RightTop + d);
            int ix1 = (int) leftBottom.X;
            int iy1 = (int) leftBottom.Y;
            int iw = (int) (rightTop.X - leftBottom.X);
            int ih = (int) (rightTop.Y - leftBottom.Y);
            this._bitmap.FillRectangle(ix1, iy1, iw, ih);
        }

        private void DrawLineSegDilated(Point p1, Point p2) {
            var a = this.PointToPixel(p1);
            var b = this.PointToPixel(p2);
            this._bitmap.DrawFatSeg(a, b, (int) (this._insertRectSize /(2* this._pixelSize)+0.5));
        }


        internal PixelPoint PointToPixel(Point p) {
            var l = this._worldToMap *p;
            return new PixelPoint((int) (l.X + 0.5), (int) (l.Y + 0.5));
        }

        internal Point GetWorldCoord(PixelPoint p) {
            return this._mapToWorld *new Point(p.X, p.Y);
        }

        public void InitTransform() {
            double mapCenterX = 0.5* this._mapWidth;
            double mapCenterY = 0.5* this._mapHeight;
            double worldCenterX = this._bbox.Center.X;
            double worldCenterY = this._bbox.Center.Y;

            double scaleX = this._mapWidth / this._bbox.Width;
            double scaleY = this._mapHeight / this._bbox.Height;
            this._worldToMap = new PlaneTransformation(scaleX, 0, mapCenterX - scaleX*worldCenterX,
                0, scaleY, mapCenterY - scaleY*worldCenterY);

            this._mapToWorld = this._worldToMap.Inverse;
        }

    }
}
