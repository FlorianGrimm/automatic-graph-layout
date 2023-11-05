using System;
using System.Diagnostics;

namespace Microsoft.Msagl.Layout.OverlapRemovalFixedSegments {
    internal class FakeBitmap {
        private readonly int _width;
        private readonly int _height;
        private readonly byte[] _m;

        public FakeBitmap(int width, int height) {
            this._width = width;
            this._height = height;
            int size = this._width * this._height;
            this._m = new byte[size];
        }


        public byte GetPixelValue(int x, int y) {
            if (!this.InBounds(x, y)) {
                return 0;
            }

            return this._m[x + y* this._width];
        }

        private int Offset(int x, int y) {
            Debug.Assert(this.InBounds(x, y));
            return x + this._width *y;
        }

        private bool InBounds(int x, int y) {
            return 0 <= x && x < this._width &&
                   0 <= y && y < this._height;
        }

        public void FillRectangle(int left, int bottom, int w, int h) {
            this.AdjustLeftTop(ref left, ref bottom);
            int top = Math.Min(bottom + h, this._height - 1);
            int right = Math.Min(left + w, this._width - 1);
            for (int y = bottom; y < top; y++) {
                int offset = this.Offset(left, y);
                for (int x = left; x < right; x++) {
                    this._m[offset++] = 1;
                }
            }
        }

        private void AdjustLeftTop(ref int left, ref int bottom) {
            if (left < 0) {
                left = 0;
            }

            if (bottom < 0) {
                bottom = 0;
            }

            if (left >= this._width) {
                left = this._width - 1;
            }

            if (bottom >= this._height) {
                bottom = this._height - 1;
            }
        }

        private void Plot(int x, int y) {
            this._m[this.Offset(x, y)] = 1;
        }

        internal void DrawFatSegByX(PixelPoint a, PixelPoint b, int insertRectHalfSize) {
            var perp = new PixelPoint(-(b.Y - a.Y), b.X - a.X);
            if (perp.Y == 0) {
                this.FillRectangle(a.X, a.Y - insertRectHalfSize, b.X - a.X, 2*insertRectHalfSize);
                return;
            }
            if (perp.Y < 0) {
                perp = -perp;
            }

            var pixelInside = a;
            double perpLen = Math.Sqrt(perp*perp);
            // pixel p is inside if and only if (p-a)*perp belongs to the interval [-halfInterval, halfInterval]
            int halfInterval = (int) (perpLen*insertRectHalfSize);
            for (int x = a.X; x <= b.X; x++) {
                this.ScanVertLine(x, ref pixelInside, a, b, perp, halfInterval);
            }
        }

        private void ScanVertLine(int x, ref PixelPoint pixelInside, PixelPoint a, PixelPoint b, PixelPoint perp,
            int halfInterval) {
            this.ScanVertLineUp(x, pixelInside, a, perp, halfInterval);
            this.ScanVertLineDown(x, pixelInside, a, perp, halfInterval);
            UpdatePixelInsideForXCase(ref pixelInside, a, b, perp, halfInterval, Math.Sign(b.Y - a.Y));
        }

        private void ScanVertLineDown(int x, PixelPoint pixelInside, PixelPoint a, PixelPoint perp, int halfInterval) {
            var y = pixelInside.Y;
            var proj = perp*(pixelInside - a);
            Debug.Assert(Math.Abs(proj) <= halfInterval);
            do {
                this.Plot(x, y);
                proj -= perp.Y;
                if (proj < - halfInterval) {
                    break;
                }

                y--;
            } while (y >= 0);
        }

        private void ScanVertLineUp(int x, PixelPoint pixelInside, PixelPoint a, PixelPoint perp, int halfInterval) {
            Debug.Assert(perp.Y > 0);
            var y = pixelInside.Y;
            var proj = perp*(pixelInside - a);
            Debug.Assert(Math.Abs(proj) <= halfInterval);
            do {
                this.Plot(x, y);
                proj += perp.Y;
                if (proj > halfInterval) {
                    break;
                }

                y++;
            } while (y < this._height);

        }

        private static void UpdatePixelInsideForXCase(ref PixelPoint pixelInside, PixelPoint a, PixelPoint b, PixelPoint perp,
            int halfInterval, int sign) {
            if (pixelInside.X == b.X) {
                return;
            }

            pixelInside.X++;
            int proj = perp*(pixelInside - a);
            if (Math.Abs(proj) <= halfInterval) {
                return;
            }

            pixelInside.Y += sign;
            proj += sign*perp.Y;
            if (Math.Abs(proj) <= halfInterval) {
                return;
            }

            pixelInside.Y -= 2*sign;
            proj -= 2*sign*perp.Y;
            Debug.Assert(Math.Abs(proj - 2*sign*perp.Y) <= halfInterval);
        }

        internal void DrawFatSegByY(PixelPoint a, PixelPoint b, int halfDistInPixels) {
            Debug.Assert(a.Y <= b.Y);
            var perp = new PixelPoint(-b.Y + a.Y, b.X - a.X);
            if (perp.X == 0) {
                this.FillRectangle(a.X - halfDistInPixels, a.Y, 2*halfDistInPixels, b.Y - a.Y);
                return;
            }
            if (perp.X < 0) {
                perp = -perp;
            }

            var pixelInside = a;
            double perpLen = Math.Sqrt(perp*perp);
            // pixel p is inside if and only if (p-a)*perp belongs to the interval [-halfInterval, halfInterval]
            int halfInterval = (int) (perpLen*halfDistInPixels);
            for (int y = a.Y; y <= b.Y; y++) {
                this.ScanHorizontalLine(y, ref pixelInside, a, b, perp, halfInterval);
            }
        }

        private void ScanHorizontalLine(int x, ref PixelPoint pixelInside, PixelPoint a, PixelPoint b, PixelPoint perp,
            int halfInterval) {
            this.ScanHorizontalLineRight(x, pixelInside, a, perp, halfInterval);
            this.ScanHorizontalLineLeft(x, pixelInside, a, perp, halfInterval);
            this.UpdatePixelInsideForYCase(ref pixelInside, a, b, perp, halfInterval, Math.Sign(b.Y - a.Y));

        }

        private void UpdatePixelInsideForYCase(ref PixelPoint pixelInside, PixelPoint a, PixelPoint b, PixelPoint perp,
            int halfInterval, int sign) {
            if (pixelInside.Y == b.Y) {
                return;
            }

            pixelInside.Y++;
            int proj = perp*(pixelInside - a);
            if (Math.Abs(proj) <= halfInterval) {
                return;
            }

            pixelInside.X += sign;
            proj += sign*perp.X;
            if (Math.Abs(proj) <= halfInterval) {
                return;
            }

            pixelInside.X -= 2*sign;
            proj -= 2*sign*perp.X;
            Debug.Assert(Math.Abs(proj) <= halfInterval);
        }

        private void ScanHorizontalLineRight(int y, PixelPoint pixelInside, PixelPoint a, PixelPoint perp, int halfInterval) {
            Debug.Assert(perp.X > 0);
            var x = pixelInside.X;
            var proj = perp*(pixelInside - a);
            Debug.Assert(Math.Abs(proj) <= halfInterval);
            do {
                this.Plot(x, y);
                proj += perp.X;
                if (proj > halfInterval) {
                    break;
                }

                x++;
            } while (x < this._width);
        }

        private void ScanHorizontalLineLeft(int y, PixelPoint pixelInside, PixelPoint a, PixelPoint perp, int halfInterval) {
            Debug.Assert(perp.X > 0);
            var x = pixelInside.X;
            var proj = perp*(pixelInside - a);
            Debug.Assert(Math.Abs(proj) <= halfInterval);
            do {
                this.Plot(x, y);
                proj -= perp.X;
                if (proj < - halfInterval) {
                    break;
                }

                x--;
            } while (x >= 0);

        }

        internal void DrawFatSeg(PixelPoint a, PixelPoint b, int insertRectHalfSizeInPixels) {
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            this.FillRectangle(a.X - insertRectHalfSizeInPixels, a.Y - insertRectHalfSizeInPixels,
                2*insertRectHalfSizeInPixels,
                2*insertRectHalfSizeInPixels);
            this.FillRectangle(b.X - insertRectHalfSizeInPixels, b.Y - insertRectHalfSizeInPixels,
                2*insertRectHalfSizeInPixels,
                2*insertRectHalfSizeInPixels);
            if (Math.Abs(dx) > Math.Abs(dy)) {
                if (dx > 0) {
                    this.DrawFatSegByX(a, b, insertRectHalfSizeInPixels);
                } else {
                    this.DrawFatSegByX(b, a, insertRectHalfSizeInPixels);
                }
            }
            else {
                if (dy > 0) {
                    this.DrawFatSegByY(a, b, insertRectHalfSizeInPixels);
                } else {
                    this.DrawFatSegByY(b, a, insertRectHalfSizeInPixels);
                }
            }
        }
    }
}