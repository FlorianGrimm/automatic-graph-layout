using System;

namespace Microsoft.Msagl.GraphmapsWithMesh
{

    public class PointSet
    {
        public int NumPoints;
        public WeightedPoint[] Pt;
        public int NumOfLevels;
        private int[] _selected;

        public int[,] pointMap;



        public PointSet(int n)
        {
            this.NumPoints = n;
            this.Pt = new WeightedPoint[n];
            Random r1 = new Random(2);


            //generate points in random
            while (n > 0)
            {
                int XLoc = r1.Next(100);
                int YLoc = r1.Next(100);
                if (this.exists(XLoc, YLoc, n)) {
                    continue;
                }

                n--;
                this.Pt[n] = new WeightedPoint(XLoc, YLoc, 0);
            }
            for (int i = 0; i < this.NumPoints; i++)
            {
                this.Pt[i].X *= 3;
                this.Pt[i].Y *= 3;
            }

        }

        public bool isVeryClose(int x, int y, int n)
        {
            double d;
            for (int i = this.NumPoints - 1; i > n; i--)
            {
                d = Math.Sqrt((this.Pt[i].X - x) * (this.Pt[i].X - x) + (this.Pt[i].Y - y) * (this.Pt[i].Y - y));
                if (d <= 3) {
                    return true;
                }
            }
            return false;
        }


        public bool exists(int x, int y, int n)
        {
            for (int i = this.NumPoints - 1; i > n; i--)
            {
                if (this.Pt[i].X == x && this.Pt[i].Y == y) {
                    return true;
                }
            }
            return false;
        }
        public PointSet(int n, Tiling g, int pointsPerLevel)
        {
            this.NumPoints = n;
            this.Pt = new WeightedPoint[n + 1];
#if SHARPKIT //https://code.google.com/p/sharpkit/issues/detail?id=340
            throw new InvalidOperationException();
#else
            this.pointMap = new int[g.NumOfnodes + 1, g.NumOfnodes + 1];
#endif
            this._selected = new int[g.NumOfnodes + 1];
            Random r1 = new Random(1);


            //generate points in random
            while (n > 0)
            {
                var temp = r1.Next(1, g.NumOfnodes); //,temp2;
                if (this._selected[temp] == 1 || (g.VList[temp].XLoc + g.VList[temp].YLoc) % 2 == 1) {
                    continue;
                }

                this.Pt[n] = new WeightedPoint(g.VList[temp].XLoc, g.VList[temp].YLoc, 0);
                this._selected[temp] = 1;
                //compute weight based on point density
                this.Pt[n].GridPoint = temp;
                g.VList[temp].Weight = this.Pt[n].Weight;
                n--;
            }
            this.AssignWeight(this.Pt, this.NumPoints, (int)Math.Sqrt(g.NumOfnodes));


            this.NumOfLevels = this.NumPoints / pointsPerLevel;
            if (this.NumPoints % pointsPerLevel > 0) {
                this.NumOfLevels++;
            }

            for (int index = 1; index <= this.NumPoints; index++)
            {
                this.Pt[index].ZoomLevel = 1 + (index - 1) / pointsPerLevel;
            }

            for (int i = 1; i <= this.NumPoints; i++)
            {
                g.VList[this.Pt[i].GridPoint].Weight = this.Pt[i].Weight;
                g.VList[this.Pt[i].GridPoint].ZoomLevel = this.Pt[i].ZoomLevel;
                this.pointMap[this.Pt[i].X, this.Pt[i].Y] = i;

            }
        }
        public void AssignWeight(WeightedPoint[] pt, int numPoints, int rad)
        {
            for (int i = 1; i <= numPoints; i++)
            {
                pt[i].Weight++;
                for (int j = i + 1; j <= numPoints; j++)
                {
                    var temp = Math.Sqrt((pt[i].X - pt[j].X) * (pt[i].X - pt[j].X) + (pt[i].Y - pt[j].Y) * (pt[i].Y - pt[j].Y));
                    if (temp < rad) { pt[i].Weight++; pt[j].Weight++; }
                }
            }

            double tempMin = numPoints;
            double tempMax = 0;
            for (int i = 1; i <= numPoints; i++)
            {
                if (tempMax < pt[i].Weight) {
                    tempMax = pt[i].Weight;
                }

                if (tempMin > pt[i].Weight) {
                    tempMin = pt[i].Weight;
                }
            }
            for (int i = 1; i <= numPoints; i++) {
                pt[i].Weight = 50 + (int)((pt[i].Weight - tempMin) * 200 / (tempMax - tempMin));
            }
        }


    }

    public class WeightedPoint
    {
        public int X;
        public int Y;
        public int Weight;
        public int GridPoint; //id of the grid point
        public int ZoomLevel;
        public WeightedPoint() { }
        public WeightedPoint(int a, int b, int c)
        {
            this.X = a; this.Y = b; this.Weight = c;
        }
    }
}
