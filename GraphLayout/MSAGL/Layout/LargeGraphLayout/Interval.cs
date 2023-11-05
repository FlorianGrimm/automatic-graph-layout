using System;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Layout.LargeGraphLayout {
    /// <summary>
    /// represents a range of doubles
    /// </summary>
    public class Interval:IRectangle<double> {
        /// <summary>
        /// constructor
        /// </summary>
        /// <param name="start"></param>
        /// <param name="end"></param>
        public Interval(double start, double end) {
            this.Start = start;
            this.End = end;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        public Interval(Interval a, Interval b)
        {
            this.Start = a.Start;
            this.End = a.End;
            this.Add(b.Start);
            this.Add(b.End);
        }



        /// <summary>
        /// expanding the range to hold v
        /// </summary>
        /// <param name="v"></param>
        public void Add(double v) {
            if (this.Start > v) {
                this.Start = v;
            }

            if (this.End < v) {
                this.End = v;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        public double Start { get; set; }
        /// <summary>
        /// 
        /// </summary>
        public double End { get; set; }

        /// <summary>
        /// the length
        /// </summary>
        public double Area { get { return this.End - this.Start; } }
        /// <summary>
        /// return true if the value is inside the range
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool Contains(double v) {
            return this.Start <= v && v <= this.End;
        }

        /// <summary>
        /// bringe v into the range
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public double GetInRange(double v) {
            return v < this.Start ? this.Start : (v > this.End ? this.End : v);
        }

        /// <summary>
        /// returns true if and only if two intervals are intersecting
        /// </summary>
        /// <param name="other"></param>
        /// <returns></returns>
        public bool Intersects(Interval other) {
            if (other.Start > this.End + ApproximateComparer.DistanceEpsilon) {
                return false;
            }

            return !(other.End < this.Start - ApproximateComparer.DistanceEpsilon);
        }

        public bool Contains(IRectangle<double> rect) {
            var r = (Interval)rect;
            return this.Contains(r);
        }

        public IRectangle<double> Intersection(IRectangle<double> rectangle) {
            var r = (Interval)rectangle;
            return new Interval(Math.Max(this.Start, r.Start), Math.Min(this.End, r.End));
        }

        public bool Intersects(IRectangle<double> rectangle) {
            var r = (Interval)rectangle;
            return this.Intersects(r);
        }

        public void Add(IRectangle<double> rectangle) {
            var r = (Interval)rectangle;
            this.Add(r.Start);
            this.Add(r.End);
        }

        public bool Contains(double p, double radius) {
            return this.Contains(p - radius) && this.Contains(p + radius);
        }

        public IRectangle<double> Unite(IRectangle<double> rectangle) {
            return new Interval(this, (Interval)rectangle);
        }
    }
}