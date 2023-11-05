using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

#pragma warning disable 1591
namespace Microsoft.Msagl.Core.Geometry
{
    /// <summary>
    /// Pack rectangles (without rotation) into a given aspect ratio
    /// </summary>
    public abstract class OptimalPacking<TData> : AlgorithmBase
    {
        private double desiredAspectRatio = 1.2;
        private double bestPackingCost;
        private Packing bestPacking = null;

        protected IList<RectangleToPack<TData>> rectangles;
        private Dictionary<double, double> cachedCosts = new Dictionary<double, double>();

        protected OptimalPacking(IList<RectangleToPack<TData>> rectangles, double aspectRatio)
        {
            this.rectangles = rectangles;
            this.desiredAspectRatio = aspectRatio;
        }

        /// <summary>
        /// The width of the widest row in the packed solution
        /// </summary>
        public double PackedWidth { 
            get
            {
                if (this.bestPacking != null)
                {
                    return this.bestPacking.PackedWidth;
                }
                return 0;
            }
        }

        /// <summary>
        /// The height of the bounding box of the packed solution
        /// </summary>
        public double PackedHeight { 
            get
            {
                if (this.bestPacking != null)
                {
                    return this.bestPacking.PackedHeight;
                }
                return 0;
            }
        }

        // controls the maximum number of steps we are allowed to take in our golden section search
        // (actually worst case is O (n log n) for n=MaxSteps)
        protected const double MaxSteps = 1000;

        protected delegate Packing PackingFactory(IList<RectangleToPack<TData>> rectangles, double limit);

        protected PackingFactory createPacking;

        protected void Pack(double lowerBound, double upperBound, double minGranularity)
        {
            double c0 = GetGoldenSectionStep(lowerBound, upperBound);

            // the worst case time complexity is O(n log(n)) where we have to do a full traversal of the
            // golden section search tree because it each stage the two candidate split points we chose had
            // the same cost.
            // the following calculation for precision limits the worst case time by making max(n) = MaxSteps.
            double precision = Math.Max(minGranularity / 10, (upperBound - lowerBound) / MaxSteps);

            // need to overshoot upperbound in case upperbound is actually optimal
            upperBound += precision;

            this.bestPackingCost = double.MaxValue;

            if (this.rectangles.Count == 1)
            {
                // the trivial solution for just one rectangle is widthLowerBound
                this.Pack(lowerBound);
            }
            else if (this.rectangles.Count == 2)
            {
                // if we have 2 rectangles just try the two possibilities
                this.Pack(lowerBound);
                this.Pack(upperBound);
            }
            else if (this.rectangles.Count > 2)
            {
                GoldenSectionSearch(this.Pack, lowerBound, c0, upperBound, precision);
            }

            // packing works on the rectangles in place, so we need to rerun to get back the best packing.
            this.bestPacking.Run();
        }

        private double Pack(double limit)
        {
            double cost;
            if (!this.cachedCosts.TryGetValue(limit, out cost))
            {
                var packing = this.createPacking(this.rectangles, limit);
                packing.Run();
                this.cachedCosts[limit] = cost = Math.Abs(packing.PackedAspectRatio - this.desiredAspectRatio);
                if (cost < this.bestPackingCost)
                {
                    this.bestPackingCost = cost;
                    this.bestPacking = packing;
                }
            }
            return cost;
        }

        /// <summary>
        /// recursively searches a weakly unimodal function f(x) between x1 and x3 for the minimum.  It is assumed x2 \le x1 and x2 \le x3
        /// and x2-x1=a \lt b=x3-x2.  The recursion generates a fourth point x4-x1=b \gt a=x3-x4 where x4-x2=c and b=a+c and:
        /// if f(x4) \lt f(x2) we search in the range [x2, x3]
        /// else if f(x2) \lt f(x4) we search in the range [x1, x4]
        /// else 
        /// f(x2)==f(x4) and we know that f is only weakly unimodal (not strongly unimodal) and we must search both branches.
        /// </summary>
        internal static double GoldenSectionSearch(Func<double, double> f, double x1, double x2, double x3, double precision)
        {
            Debug.Assert((Math.Abs(x3 - x1) - precision) / precision <= MaxSteps + 0.1, "precision would violate the limit imposed by MaxSteps");

            // check termination
            if (Math.Abs(x1 - x3) < precision)
            {
                return f(x1) < f(x3) ? x1 : x3;
            }

            // x2 must be between x1 and x3
            Debug.Assert(x1 < x2 && x2 < x3 || x3 < x2 && x2 < x1, "x2 not bounded by x1 and x3");

            // x4 will be our new midpoint candidate
            double x4 = GetGoldenSectionStep(x2, x3);
            
            // now we have two candidates (x2,x4) both between x1 and x3: choose the bracket that most reduces f
            double fx2 = f(x2);
            double fx4 = f(x4);

            Func<double> leftSearch = () => GoldenSectionSearch(f, x4, x2, x1, precision);
            Func<double> rightSearch = () => GoldenSectionSearch(f, x2, x4, x3, precision);

            if (fx4 < fx2)
            {
                Debug.Assert(Math.Abs(x2 - x3) < Math.Abs(x1 - x3), "Search region not narrowing!");
                return rightSearch();
            }
            if (fx4 > fx2)
            {
                Debug.Assert(Math.Abs(x4 - x1) < Math.Abs(x1 - x3), "Search region not narrowing!");
                return leftSearch();
            }
            
            // Doh! f(x2) == f(x4)!  Have to search both branches.
            double right = rightSearch();
            double left = leftSearch();
            return f(left) < f(right) ? left : right;
        }

        private static double GetGoldenSectionStep(double x1, double x2)
        {
            if (x1 < x2)
            {
                return x1 + PackingConstants.GoldenRatioRemainder * (x2 - x1);
            }
            return x1 - PackingConstants.GoldenRatioRemainder * (x1 - x2);
        }
    }
}
