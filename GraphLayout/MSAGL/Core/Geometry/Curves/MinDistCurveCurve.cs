using System;

namespace Microsoft.Msagl.Core.Geometry.Curves {
    //For curves A(s) and B(t) with some guess for the parameters (s0, t0) we 
    //are trying to bring to (0,0) the vector(Fs,Ft).
    //where F(s,t) = (A(s) - B(t))^2.  To minimize F^2,
    //you get the system of equations to solve for ds and dt: 
    //Fs + Fss*ds + Fst*dt = 0
    //Ft + Fst*ds + Ftt*dt = 0
    // 
    //Where F = F(si,ti), Fs and Ft are the first partials at si, ti, Fxx are the second partials, 
    //    and s(i+1) = si+ds, t(i+1) = ti+dt. 
    //Of course you have to make sure that ds and dt do not take you out of your domain. 
    //This will converge if the curves have 2nd order continuity 
    //and we are starting parameters are reasonable.  
    //It is not a good method for situations that are not well behaved, but it is really simple.
/// <summary>
/// Implements the minimal distance between curves functionality
/// </summary>
#if TEST_MSAGL
    [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Naming", "CA1704:IdentifiersShouldBeSpelledCorrectly", MessageId = "Dist")]
    public 
#else
    internal
#endif
        class MinDistCurveCurve {
        private ICurve curveA;
        private ICurve curveB;
        private double aMin;
        private double aMax;
        private double bMin;
        private double bMax;
        private double aGuess;
        private double bGuess;
        private double aSolution;
        internal double ASolution {
            get { return this.aSolution; }
        }

        private double bSolution;
        internal double BSolution {
            get { return this.bSolution; }
        }

        private Point aPoint;
        internal Point APoint { get { return this.aPoint; } }

        private Point bPoint;
        internal Point BPoint { get { return this.bPoint; } }

        private bool status;
        internal bool Status { get { return this.status; } }

        private double si;
        private double ti;
        private Point a, b, a_b, ad, bd, add, bdd;

        private void InitValues() {
            this.a = this.curveA[this.si];
            this.b = this.curveB[this.ti];
            this.a_b = this.a - this.b;
            this.ad = this.curveA.Derivative(this.si);
            this.add = this.curveA.SecondDerivative(this.si);
            this.bd = this.curveB.Derivative(this.ti);
            this.bdd = this.curveB.SecondDerivative(this.ti);
        }
/// <summary>
/// constructor
/// </summary>
/// <param name="curveAPar">first curve</param>
/// <param name="curveBPar">second curve</param>
/// <param name="lowBound0">the first curve minimal parameter</param>
        /// <param name="upperBound0">the first curve maximal parameter</param>
        /// <param name="lowBound1">the second curve minimal parameter</param>
        /// <param name="upperBound1">the first curve maximal parameter</param>
/// <param name="guess0"></param>
/// <param name="guess1"></param>
        public MinDistCurveCurve(ICurve curveAPar,
                ICurve curveBPar,
                double lowBound0,
                double upperBound0, 
                double lowBound1,
                double upperBound1,
                double guess0,
                double guess1) {
            this.curveA = curveAPar;
            this.curveB = curveBPar;
            this.aMin = lowBound0;
            this.bMin = lowBound1;
            this.aMax = upperBound0;
            this.bMax = upperBound1;
            this.aGuess = guess0;
            this.bGuess = guess1;
            this.si = guess0;
            this.ti = guess1;
        }


        //we ignore the mulitplier 2 here fore efficiency reasons
        private double Fs {
            get {
                return /*2**/this.a_b * this.ad;
            }
        }

        private double Fss {
            get {
                return /*2**/(this.a_b * this.add + this.ad * this.ad);
            }
        }

        private double Fst //equals to Fts
        {
            get {
                return - /*2**/this.bd * this.ad;
            }
        }

        private double Ftt {
            get {
                return /*2**/(-this.a_b * this.bdd + this.bd * this.bd);
            }
        }

        private double Ft {
            get {
                return -/*2**/this.a_b * this.bd;
            }
        }



        /// <summary>
        /// xy - the first row
        /// uw - the second row
        /// </summary>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="u"></param>
        /// <param name="w"></param>
        internal static double Delta(double x, double y, double u, double w) {
            return x * w - u * y;
        }
        //Fs + Fss*ds + Fst*dt = 0
        //Ft + Fst*ds + Ftt*dt = 0
        internal void Solve() {

            int numberOfBoundaryCrossings = 0;
            const int maxNumberOfBoundaryCrossings = 10;
            int numberOfTotalReps = 0;
            const int maxNumberOfTotalReps = 100;

            bool abort = false;

            this.InitValues();

            if (this.curveA is LineSegment && this.curveB is LineSegment ) {
                Point bd1 = this.curveB.Derivative(0);
                bd1 /= bd1.Length;
                Point an = (this.curveA as LineSegment).Normal;

                double del = Math.Abs(an * bd1);


                if (Math.Abs(del) < ApproximateComparer.DistanceEpsilon || Delta(this.Fss, this.Fst, this.Fst, this.Ftt) < ApproximateComparer.Tolerance) {
                    this.status = true;
                    this.ParallelLineSegLineSegMinDist();
                    return;
                }
            }

            double ds, dt;
            do {

                //hopefully it will be inlined by the compiler
                double delta = Delta(this.Fss, this.Fst, this.Fst, this.Ftt);
                if (Math.Abs(delta) < ApproximateComparer.Tolerance) {
                    this.status = false;
                    abort = true;
                    break;
                }

                ds = Delta(-this.Fs, this.Fst, -this.Ft, this.Ftt) / delta;
                dt = Delta(this.Fss, -this.Fs, this.Fst, -this.Ft) / delta;


                double nsi = this.si + ds;
                double nti = this.ti + dt;

                bool bc;

                if (nsi > this.aMax + ApproximateComparer.DistanceEpsilon || nsi < this.aMin - ApproximateComparer.DistanceEpsilon || nti > this.bMax + ApproximateComparer.DistanceEpsilon || nti < this.bMin - ApproximateComparer.DistanceEpsilon) {
                    numberOfBoundaryCrossings++;
                    this.ChopDsDt(ref ds, ref dt);
                    this.si += ds;
                    this.ti += dt;
                    bc = true;
                } else {
                    bc = false;
                    this.si = nsi;
                    this.ti = nti;
                    if (this.si > this.aMax) {
                        this.si = this.aMax;
                    } else if (this.si < this.aMin) {
                        this.si = this.aMin;
                    }

                    if (this.ti > this.bMax) {
                        this.ti = this.bMax;
                    } else if (this.ti < this.bMin) {
                        this.ti = this.bMin;
                    }
                }

                this.InitValues();

                numberOfTotalReps++;

                abort = numberOfBoundaryCrossings >= maxNumberOfBoundaryCrossings ||
                  numberOfTotalReps >= maxNumberOfTotalReps || (ds == 0 && dt == 0 && bc);

            } while ((Math.Abs(ds) >= ApproximateComparer.Tolerance || Math.Abs(dt) >= ApproximateComparer.Tolerance) && !abort);

            if (abort) {
                //may be the initial values were just OK
                Point t = this.curveA[this.aGuess] - this.curveB[this.bGuess];
                if (t * t < ApproximateComparer.DistanceEpsilon * ApproximateComparer.DistanceEpsilon) {
                    this.aSolution = this.aGuess;
                    this.bSolution = this.bGuess;
                    this.aPoint = this.curveA[this.aGuess];
                    this.bPoint = this.curveB[this.bGuess];
                    this.status = true;
                    return;

                }
            }


            this.aSolution = this.si;
            this.bSolution = this.ti;
            this.aPoint = this.a;
            this.bPoint = this.b;
            this.status = !abort;

        }

        private void ChopDsDt(ref double ds, ref double dt) {
            if (ds != 0 && dt != 0) {
                double k1 = 1; //we are looking for a chopped vector of the form k(ds,dt)

                if (this.si + ds > this.aMax)  //we have si+k*ds=aMax           
{
                    k1 = (this.aMax - this.si) / ds;
                } else if (this.si + ds < this.aMin) {
                    k1 = (this.aMin - this.si) / ds;
                }

                double k2 = 1;

                if (this.ti + dt > this.bMax)  //we need to have ti+k*dt=bMax  or ti+k*dt=bMin 
{
                    k2 = (this.bMax - this.ti) / dt;
                } else if (this.ti + dt < this.bMin) {
                    k2 = (this.bMin - this.ti) / dt;
                }

                double k = Math.Min(k1, k2);
                ds *= k;
                dt *= k;

            } else if (ds == 0) {
                if (this.ti + dt > this.bMax) {
                    dt = this.bMax - this.ti;
                } else if (this.ti + dt < this.bMin) {
                    dt = this.bMin - this.ti;
                }
            } else {   //dt==0)
                if (this.si + ds > this.aMax) {
                    ds = this.aMax - this.si;
                } else if (this.si + ds < this.aMin) {
                    ds = this.aMin - this.si;
                }
            }
        }

        private void ParallelLineSegLineSegMinDist() {
            LineSegment l0 = this.curveA as LineSegment;
            LineSegment l1 = this.curveB as LineSegment;

            Point v0 = l0.Start;
            Point v1 = l0.End;
            Point v2 = l1.Start;
            Point v3 = l1.End;

            Point d0 = v1 - v0;

            double nd0 = d0.Length;

            double r0 = 0, r1, r2, r3;

            if (nd0 > ApproximateComparer.DistanceEpsilon) {
                //v0 becomes the zero point
                d0 /= nd0;
                r1 = d0 * (v1 - v0);
                r2 = d0 * (v2 - v0);
                r3 = d0 * (v3 - v0);

                bool swapped = false;
                if (r2 > r3) {
                    swapped = true;
                    double t = r2;
                    r2 = r3;
                    r3 = t;
                }

                if (r3 < r0) {
                    this.aSolution = 0;
                    this.bSolution = swapped ? 0 : 1;
                } else if (r2 > r1) {
                    this.aSolution = 1;
                    this.bSolution = swapped ? 1 : 0;

                } else {
                    double r = Math.Min(r1, r3);
                    this.aSolution = r / (r1 - r0);
                    this.bSolution = (r - r2) / (r3 - r2);
                    if (swapped) {
                        this.bSolution = 1 - this.bSolution;
                    }
                }
            } else {
                Point d1 = v3 - v2;
                double nd1 = d1.Length;
                if (nd1 > ApproximateComparer.DistanceEpsilon) {
                    //v2 becomes the zero point
                    d1 /= nd1;
                    r0 = 0; //v2 position
                    r1 = d1 * (v3 - v2);//v3 position
                    r2 = d1 * (v0 - v2);//v0 position - here v0 and v1 are indistinguishable


                    if (r2 < r0) {
                        this.bSolution = 0;
                        this.aSolution = 1;
                    } else if (r2 > r1) {
                        this.bSolution = 1;
                        this.aSolution = 0;

                    } else {
                        double r = Math.Min(r1, r2);
                        this.bSolution = r / (r1 - r0);
                        this.aSolution = 0;
                    }

                } else {
                    this.aSolution = 0;
                    this.bSolution = 0;
                }
            }
            this.aPoint = this.curveA[this.aSolution];
            this.bPoint = this.curveB[this.bSolution];
        }
    }

}
