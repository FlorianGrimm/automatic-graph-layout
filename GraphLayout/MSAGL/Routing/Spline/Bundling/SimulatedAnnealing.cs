using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core.Routing;
using Microsoft.Msagl.Routing.Visibility;
using Microsoft.Msagl.DebugHelpers;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;

namespace Microsoft.Msagl.Routing.Spline.Bundling {
    /// <summary>
    /// Adjust current bundle-routing
    /// </summary>
    public class SimulatedAnnealing {
        /// <summary>
        /// bundle data
        /// </summary>
        private readonly MetroGraphData metroGraphData;

        /// <summary>
        /// Algorithm settings
        /// </summary>
        private readonly BundlingSettings bundlingSettings;

        ///  calculates rouing cost
        private readonly CostCalculator costCalculator;

        ///  used for fast calculation of intersections
        private readonly IntersectionCache cache;

        /// <summary>
        /// fix routing by simulated annealing algorithm
        /// </summary>
        internal static bool FixRouting(MetroGraphData metroGraphData, BundlingSettings bundlingSettings) {
            return FixRouting(metroGraphData, bundlingSettings, null);
        }

        internal static bool FixRouting(MetroGraphData metroGraphData, BundlingSettings bundlingSettings, HashSet<Point> changedPoints) {
            return new SimulatedAnnealing(metroGraphData, bundlingSettings).FixRouting(changedPoints);
        }

        private SimulatedAnnealing(MetroGraphData metroGraphData, BundlingSettings bundlingSettings) {
            this.metroGraphData = metroGraphData;
            this.bundlingSettings = bundlingSettings;
            this.costCalculator = new CostCalculator(metroGraphData, bundlingSettings);
            this.cache = new IntersectionCache(metroGraphData, bundlingSettings, this.costCalculator, metroGraphData.Cdt);
        }

        private const int MaxIterations = 100;
        private const double MaxStep = 50;
        private const double MinStep = 1;
        private const double MinRelativeChange = 0.0005;
        private HashSet<Station> stationsForOptimizations;

        /// <summary>
        /// Use constraint edge routing to reduce ink
        /// </summary>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters",
         MessageId = "Microsoft.Msagl.Routing.Spline.Bundling.GeneralBundling.InkMetric.OutputQ(System.String,Microsoft.Msagl.Routing.Spline.Bundling.GeneralBundling.MetroGraphData,Microsoft.Msagl.Core.Routing.BundlingSettings)"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Globalization", "CA1303:Do not pass literals as localized parameters", MessageId = "Microsoft.Msagl.Routing.Spline.Bundling.GeneralBundling.InkMetric.OutputQ(System.String,Microsoft.Msagl.Routing.Spline.Bundling.GeneralBundling.MetroGraphData,Microsoft.Msagl.Routing.Spline.Bundling.BundlingSettings)")]
        private bool FixRouting(HashSet<Point> changedPoints) {
            this.stationsForOptimizations = this.GetStationsForOptimizations(changedPoints);

            this.cache.InitializeCostCache();

            double step = MaxStep;
            double energy = double.PositiveInfinity;

            List<Point> x = new List<Point>(this.metroGraphData.VirtualNodes().Select(v => v.Position));
            int iteration = 0;
            while (iteration++ < MaxIterations) {
                bool coordinatesChanged = this.TryMoveNodes();
                //TimeMeasurer.DebugOutput("  #iter = " + iteration + " moved: " + cnt + "/" + metroGraphData.VirtualNodes().Count() + " step: " + step);

                if (iteration <= 1 && !coordinatesChanged) {
                    return false;
                }

                if (!coordinatesChanged) {
                    break;
                }

                double oldEnergy = energy;
                energy = CostCalculator.Cost(this.metroGraphData, this.bundlingSettings);
                //TimeMeasurer.DebugOutput("energy: " + energy);

                step = this.UpdateMaxStep(step, oldEnergy, energy);
                List<Point> oldX = x;
                x = new List<Point>(this.metroGraphData.VirtualNodes().Select(v => v.Position));
                if (step < MinStep || this.Converged(step, oldX, x)) {
                    break;
                }
            }

            //TimeMeasurer.DebugOutput("SA completed after " + iteration + " iterations");
            return true;
        }

        private HashSet<Station> GetStationsForOptimizations(HashSet<Point> changedPoints) {
            if (changedPoints == null) {
                return new HashSet<Station>(this.metroGraphData.VirtualNodes());
            }
            else {
                HashSet<Station> result = new HashSet<Station>();
                foreach (var p in changedPoints) {
                    if (this.metroGraphData.PointToStations.ContainsKey(p)) {
                        var s = this.metroGraphData.PointToStations[p];
                        if (!s.IsRealNode) {
                            result.Add(s);
                        }
                    }
                }
                return result;
            }
        }

        /// <summary>
        /// stop SA if relative changes are small
        /// </summary>
        private bool Converged(double step, List<Point> oldx, List<Point> newx) {
            //return false;

            double num = 0, den = 0;
            for (int i = 0; i < oldx.Count; i++) {
                num += (oldx[i] - newx[i]).LengthSquared;
                den += oldx[i].LengthSquared;
            }
            double res = Math.Sqrt(num / den);
            return (res < MinRelativeChange);
        }

        private int stepsWithProgress = 0;

        private double UpdateMaxStep(double step, double oldEnergy, double newEnergy) {
            //cooling factor
            double T = 0.8;
            if (newEnergy + 1.0 < oldEnergy) {
                this.stepsWithProgress++;
                if (this.stepsWithProgress >= 5) {
                    this.stepsWithProgress = 0;
                    step = Math.Min(MaxStep, step / T);
                }
            }
            else {
                this.stepsWithProgress = 0;
                step *= T;
            }

            return step;
        }

        private bool TryMoveNodes() {
            var coordinatesChanged = false;
            HashSet<Station> movedStations = new HashSet<Station>();
            //foreach (var node in metroGraphData.VirtualNodes()) {
            foreach (var node in this.stationsForOptimizations) {
                if (this.TryMoveNode(node)) {
                    Debug.Assert(this.stationsForOptimizations.Contains(node));

                    coordinatesChanged = true;
         
                    movedStations.Add(node);
                    foreach (var adj in node.Neighbors) {
                        if (!adj.IsRealNode) {
                            movedStations.Add(adj);
                        }
                    }
                }
            }

            this.stationsForOptimizations = movedStations;
            return coordinatesChanged;
        }

        /// <summary>
        /// Move node to decrease the cost of the drawing
        /// Returns true iff position has changed
        /// </summary>
        private bool TryMoveNode(Station node) {
            Point direction = this.BuildDirection(node);
            if (direction.Length == 0) {
                return false;
            }

            double stepLength = this.BuildStepLength(node, direction);
            if (stepLength < MinStep) {
                //try random direction
                direction = Point.RandomPoint();
                stepLength = this.BuildStepLength(node, direction);
                if (stepLength < MinStep) {
                    return false;
                }
            }

            Point step = direction * stepLength;
            Point newPosition = node.Position + step;
            //can this happen?
            if (this.metroGraphData.PointToStations.ContainsKey(newPosition)) {
                return false;
            }

            this.metroGraphData.MoveNode(node, newPosition);
            this.cache.UpdateCostCache(node);
            return true;
        }

        /// <summary>
        /// Calculate the direction to improve the ink function
        /// </summary>
        private Point BuildDirection(Station node) {
            var forceInk = this.BuildForceForInk(node);
            var forcePL = this.BuildForceForPathLengths(node);
            var forceR = this.BuildForceForRadius(node);
            var forceBundle = this.BuildForceForBundle(node);

            var force = forceInk + forcePL + forceR + forceBundle;
            if (force.Length < 0.1) {
                return new Point();
            }

            force = force.Normalize();

            return force;
        }

        private double BuildStepLength(Station node, Point direction) {
            double stepLength = MinStep;

            double costGain = this.CostGain(node, node.Position + direction * stepLength);
            if (costGain < 0.01) {
                return 0;
            }

            while (2 * stepLength <= MaxStep) {
                double newCostGain = this.CostGain(node, node.Position + direction * stepLength * 2);
                if (newCostGain <= costGain) {
                    break;
                }

                stepLength *= 2;
                costGain = newCostGain;
            }

            return stepLength;
        }

        /// <summary>
        /// Computes cost delta when moving the node
        /// the cost will be negative if a new position overlaps obstacles
        /// </summary>
        private double CostGain(Station node, Point newPosition) {
            double MInf = -12345678.0;
            double rGain = this.costCalculator.RadiusGain(node, newPosition);
            if (rGain < MInf) {
                return MInf;
            }

            double bundleGain = this.costCalculator.BundleGain(node, newPosition);
            if (bundleGain < MInf) {
                return MInf;
            }

            double inkGain = this.costCalculator.InkGain(node, newPosition);
            double plGain = this.costCalculator.PathLengthsGain(node, newPosition);

            return rGain + inkGain + plGain + bundleGain;
        }

        /// <summary>
        /// force to decrease ink
        /// </summary>
        private Point BuildForceForInk(Station node) {
            //return new Point();
            Point direction = new Point();
            foreach (var adj in node.Neighbors) {
                var p = (adj.Position - node.Position);
                direction += p / p.Length;
            }

            //derivative
            Point force = direction * this.bundlingSettings.InkImportance;

            return force;
        }

        /// <summary>
        /// direction to decrease path lengths
        /// </summary>
        private Point BuildForceForPathLengths(Station node) {
            //return new Point();
            var direction = new Point();

            foreach (var mni in this.metroGraphData.MetroNodeInfosOfNode(node)) {
                var metroline = mni.Metroline;
                Point u = mni.PolyPoint.Next.Point;
                Point v = mni.PolyPoint.Prev.Point;

                var p1 = u - node.Position;
                var p2 = v - node.Position;
                direction += p1 / (p1.Length * metroline.IdealLength);
                direction += p2 / (p2.Length * metroline.IdealLength);
            }

            //derivative
            Point force = direction * this.bundlingSettings.PathLengthImportance;

            return force;
        }

        /// <summary>
        /// direction to increase radii
        /// </summary>
        private Point BuildForceForRadius(Station node) {
            Point direction = new Point();

            double idealR = node.cachedIdealRadius;
            List<Tuple<Polyline, Point>> touchedObstacles;
            bool res = this.metroGraphData.looseIntersections.HubAvoidsObstacles(node, node.Position, idealR, out touchedObstacles);
            Debug.Assert(res);

            foreach (var d in touchedObstacles) {
                double dist = (d.Item2 - node.Position).Length;
                Debug.Assert(dist <= idealR);
                double lforce = 2.0 * (1.0 - dist / idealR);
                Point dir = (node.Position - d.Item2).Normalize();
                direction += dir * lforce;
            }

            //derivative
            Point force = direction * this.bundlingSettings.HubRepulsionImportance;

            return force;
        }

        /// <summary>
        /// direction to push a bundle away from obstacle
        /// </summary>
        private Point BuildForceForBundle(Station node) {
            var direction = new Point();
            foreach (var adj in node.Neighbors) {
                double idealWidth = this.metroGraphData.GetWidth(node, adj, this.bundlingSettings.EdgeSeparation);

                List<Tuple<Point, Point>> closestPoints;
                bool res = this.metroGraphData.cdtIntersections.BundleAvoidsObstacles(node, adj, node.Position, adj.Position, idealWidth / 2, out closestPoints);
                if (!res) {
#if TEST_MSAGL&& TEST_MSAGL
                    HubDebugger.ShowHubsWithAdditionalICurves(this.metroGraphData, this.bundlingSettings, new LineSegment(node.Position, adj.Position));
#endif
                }
                Debug.Assert(res);  //todo : still unsolved

                foreach (var d in closestPoints) {
                    double dist = (d.Item1 - d.Item2).Length;
                    Debug.Assert(ApproximateComparer.LessOrEqual(dist, idealWidth / 2));
                    double lforce = 2.0 * (1.0 - dist / (idealWidth / 2));
                    Point dir = -(d.Item1 - d.Item2).Normalize();
                    direction += dir * lforce;
                }
            }

            //derivative
            Point force = direction * this.bundlingSettings.BundleRepulsionImportance;

            return force;
        }
    }
}
