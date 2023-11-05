using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Routing.Visibility;

namespace Microsoft.Msagl.Routing.Spline.ConeSpanner {
    internal class ConeSpanner : AlgorithmBase {
        private readonly IEnumerable<Polyline> _obstacles;

        // double coneAngle = Math.PI / 18;// ten degrees
        //double coneAngle = Math.PI / 9;// twenty degrees


        private readonly VisibilityGraph _visibilityGraph;
        private double coneAngle = Math.PI/6; // thirty degrees
        private bool _bidirectional;



        internal ConeSpanner(IEnumerable<Polyline> obstacles, VisibilityGraph visibilityGraph) {
            this._obstacles = VisibilityGraph.OrientHolesClockwise(obstacles);
            this._visibilityGraph = visibilityGraph;
        }

        internal ConeSpanner(IEnumerable<Polyline> obstacles, VisibilityGraph visibilityGraph, double coneAngle,
                             Set<Point> ports, Polyline borderPolyline)
            : this(obstacles, visibilityGraph) {
            Debug.Assert(borderPolyline == null || obstacles.All(o => Curve.CurveIsInsideOther(o, borderPolyline)));
            Debug.Assert(borderPolyline == null ||
                         ports.All(o => Curve.PointRelativeToCurveLocation(o, borderPolyline) == PointLocation.Inside));

            //Debug.Assert(obstacles.All(o => ports.All(p => Curve.PointRelativeToCurveLocation(p, o) == PointLocation.Outside)));
            //todo: uncomment this assert - it failes on D:\progression\src\ddsuites\src\vs\Progression\ScenarioTests\Grouping\GroupingResources\GroupBySelection2.dgml
            //when dragging
            Debug.Assert(coneAngle > Math.PI/180*2 && coneAngle <= Math.PI/2);

            this.Ports = ports;
            this.BorderPolyline = borderPolyline;
            this.ConeAngle = coneAngle;
        }

        internal double ConeAngle {
            get { return this.coneAngle; }
            set { this.coneAngle = value; }
        }

        private Set<Point> Ports { get; set; }
        private Polyline BorderPolyline { get; set; }

        /// <summary>
        /// If set to true then a smaller visibility graph is created.
        /// An edge is added to the visibility graph only if it is found at least twice: 
        /// once sweeping with a direction d and the second time with -d
        /// </summary>
        internal bool Bidirectional {
            get { return this._bidirectional; }
            set { this._bidirectional = value; }
        }

        internal static int GetTotalSteps(double coneAngle) {
#if SHARPKIT //https://github.com/SharpKit/SharpKit/issues/4 integer rounding issue
            return ((int)((2 * Math.PI - coneAngle / 2) / coneAngle)) + 1;
#else
            return (int)((2 * Math.PI - coneAngle / 2) / coneAngle) + 1;
#endif
        }

        protected override void RunInternal() {
            double offset =  2*Math.PI - this.coneAngle /2;
            if (!this.Bidirectional) {
                double angle;
                for (int i = 0; (angle = this.coneAngle *i) <= offset; i++) {
                    this.ProgressStep();
                    this.AddDirection(new Point(Math.Cos(angle), Math.Sin(angle)), this.BorderPolyline, this._visibilityGraph);
                }
            }
            else {
                this.HandleBideractionalCase();
            }
        }

        private void HandleBideractionalCase() {
            int k = (int)(Math.PI/ this.coneAngle);
            for (int i = 0; i < k; i++) {
                var angle = i* this.coneAngle;
                var vg0 = new VisibilityGraph();
                this.AddDirection(new Point(Math.Cos(angle), Math.Sin(angle)), this.BorderPolyline, vg0);
                var vg1 = new VisibilityGraph();
                this.AddDirection(new Point(-Math.Cos(angle), -Math.Sin(angle)), this.BorderPolyline, vg1);
                this.AddIntersectionOfBothDirectionSweepsToTheResult(vg0, vg1);                
            }

          

        }

        private void AddIntersectionOfBothDirectionSweepsToTheResult(VisibilityGraph vg0, VisibilityGraph vg1) {
            foreach (var edge in vg0.Edges) {
                if (vg1.FindEdge(edge.SourcePoint, edge.TargetPoint) != null) {
                    this._visibilityGraph.AddEdge(edge.SourcePoint, edge.TargetPoint);
                }
            }
        }

        private void AddDirection(Point direction, Polyline borderPolyline, VisibilityGraph visibilityGraph) {
            LineSweeper.Sweep(this._obstacles, direction, this.coneAngle, visibilityGraph, this.Ports, borderPolyline);
        }
    }
}