using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Routing.Rectilinear.Nudging {
    /// <summary>
    ///Avoid a situation where two paths cross each other more than once. Remove self loops.
    /// 
    /// </summary>
    internal class PathMerger{
        internal PathMerger(IEnumerable<Path> paths){
            this.Paths = paths;
        }

        internal IEnumerable<Path> Paths { get; set; }

        private readonly Dictionary<Point, Dictionary<Path, LinkedPoint>> verticesToPathOffsets =
            new Dictionary<Point, Dictionary<Path, LinkedPoint>>();
            

        /// <summary>
        /// Avoid a situation where two paths cross each other more than once. Remove self loops.
        /// </summary>
        internal void MergePaths(){
            this.InitVerticesToPathOffsetsAndRemoveSelfCycles();
            foreach (var path in this.Paths) {
                this.ProcessPath(path);
            }
        }

        private void ProcessPath(Path path) {
            var departedPaths = new Dictionary<Path, LinkedPoint>();
            Dictionary<Path, LinkedPoint> prevLocationPathOffsets = null;
            for (var linkedPoint = (LinkedPoint) path.PathPoints; linkedPoint != null; linkedPoint = linkedPoint.Next) {
                var pathOffsets = this.verticesToPathOffsets[linkedPoint.Point];
                if (prevLocationPathOffsets != null) {
                    //handle returning paths
                    if (departedPaths.Count > 0) {
                        foreach (var pair in pathOffsets) {
                            LinkedPoint departerLinkedPoint;
                            var path0 = pair.Key;
                            if (departedPaths.TryGetValue(path0, out departerLinkedPoint)) {//returned!
                                this.CollapseLoopingPath(path0, departerLinkedPoint, pair.Value, path, linkedPoint);
                                departedPaths.Remove(path0);
                            }
                        }
                    }

                    //find departed paths
                    foreach (var pair in prevLocationPathOffsets.Where(pair => !pathOffsets.ContainsKey(pair.Key))) {
                        departedPaths.Add(pair.Key, pair.Value);
                    }
                }
                prevLocationPathOffsets = pathOffsets;
            }
        }

        //        bool Correct() {
        //            foreach (var kv in verticesToPathOffsets) {
        //                Point p = kv.Key;
        //                Dictionary<Path, LinkedPoint> pathOffs = kv.Value;
        //                foreach (var pathOff in pathOffs) {
        //                    var path = pathOff.Key;
        //                    var linkedPoint = pathOff.Value;
        //                    if (linkedPoint.Point != p)
        //                        return false;
        //                    if (FindLinkedPointInPath(path, p) == null) {
        //                        return false;
        //                    }
        //                }
        //            }
        //            return true;
        //        }

        private void CollapseLoopingPath(Path loopingPath, LinkedPoint departureFromLooping, LinkedPoint arrivalToLooping, Path stemPath, LinkedPoint arrivalToStem){
            var departurePointOnStem = FindLinkedPointInPath(stemPath, departureFromLooping.Point);
            IEnumerable<Point> pointsToInsert = GetPointsInBetween(departurePointOnStem, arrivalToStem);
            if(Before(departureFromLooping, arrivalToLooping)){
                this.CleanDisappearedPiece(departureFromLooping, arrivalToLooping, loopingPath);
                this.ReplacePiece(departureFromLooping, arrivalToLooping, pointsToInsert,loopingPath);
            }else{
                this.CleanDisappearedPiece(arrivalToLooping, departureFromLooping, loopingPath);
                this.ReplacePiece(arrivalToLooping, departureFromLooping, pointsToInsert.Reverse(),loopingPath);
            }
        }

        private static IEnumerable<Point> GetPointsInBetween(LinkedPoint a, LinkedPoint b){
            for(var i=a.Next;i!=b;i=i.Next) {
                yield return i.Point;
            }
        }

        private void ReplacePiece(LinkedPoint a, LinkedPoint b, IEnumerable<Point> points, Path loopingPath){
            var prevPoint = a;
            foreach (var point in points){
                var lp = new LinkedPoint(point);
                prevPoint.Next = lp;
                prevPoint = lp;
                var pathOffset = this.verticesToPathOffsets[point];
                Debug.Assert(!pathOffset.ContainsKey(loopingPath));
                pathOffset[loopingPath] = prevPoint;
            }
            prevPoint.Next = b;
        }

        private void CleanDisappearedPiece(LinkedPoint a, LinkedPoint b, Path loopingPath){
            foreach (var point in GetPointsInBetween(a, b)) {
                var pathOffset = this.verticesToPathOffsets[point];
                Debug.Assert(pathOffset.ContainsKey(loopingPath));
                pathOffset.Remove(loopingPath);
            }
        }

        /// <summary>
        /// checks that a is before b in the path
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <returns>true is a is before b in the path</returns>
        private static bool Before(LinkedPoint a, LinkedPoint b){
            for(a=a.Next;a!=null;a=a.Next) {
                if (a==b) {
                    return true;
                }
            }

            return false;
        }

        private static LinkedPoint FindLinkedPointInPath(Path path, Point point) {
            //this function is supposed to always succeed. it will throw a null reference exception otherwise
            for (var linkedPoint = (LinkedPoint) path.PathPoints;; linkedPoint = linkedPoint.Next) {
                if (linkedPoint.Point == point) {
                    return linkedPoint;
                }
            }
        }

        private void InitVerticesToPathOffsetsAndRemoveSelfCycles(){
            foreach(var path in this.Paths){
                for (var linkedPoint = (LinkedPoint)path.PathPoints; linkedPoint != null; linkedPoint = linkedPoint.Next) {
                    Dictionary<Path, LinkedPoint> pathOffsets;
                    if (!this.verticesToPathOffsets.TryGetValue(linkedPoint.Point, out pathOffsets)) {
                        this.verticesToPathOffsets[linkedPoint.Point] = pathOffsets = new Dictionary<Path, LinkedPoint>();
                    }
                    //check for the loop
                    LinkedPoint loopPoint;
                    if (pathOffsets.TryGetValue(path, out loopPoint)) {//we have a loop
                        this.CleanDisappearedPiece(loopPoint, linkedPoint, path);
                        loopPoint.Next = linkedPoint.Next;
                    } else {
                        pathOffsets[path] = linkedPoint;
                    }
                }
            }
        }
    }
}