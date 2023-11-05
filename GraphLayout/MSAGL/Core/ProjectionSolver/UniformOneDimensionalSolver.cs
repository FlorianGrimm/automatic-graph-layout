using System;
using System.Collections.Generic;
using System.Diagnostics;

using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.GraphAlgorithms;

namespace Microsoft.Msagl.Core.ProjectionSolver{
    internal class UniformOneDimensionalSolver {
        private readonly Dictionary<int, double> idealPositions = new Dictionary<int, double>();
        private readonly double varSepartion;
        /// <summary>
        /// desired variable separation
        /// </summary>
        /// <param name="variableSeparation"></param>
        public UniformOneDimensionalSolver(double variableSeparation){
            this.varSepartion = variableSeparation;
        }

        private readonly List<UniformSolverVar> varList = new List<UniformSolverVar>();
        private readonly Set<IntPair> constraints = new Set<IntPair>();
        private BasicGraphOnEdges<IntPair> graph;

//        delegate IEnumerable<NudgerConstraint> Edges(int i);
//
//        delegate int End(NudgerConstraint constraint);

//        Edges outEdgesDel;
//        Edges inEdgesDel;
//        End sourceDelegate;
//        End targetDelegate;
//        Supremum minDel;
//        Supremum maxDel;

        internal void SetLowBound(double bound, int id){
            var v = this.Var(id);
            v.LowBound = Math.Max(bound, v.LowBound);
        }

        private UniformSolverVar Var(int id){
            return this.varList[id];
        }

        internal void SetUpperBound(int id, double bound){
            var v = this.Var(id);
            v.UpperBound = Math.Min(bound, v.UpperBound);
        }

 

        internal void Solve(){
            this.SolveByRegularSolver();
        }

        private readonly SolverShell solverShell = new SolverShell();

        private void SolveByRegularSolver() {
            this.CreateVariablesForBounds();
            for (int i = 0; i < this.varList.Count; i++) {
                var v = this.varList[i];
                if (v.IsFixed) {
                    this.solverShell.AddFixedVariable(i, v.Position);
                } else {
                    this.solverShell.AddVariableWithIdealPosition(i, this.idealPositions[i]);
                    if (v.LowBound != double.NegativeInfinity) {
                        //    solverShell.AddLeftRightSeparationConstraint(GetBoundId(v.LowBound), i, varSepartion);
                        this.constraints.Insert(new IntPair(this.GetBoundId(v.LowBound), i));
                    }

                    if (v.UpperBound != double.PositiveInfinity) {
                        this.constraints.Insert(new IntPair(i, this.GetBoundId(v.UpperBound)));
                    }
                }
            }

            this.CreateGraphAndRemoveCycles();

            foreach (var edge in this.graph.Edges) {
                var w = 0.0;
                if(edge.First< this.varList.Count) {
                    w += this.varList[edge.First].Width;
                }

                if (edge.Second < this.varList.Count) {
                    w += this.varList[edge.Second].Width;
                }

                w /= 2;
                this.solverShell.AddLeftRightSeparationConstraint(edge.First, edge.Second, this.varSepartion + w);
            }
            this.solverShell.Solve();

            for (int i = 0; i < this.varList.Count; i++) {
                this.varList[i].Position = this.solverShell.GetVariableResolvedPosition(i);
            }
        }

        private int GetBoundId(double bound) {
            return this.boundsToInt[bound];
        }

        private void CreateVariablesForBounds(){
            foreach (var v in this.varList){
                if(v.IsFixed) {
                    continue;
                }

                if (v.LowBound != double.NegativeInfinity) {
                    this.RegisterBoundVar(v.LowBound);
                }

                if (v.UpperBound != double.PositiveInfinity) {
                    this.RegisterBoundVar(v.UpperBound);
                }
            }
        }

        private readonly Dictionary<double, int> boundsToInt = new Dictionary<double, int>();

        private void RegisterBoundVar(double bound){
            if (!this.boundsToInt.ContainsKey(bound)){
                int varIndex= this.varList.Count + this.boundsToInt.Count;
                this.boundsToInt[bound] = varIndex;
                this.solverShell.AddFixedVariable(varIndex, bound);
            }
        }

        private void CreateGraphAndRemoveCycles(){
            //edges in the graph go from a smaller value to a bigger value
            this.graph = new BasicGraphOnEdges<IntPair>(this.constraints, this.varList.Count+ this.boundsToInt.Count);
            //removing cycles
            var feedbackSet = CycleRemoval<IntPair>.GetFeedbackSet(this.graph);
            if(feedbackSet!=null) {
                foreach (var edge in feedbackSet) {
                    this.graph.RemoveEdge(edge as IntPair);
                }
            }
        }

        internal double GetVariablePosition(int id){
            return this.varList[id].Position;
        }

        internal void AddConstraint(int i, int j){
            this.constraints.Insert(new IntPair(i, j));
        }

        internal void AddVariable(int id, double currentPosition, double idealPosition, double width){
            this.idealPositions[id] = idealPosition;
            this.AddVariable(id, currentPosition, false, width);

        }

        internal void AddFixedVariable(int id, double position){
            this.AddVariable(id, position,true, 0); //0 for width
        }

        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Usage", "CA1801:ReviewUnusedParameters", MessageId = "id")]
        private void AddVariable(int id, double position, bool isFixed, double width) {
            Debug.Assert(id== this.varList.Count);
            this.varList.Add(new UniformSolverVar { IsFixed = isFixed, Position = position, Width=width });
        }
    }
}