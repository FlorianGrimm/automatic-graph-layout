using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Linq;

using Microsoft.Msagl.Core.DataStructures;

// Retain SolverFoundation code for the moment in case it's useful for testing.
// To use it, edit MSAGL Project properties to enable the SOLVERFOUNDATION #define
// and include a reference to GraphLayout\MSAGL\Microsoft.Solver.Foundation.dll.
#if SOLVERFOUNDATION
using Microsoft.SolverFoundation.Services;
#else // SOLVERFOUNDATION

#endif // SOLVERFOUNDATION

// ReSharper disable CheckNamespace
namespace Microsoft.Msagl.Core.ProjectionSolver{
// ReSharper restore CheckNamespace
    /// <summary>
    /// just a convenient interface to the real solver
    /// </summary>
    public class SolverShell {
        private const double FixedVarWeight = 10e8;
        private readonly Dictionary<int, Variable> variables = new Dictionary<int, Variable>();
        private Solver solver;
        private Solution solution;
        private readonly Dictionary<int, double> fixedVars = new Dictionary<int, double>();

        /// <summary>
        /// Constructor.
        /// </summary>
        public SolverShell(){
            this.InitSolver();
        }

        /// <summary>
        /// Add a node that we would like as close to position i as possible, with the requested weight.
        /// </summary>
        /// <param name="id">Caller's unique identifier for this node</param>
        /// <param name="position">Desired position</param> 
        /// <param name="weight">The weight of the corresponding term in the goal function</param>
        public void AddVariableWithIdealPosition(int id, double position, double weight){
            // This throws an ArgumentException if a variable with id is already there.
            this.variables.Add(id, this.solver.AddVariable(id, position, weight));
        }

        /// <summary>
        /// Add a node that we would like as close to position i as possible, with the requested weight.
        /// </summary>
        /// <param name="id"></param>
        /// <param name="position"></param>
        public void AddVariableWithIdealPosition(int id, double position){
            this.AddVariableWithIdealPosition(id, position, 1.0);
        }

        /// <summary>
        /// Add a constraint that leftNode+gap eq|leq RightNode.
        /// </summary>
        /// <param name="idLeft">Caller's unique identifier for the left node</param>
        /// <param name="idRight">Caller's unique identifier for the right node</param>
        /// <param name="gap">Required gap</param>
        /// <param name="isEquality">Gap is exact rather than minimum</param>
        public void AddLeftRightSeparationConstraint(int idLeft, int idRight, double gap, bool isEquality){
            // The variables must already have been added by AddNodeWithDesiredPosition.
            var varLeft = this.GetVariable(idLeft);
            if (varLeft == null) {
                return;
            }

            var varRight = this.GetVariable(idRight);
            if (varRight == null) {
                return;
            }

            this.solver.AddConstraint(varLeft, varRight, gap, isEquality);
        }

        /// <summary>
        /// Add a constraint that leftNode+gap leq RightNode.
        /// </summary>
        /// <param name="idLeft">Caller's unique identifier for the left node</param>
        /// <param name="idRight">Caller's unique identifier for the right node</param>
        /// <param name="gap">Required minimal gap</param>
        public void AddLeftRightSeparationConstraint(int idLeft, int idRight, double gap){
            this.AddLeftRightSeparationConstraint(idLeft, idRight, gap, false /*isEquality*/);
        }

        /// <summary>
        /// Add a goal that minimizes the distance between two nodes, i.e. weight*((id1-id2)^2).
        /// </summary>
        /// <param name="id1">Caller's unique identifier for the first node.</param>
        /// <param name="id2">Caller's unique identifier for the second node.</param>
        /// <param name="weight">The weight of the corresponding term in the goal function</param>
        public void AddGoalTwoVariablesAreClose(int id1, int id2, double weight){
            var var1 = this.GetVariable(id1);
            if (var1 == null) {
                return;
            }

            var var2 = this.GetVariable(id2);
            if (var2 == null) {
                return;
            }

            this.solver.AddNeighborPair(var1, var2, weight);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="id1"></param>
        /// <param name="id2"></param>
        public void AddGoalTwoVariablesAreClose(int id1, int id2){
            this.AddGoalTwoVariablesAreClose(id1, id2, 1);
        }

        private Variable GetVariable(int i)
        {
            Variable v;
            return this.variables.TryGetValue(i, out v) ? v : null;
        }

        /// <summary>
        /// Execute the solver, filling in the Solution object and the values to be returned by GetVariableResolvedPosition.
        /// </summary>
        public void Solve(){
            this.Solve(null);
        }

        /// <summary>
        /// Execute the solver, filling in the Solution object and the values to be returned by GetVariableResolvedPosition.
        /// </summary>
        /// <param name="parameters">Parameter object class specific to the underlying solver</param>
        /// <returns>Pass or fail</returns>
        public void Solve(object parameters){
            bool executionLimitExceeded;
            this.Solve(parameters, out executionLimitExceeded);
        }

        /// <summary>
        /// Execute the solver, filling in the Solution object and the values to be returned by GetVariableResolvedPosition.
        /// </summary>
        /// <param name="parameters">Parameter object class specific to the underlying solver</param>
        /// <param name="executionLimitExceeded">if true, one or more limits such as iteration count 
        ///         or timeout were exceeded</param>
        /// <returns>Pass or fail</returns>
        [SuppressMessage("Microsoft.Usage", "CA2208")]
        public bool Solve(object parameters, out bool executionLimitExceeded){
            bool fixedVarsMoved;
            do{
                this.solution = null; // Remove any stale solution in case parameters validation or Solve() throws.

                Parameters solverParameters = null;
                if(null != parameters){
                    solverParameters = parameters as Parameters;
                    if (solverParameters == null) {
                        throw new ArgumentException("parameters");
                    }
                }

                this.solution = this.solver.Solve(solverParameters);
#if DEVTRACE
                System.Diagnostics.Debug.Assert(0 == solution.NumberOfUnsatisfiableConstraints, "Unsatisfiable constraints encountered");
#endif // DEVTRACE
                executionLimitExceeded = this.solution.ExecutionLimitExceeded;
                fixedVarsMoved = this.AdjustConstraintsForMovedFixedVars();
            } while (fixedVarsMoved && this.solution.ExecutionLimitExceeded==false);
            return this.solution.ExecutionLimitExceeded == false;
        }

        //        void DumpToFile(string fileName) {
        //            var file = new StreamWriter(fileName);
        //            file.WriteLine("digraph {");
        //            foreach (var v in solver.Variables) {
        //                var s = v.Weight > 100 ? "color=\"red\"" : "";
        //                file.WriteLine(v.UserData + " [ label=" + "\"" + v.UserData +"\\n" +
        //                               v.DesiredPos + "\" " +s+ "]");
        //                
        //            }
        //
        //            foreach (var cs in solver.Constraints) {
        //                file.WriteLine(cs.Left.UserData + " -> " + cs.Right.UserData + " [ label=\"" + cs.Gap + "\"]");
        //            }
        //            file.WriteLine("}");
        //            file.Close();
        //        }

        private bool AdjustConstraintsForMovedFixedVars(){
            var movedFixedVars=new Set<int>(this.fixedVars.
                Where(kv=>!Close(kv.Value, this.GetVariableResolvedPosition(kv.Key))).Select(p=>p.Key));
            if(movedFixedVars.Count==0) {
                return false;
            }

            return this.AdjustConstraintsForMovedFixedVarSet(movedFixedVars);
            
        }

        private static bool Close(double a, double b){
            return Math.Abs(a - b) < 0.0005; //so if a fixed variable moved less than 0.0001 we do not care!
        }

        private bool AdjustConstraintsForMovedFixedVarSet(Set<int> movedFixedVars){
            while (movedFixedVars.Count>0){
                var fixedVar = movedFixedVars.First();
                if(!this.AdjustSubtreeOfFixedVar(fixedVar, movedFixedVars)) {
                    return false;
                }
            }
            return true;
        }

        private bool AdjustSubtreeOfFixedVar(int fixedVar, Set<int> movedFixedVars){
            bool successInAdjusting;
            var neighbors= this.AdjustConstraintsOfNeighborsOfFixedVariable(fixedVar, out successInAdjusting);
            if (!successInAdjusting) {
                return false;
            }

            if (!neighbors.Any()) {
                return false;
            }

            foreach (var i in neighbors) {
                movedFixedVars.Remove(i);
            }

            return true;
        }

        /// <summary>
        /// returns the block of the fixed variable
        /// </summary>
        /// <param name="fixedVar"></param>
        /// <param name="successInAdjusing"></param>
        /// <returns></returns>
        private IEnumerable<int> AdjustConstraintsOfNeighborsOfFixedVariable(int fixedVar, out bool successInAdjusing){
            var nbs = this.variables[fixedVar].Block.Variables;
            var currentSpan = new RealNumberSpan();
            var idealSpan = new RealNumberSpan();
            double scale = 1;
            foreach (var u in nbs){
                if (!this.fixedVars.ContainsKey((int)u.UserData )) {
                    continue;
                }

                currentSpan.AddValue(u.ActualPos);
                idealSpan.AddValue(u.DesiredPos);
                if (idealSpan.Length > 0) {
                    scale = Math.Max(scale, currentSpan.Length/idealSpan.Length);
                }
            }
            if (scale == 1) {
                scale = 2;//just relax the constraints 
            }

            successInAdjusing = this.FixActiveConstraints(nbs, scale);
            return nbs.Select(u => (int) u.UserData);
        }

        /// <summary>
        /// if all active constraint gaps are less than this epsilon we should stop trying adjusting
        /// </summary>
        private const double FailToAdjustEpsilon=0.001;

        private bool FixActiveConstraints(IEnumerable<Variable> neighbs, double scale){
            var ret = false;
            foreach (var c in from v in neighbs from c in v.LeftConstraints where c.IsActive select c) {
                if (c.Gap > FailToAdjustEpsilon) {
                    ret = true;
                }

                this.solver.SetConstraintUpdate(c, c.Gap/scale);
            }
            return ret;
        }

        /// <summary>
        /// Obtain the solved position for a node.
        /// </summary>
        /// <param name="id">Caller's unique identifier for the node.</param>
        /// <returns>The node's solved position.</returns>
        public double GetVariableResolvedPosition(int id){
            var v = this.GetVariable(id);
            return v == null ? 0 : v.ActualPos;
        }

        /// <summary>
        /// 
        /// </summary>
        public void InitSolver(){
            this.solver = new Solver();
            this.variables.Clear();
        }


        /// <summary>
        /// Add a variable with a known and unchanging position.
        /// </summary>
        /// <param name="id">Caller's unique identifier for the node</param>
        /// <param name="position">Desired position.</param>
        public void AddFixedVariable(int id, double position){
            this.AddVariableWithIdealPosition(id, position, FixedVarWeight);
            this.fixedVars[id] = position;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public bool ContainsVariable(int v){
            return this.variables.ContainsKey(v);
        }

        /// <summary>
        /// returns the ideal position of the node that had been set at the variable construction
        /// </summary>
        /// <param name="v"></param>
        /// <returns></returns>
        public double GetVariableIdealPosition(int v){
            return this.variables[v].DesiredPos;
        }

        /// <summary>
        /// Returns the solution object class specific to the underlying solver, or null if there has
        /// been no call to Solve() or it threw an exception.
        /// </summary>
        public object Solution{
            get { return this.solution; }
        }
    } // end class SolverShell
} // end namespace Microsoft.Msagl
