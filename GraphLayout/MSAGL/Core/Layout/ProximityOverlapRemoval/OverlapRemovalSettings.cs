using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.MinimumSpanningTree;
using Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval.StressEnergy;
using Microsoft.Msagl.Layout.MDS;

namespace Microsoft.Msagl.Core.Layout.ProximityOverlapRemoval {
    /// <summary>
    /// Settings for Overlap Removal process. Usage of the properties depends on the algorithm.
    /// </summary>
    public class OverlapRemovalSettings {
        private double epsilon = 0.01;
        private int iterationsMax=1000;
        private bool stopOnMaxIterat = false;
        private double nodeSeparation = 4;
        private int randomizationSeed = 1;
        private bool workInInches;

        /// <summary>
        /// Constructor.
        /// </summary>
        public OverlapRemovalSettings() {
            this.StressSettings =new StressMajorizationSettings();
        }

        /// <summary>
        /// Settings for the StressMajorization process.
        /// </summary>
        public StressMajorizationSettings StressSettings { get; set; }

        private bool randomizeAllPointsOnStart = false;
        /// <summary>
        /// If true, the overlap iteration process stops after maxIterat iterations.
        /// </summary>
        public bool StopOnMaxIterat {
            get { return this.stopOnMaxIterat; }
            set { this.stopOnMaxIterat = value; }
        }

        /// <summary>
        /// Epsilon
        /// </summary>
        public double Epsilon {
            get { return this.epsilon; }
            set { this.epsilon = value; }
        }

        /// <summary>
        /// Number of maxIterat to be made. In each iteration overlap is partly removed.
        /// </summary>
        public int IterationsMax {
            get { return this.iterationsMax; }
            set { this.iterationsMax = value; }
        }

        /// <summary>
        /// Minimal distance between nodes.
        /// </summary>
        public double NodeSeparation {
            get { return this.nodeSeparation; }
            set { this.nodeSeparation = value; }
        }

     
        /// <summary>
        /// 
        /// </summary>
        public int RandomizationSeed {
            get { return this.randomizationSeed; }
            set { this.randomizationSeed = value; }
        }

        /// <summary>
        /// 
        /// </summary>
        public bool RandomizeAllPointsOnStart {
            get { return this.randomizeAllPointsOnStart; }
            set { this.randomizeAllPointsOnStart = value; }
        }
        
        /// <summary>
        /// Divide the coordinates by 72(Pixels) to work in inches. At the end this transformation is reverted again.
        /// </summary>
        public bool WorkInInches {
            get { return this.workInInches; }
            set { this.workInInches = value; }
        }

        
        /// <summary>
        /// Clones the settings together with the stressmajorization settings
        /// </summary>
        /// <returns></returns>
        public OverlapRemovalSettings Clone() {
            OverlapRemovalSettings settings = new OverlapRemovalSettings();
            settings.Epsilon = this.Epsilon;
            settings.IterationsMax = this.IterationsMax;
            settings.StopOnMaxIterat = this.StopOnMaxIterat;
            settings.NodeSeparation = this.NodeSeparation;
            settings.RandomizationSeed = this.RandomizationSeed;
            settings.RandomizeAllPointsOnStart = this.randomizeAllPointsOnStart;
            settings.WorkInInches = this.WorkInInches;
       
            settings.StressSettings=new StressMajorizationSettings();
            settings.StressSettings.MaxStressIterations = this.StressSettings.MaxStressIterations;
            settings.StressSettings.SolvingMethod = this.StressSettings.SolvingMethod;
            settings.StressSettings.UpdateMethod = this.StressSettings.UpdateMethod;
            settings.StressSettings.StressChangeTolerance = this.StressSettings.StressChangeTolerance;
            settings.StressSettings.CancelOnStressConvergence = this.StressSettings.CancelOnStressConvergence;
            settings.StressSettings.CancelOnStressMaxIteration = this.StressSettings.CancelOnStressMaxIteration;
            //relevant for conjugate gradient methods only
            settings.StressSettings.ResidualTolerance = this.StressSettings.ResidualTolerance;
            settings.StressSettings.CancelAfterFirstConjugate = this.StressSettings.CancelAfterFirstConjugate;
            settings.StressSettings.MaxSolverIterations = this.StressSettings.MaxSolverIterations;
            settings.StressSettings.SolverMaxIteratMethod = this.StressSettings.SolverMaxIteratMethod;
            settings.StressSettings.Parallelize = this.StressSettings.Parallelize;
            settings.StressSettings.ParallelDegree = this.StressSettings.ParallelDegree;
            return settings;
        }
    }
}
