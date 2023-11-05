// --------------------------------------------------------------------------------------------------------------------
// <copyright file="BlockVector.cs" company="Microsoft">
//   (c) Microsoft Corporation.  All rights reserved.
// </copyright>
// <summary>
// MSAGL class for Block vector management.
// </summary>
// --------------------------------------------------------------------------------------------------------------------
using System.Collections.Generic;
using System.Diagnostics;

namespace Microsoft.Msagl.Core.ProjectionSolver
{
    /// <summary>
    /// </summary>
    internal class BlockVector
    {
        internal List<Block> Vector { get; private set; }
        internal int Count { get { return this.Vector.Count; } }
        internal Block this[int index] { get { return this.Vector[index]; } }

        internal BlockVector()
        {
            this.Vector = new List<Block>();
        }

        internal void Add(Block block)
        {
            block.VectorIndex = this.Vector.Count;
            this.Vector.Add(block);
            Debug.Assert(this.Vector[block.VectorIndex] == block, "Inconsistent block.VectorIndex");
        }

        internal void Remove(Block block)
        {
            Debug.Assert(this.Vector[block.VectorIndex] == block, "Inconsistent block.VectorIndex");
            Block swapBlock = this.Vector[this.Vector.Count - 1];
            this.Vector[block.VectorIndex] = swapBlock;
            swapBlock.VectorIndex = block.VectorIndex;
            this.Vector.RemoveAt(this.Vector.Count - 1);
            Debug.Assert((0 == this.Vector.Count) || (block == swapBlock) || (this.Vector[swapBlock.VectorIndex] == swapBlock),
                    "Inconsistent swapBlock.VectorIndex");
            Debug.Assert((0 == this.Vector.Count) || (this.Vector[this.Vector.Count - 1].VectorIndex == (this.Vector.Count - 1)),
                    "Inconsistent finalBlock.VectorIndex");
        }

        /// <summary>
        /// </summary>
        public override string ToString()
        {
            return this.Vector.ToString();
        }
    }
}