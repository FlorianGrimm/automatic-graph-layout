﻿using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;

namespace Microsoft.Msagl.Core.Geometry
{
    /// <summary>
    /// Flow fill of columns to some maximum height
    /// </summary>
    public class ColumnPacking<TData> : Packing
    {
        private IEnumerable<RectangleToPack<TData>> orderedRectangles;
        private double maxHeight;

        /// <summary>
        /// Constructor for packing, call Run to do the actual pack.
        /// Each RectangleToPack.Rectangle is updated in place.
        /// Pack rectangles tallest to shortest, left to right until wrapWidth is reached, 
        /// then wrap to right-most rectangle still with vertical space to fit the next rectangle
        /// </summary>
        /// <param name="rectangles"></param>
        /// <param name="maxHeight"></param>
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1026:DefaultParametersShouldNotBeUsed"), System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Design", "CA1006:DoNotNestGenericTypesInMemberSignatures")]
        public ColumnPacking(IEnumerable<RectangleToPack<TData>> rectangles, double maxHeight)
        {
            this.orderedRectangles = rectangles;
            this.maxHeight = maxHeight;
        }

        /// <summary>
        /// Pack columns by iterating over rectangle enumerator until column height exceeds wrapHeight.
        /// When that happens, create a new column at position PackedWidth.
        /// </summary>
        protected override void RunInternal()
        {
            this.PackedHeight = this.PackedWidth = 0;
            double columnPosition = 0;
            double columnHeight = 0;
            foreach (var current in this.orderedRectangles)
            {
                Rectangle r = current.Rectangle;
                if (columnHeight + r.Height > this.maxHeight)
                {
                    columnPosition = this.PackedWidth;
                    columnHeight = 0;
                }
                r = current.Rectangle = new Rectangle(columnPosition, columnHeight,
                        new Point(r.Width, r.Height));
                this.PackedWidth = Math.Max(this.PackedWidth, columnPosition + r.Width);
                columnHeight += r.Height;
                this.PackedHeight = Math.Max(this.PackedHeight, columnHeight);
            }
        }
    }
}
