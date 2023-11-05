//
// EventQueue.cs
// MSAGL class for Rectilinear Edge Routing between two nodes.
//
// Copyright Microsoft Corporation.

using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Routing.Spline.ConeSpanner;
using RectRout = Microsoft.Msagl.Routing.Rectilinear;

namespace Microsoft.Msagl.Routing.Rectilinear {
    using DebugHelpers;

    /// <summary>
    /// Wrap the tree of events.
    /// </summary>
    internal class EventQueue : IComparer<SweepEvent> {
        private ScanDirection scanDirection;
        private readonly BinaryHeapWithComparer<SweepEvent> eventTree;

        internal EventQueue() {
            this.eventTree = new BinaryHeapWithComparer<SweepEvent>(this);
        }
        
        internal void Reset(ScanDirection scanDir) {
            Debug.Assert(0 == this.eventTree.Count, "Stray events in EventQueue.Reset");
            this.scanDirection = scanDir;
        }

        internal void Enqueue(SweepEvent evt) {
            this.DevTraceInfo(1, "Enqueueing {0}", evt);
            this.eventTree.Enqueue(evt);
        }

        internal SweepEvent Dequeue() {
            SweepEvent evt = this.eventTree.Dequeue();
            this.DevTraceInfo(1, "Dequeueing {0}", evt);
            return evt;
        }

        internal int Count {
            get { return this.eventTree.Count; }
        }

        #region IComparer<SweepEvent>
        /// <summary>
        /// For ordering events in the event list.
        /// Assuming vertical sweep (sweeping up from bottom, scanning horizontally) then order events
        /// first by lowest Y coord, then by lowest X coord (thus assuming we use Cartesian coordinates
        /// (negative is downward == bottom).
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public int Compare(SweepEvent lhs, SweepEvent rhs) {
            if (lhs == rhs) {
                return 0;
            }
            if (lhs == null) {
                return -1;
            }
            if (rhs == null) {
                return 1;
            }

            // First see if it's at the same scanline level (perpendicular coordinate).
            int cmp = this.scanDirection.ComparePerpCoord(lhs.Site, rhs.Site);
            if (0 == cmp) {
                // Event sites are at the same scanline level. Make sure that any reflection events are lowest (come before
                // any side events, which could remove the side the reflection event was queued for).  We may have two
                // reflection events at same coordinate, because we enqueue in two situations: when a side is opened,
                // and when a side that is within that side's scanline-parallel span is closed.
                bool lhsIsNotReflection = !(lhs is BasicReflectionEvent);
                bool rhsIsNotReflection = !(rhs is BasicReflectionEvent);
                cmp = lhsIsNotReflection.CompareTo(rhsIsNotReflection);

                // If the scanline-parallel coordinate is the same these events are at the same point.
                if (0 == cmp) {
                    cmp = this.scanDirection.CompareScanCoord(lhs.Site, rhs.Site);
                }
            }   
            return cmp;
        }
        #endregion // IComparer<SweepEvent>

        #region DevTrace
#if DEVTRACE
        readonly DevTrace eventQueueTrace = new DevTrace("Rectilinear_EventQueueTrace", "EventQueue");
#endif // DEVTRACE

        [Conditional("DEVTRACE")]
        [System.Diagnostics.CodeAnalysis.SuppressMessage("Microsoft.Performance", "CA1822:MarkMembersAsStatic")]
        private void DevTraceInfo(int verboseLevel, string format, params object[] args) {
#if DEVTRACE
            eventQueueTrace.WriteLineIf(DevTrace.Level.Info, verboseLevel, format, args);
#endif // DEVTRACE
        }
        #endregion // DevTrace
    }
}
