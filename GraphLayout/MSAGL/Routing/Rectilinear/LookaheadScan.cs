//
// LookaheadScan.cs
// MSAGL class for handling lookahead reflections in Rectilinear Edge Routing.
//
// Copyright Microsoft Corporation.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using RectRout = Microsoft.Msagl.Routing.Rectilinear;
using Microsoft.Msagl.Core;

namespace Microsoft.Msagl.Routing.Rectilinear {
    /// <summary>
    /// For lookahead points, we record the point of the intersection on the reflecting side, then
    /// whenever we load a side, we check for active lookahead lines within this range.  Since we
    /// are just intersecting rays, we only care about the X (H scan) or Y (V scan) coordinate.
    /// </summary>
    internal class LookaheadScan : IComparer<BasicReflectionEvent> {
        private readonly RbTree<BasicReflectionEvent> eventTree;
        private readonly Func<BasicReflectionEvent, bool> findFirstPred;
        private readonly ScanDirection scanDirection;
        private readonly List<BasicReflectionEvent> staleSites = new List<BasicReflectionEvent>();
        private Point findFirstPoint;

        internal LookaheadScan(ScanDirection scanDir) {
            this.scanDirection = scanDir;
            this.eventTree = new RbTree<BasicReflectionEvent>(this);
            this.findFirstPred = new Func<BasicReflectionEvent, bool>(n => this.CompareToFindFirstPoint(n.Site) >= 0);
        }

        internal void Add(BasicReflectionEvent initialSite) {
            // Assert we can't find it - subsumption should have taken care of that.
            Debug.Assert(null == this.Find(initialSite.Site), "Should not add the same Lookahead coordinate twice");
            this.eventTree.Insert(initialSite);
        }

        // Buffer up the events that are known to be stale - that is, will never queued as events because the
        // event-load intersection is the same as the site.

        internal void MarkStaleSite(BasicReflectionEvent siteEvent) {
            this.staleSites.Add(siteEvent);
        }

        internal void RemoveStaleSites() {
            int cSites = this.staleSites.Count; // for (;;) is faster than IEnumerator for Lists
            if (cSites > 0) {
                for (int ii = 0; ii < cSites; ++ii) {
                    this.RemoveExact(this.staleSites[ii]);
                }
                this.staleSites.Clear();
            }
        }

        internal void RemoveSitesForFlatBottom(Point low, Point high) {
            for (RBNode<BasicReflectionEvent> node = this.FindFirstInRange(low, high);
                 null != node;
                 node = this.FindNextInRange(node, high)) {
                this.MarkStaleSite(node.Item);
            }
            this.RemoveStaleSites();
        }

        internal RBNode<BasicReflectionEvent> Find(Point site) {
            return this.FindFirstInRange(site, site);
        }

        internal bool RemoveExact(BasicReflectionEvent initialSite) {
            RBNode<BasicReflectionEvent> node = this.eventTree.Find(initialSite);
            if (null != node) {
                if (node.Item.Site == initialSite.Site) {
                    this.eventTree.DeleteNodeInternal(node);
                    return true;
                }
            }
            return false;
        }

        internal RBNode<BasicReflectionEvent> FindFirstInRange(Point low, Point high) {
            // We only use FindFirstPoint in this routine, to find the first satisfying node,
            // so we don't care that we leave leftovers in it.
            this.findFirstPoint = low;
            RBNode<BasicReflectionEvent> nextNode = this.eventTree.FindFirst(this.findFirstPred);

            if (null != nextNode) {
                // It's >= low; is it <= high?
                if (this.Compare(nextNode.Item.Site, high) <= 0) {
                    return nextNode;
                }
            }
            return null;
        }

        private int CompareToFindFirstPoint(Point treeItem) {
            return this.Compare(treeItem, this.findFirstPoint);
        }

        internal RBNode<BasicReflectionEvent> FindNextInRange(RBNode<BasicReflectionEvent> prev, Point high) {
            RBNode<BasicReflectionEvent> nextNode = this.eventTree.Next(prev);
            if ((null != nextNode) && (this.Compare(nextNode.Item.Site, high) <= 0)) {
                return nextNode;
            }
            return null;
        }

        #region IComparer<BasicReflectionEvent>

        /// <summary>
        /// For ordering Points in the lookahead list.  We just care about the coordinate that changes
        /// parallel to the scanline, so for vertical sweep (sweeping up from bottom, scanning
        /// horizontally) then order points by X only, else by Y only.
        /// </summary>
        /// <param name="lhs"></param>
        /// <param name="rhs"></param>
        /// <returns></returns>
        public int Compare(BasicReflectionEvent lhs, BasicReflectionEvent rhs) {
            ValidateArg.IsNotNull(lhs, "lhs");
            ValidateArg.IsNotNull(rhs, "rhs");
            return this.scanDirection.CompareScanCoord(lhs.Site, rhs.Site);
        }

        internal int Compare(Point lhs, Point rhs) {
            return this.scanDirection.CompareScanCoord(lhs, rhs);
        }

        #endregion // IComparer<BasicReflectionEvent>
    }
}