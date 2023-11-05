using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;

namespace Microsoft.Msagl.Miscellaneous.Routing
{
    public class ShortestPathTree
    {
        private Set<Point> _points = new Set<Point>();
        private Dictionary<Point, Point> _paret = new Dictionary<Point, Point>();
        private Point _root;

        public ShortestPathTree(Point root)
        {
            this._root = root;
            this._points.Insert(root);
            this._paret[this._root] = this._root;
        }

        public void AddPathToTree(List<Point> path)
        {
            if (!path.First().Equals(this._root)) {
                path.Reverse();
            }

            if (!path.First().Equals(this._root)) {
                return;
            }

            int i;

            for (i = path.Count - 1; i >= 0; i--)
            {
                if (this._points.Contains(path[i])) {
                    break;
                }
            }

            for (int j = path.Count - 1; j > i; j--)
            {
                if (this._points.Contains(path[j]))
                {
                    // shouldn't happen!
                    break;
                }
                this._points.Insert(path[j]);
                this._paret[path[j]] = path[j - 1];
            }
        }

        public List<Point> GetPathFromRoot(Point s)
        {
            var path = new List<Point>();
            if (!this._points.Contains(s)) {
                return null;
            }

            for (Point p = s; !p.Equals(this._root); p = this._paret[p])
            {
                path.Insert(0,p);
            }
            path.Insert(0, this._root);
            return path;
        }
    }
}
