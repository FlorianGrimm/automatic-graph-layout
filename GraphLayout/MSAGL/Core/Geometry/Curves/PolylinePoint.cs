using System;
using System.Diagnostics.CodeAnalysis;

namespace Microsoft.Msagl.Core.Geometry.Curves {
	/// <summary>
	/// 
	/// </summary>
#if TEST_MSAGL
    [Serializable]
#endif
	public class PolylinePoint {
        /// <summary>
        /// 
        /// </summary>
        private Point point;

		/// <summary>
		/// 
		/// </summary>
		public Point Point {
            get { return this.point; }
            set {
#if SHARPIT
                point = value.Clone();
#else
                this.point = value;
#endif
                if (this.Polyline != null) {
                    this.Polyline.RequireInit();
                }
            }
        }

        private PolylinePoint next;

		/// <summary>
		/// 
		/// </summary>
		public PolylinePoint Next {
            get { return this.next; }
            set {
                this.next = value;
                if (this.Polyline != null) {
                    this.Polyline.RequireInit();
                }
            }
        }

        private PolylinePoint prev;

        /// <summary>
        /// 
        /// </summary>
        public PolylinePoint Prev {
            get { return this.prev; }
            set {
                this.prev = value;
                if (this.Polyline != null) {
                    this.Polyline.RequireInit();
                }
            }
        }

        internal PolylinePoint() {
        }

        /// <summary>
        /// 
        /// </summary>
        public PolylinePoint(Point p) {
#if SHARKPIT
            Point = p.Clone();
#else
            this.Point = p;
#endif
        }

        private Polyline? _Polyline;

        /// <summary>
        /// 
        /// </summary>
        [SuppressMessage("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        public Polyline? Polyline {
            get { return this._Polyline; }
            set { this._Polyline = value; }
        }

		/// <summary>
		/// 
		/// </summary>
		public override string ToString() {
            return this.point.ToString();
        }

        /// <summary>
        /// 
        /// </summary>
        public PolylinePoint? NextOnPolyline {
            get { return this.Polyline?.Next(this); }
        }

        /// <summary>
        /// 
        /// </summary>
        public PolylinePoint? PrevOnPolyline {
            get { return this.Polyline?.Prev(this); }
        }

    }
}