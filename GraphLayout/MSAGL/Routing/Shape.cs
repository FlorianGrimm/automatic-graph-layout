//
// Shape.cs
// MSAGL Shape class for Rectilinear Edge Routing.
//
// Copyright Microsoft Corporation.

using System;
using System.Collections.Generic;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Core;
using System.Diagnostics;

namespace Microsoft.Msagl.Routing {
    /// <summary>
    /// A shape wrapping an ICurve, providing additional information.
    /// </summary>
    [DebuggerDisplay("Shape = {UserData}")]
    public class Shape {
        private readonly Set<Shape> parents = new Set<Shape>();
        ///<summary>
        /// shape parents
        ///</summary>
        public IEnumerable<Shape> Parents {
            get { return this.parents; }
        }

        private readonly Set<Shape> children = new Set<Shape>();
        /// <summary>
        /// shape children
        /// </summary>
        public IEnumerable<Shape> Children {
            get { return this.children; }
        }
        /// <summary>
        /// The curve of the shape.
        /// </summary>
        public virtual ICurve BoundaryCurve { 
            get { return this._BoundaryCurve; }
            set { this._BoundaryCurve = value; }
        }

        private ICurve _BoundaryCurve;

        /// <summary>
        /// The bounding box of the shape.
        /// </summary>
        public Rectangle BoundingBox { get { return this.BoundaryCurve.BoundingBox; } }

        /// <summary>
        /// The set of Ports for this obstacle, usually RelativePorts.  In the event of overlapping
        /// obstacles, this identifies the obstacle to which the port applies.
        /// </summary>
        public Set<Port> Ports { get { return this.ports; } }

        private readonly Set<Port> ports = new Set<Port>();

        /// <summary>
        /// A location for storing user data associated with the Shape.
        /// </summary>
        public object UserData { get; set; }

        /// <summary>
        /// Default constructor.
        /// </summary>
        public Shape() : this (null) {
            this.UserData = string.Empty;
        }

        /// <summary>
        /// Constructor taking the ID and the curve of the shape.
        /// </summary>
        /// <param name="boundaryCurve"></param>
        public Shape(ICurve? boundaryCurve) {
            this._BoundaryCurve = boundaryCurve ?? NilCurve.Empty;     // RelativeShape throws an exception on BoundaryCurve_set so set _boundaryCurve directly.
        }

        /// <summary>
        /// A group is a shape that has children.
        /// </summary>
        public bool IsGroup {
            get { return this.children.Count > 0; }
        }

        internal bool IsTransparent { get; set; }

        internal IEnumerable<Shape> Descendants {
            get {
                var q = new Queue<Shape>();
                foreach (var shape in this.Children) {
                    q.Enqueue(shape);
                }

                while (q.Count > 0) {
                    var sh = q.Dequeue();
                    yield return sh;
                    foreach (var shape in sh.Children) {
                        q.Enqueue(shape);
                    }
                }
            }
        }

        internal IEnumerable<Shape> Ancestors {
            get {
                var q = new Queue<Shape>();
                foreach (var shape in this.Parents) {
                    q.Enqueue(shape);
                }

                while (q.Count > 0) {
                    var sh = q.Dequeue();
                    yield return sh;
                    foreach (var shape in sh.Parents) {
                        q.Enqueue(shape);
                    }
                }
            }
        }
        ///<summary>
        /// Adds a parent. A shape can have several parents
        ///</summary>
        ///<param name="shape"></param>
        public void AddParent(Shape shape) {
            ValidateArg.IsNotNull(shape, "shape");
            this.parents.Insert(shape);
            shape.children.Insert(this);
        }

        ///<summary>
        ///</summary>
        ///<param name="shape"></param>
        public void AddChild(Shape shape) {
            ValidateArg.IsNotNull(shape, "shape");
            shape.parents.Insert(this);
            this.children.Insert(shape);
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="shape"></param>
        public void RemoveChild(Shape shape) {
            ValidateArg.IsNotNull(shape, "shape");
            this.children.Remove(shape);
            shape.parents.Remove(this);
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="shape"></param>
        public void RemoveParent(Shape shape) {
            ValidateArg.IsNotNull(shape, "shape");
            this.parents.Remove(shape);
            shape.children.Remove(this);
        }

#if TEST_MSAGL
        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return this.UserData == null ? "null" : this.UserData.ToString();
        }
#endif 
    }
}
