using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Msagl.Core.DataStructures;
using Microsoft.Msagl.Core.Geometry;
using Microsoft.Msagl.Core.Geometry.Curves;
using Microsoft.Msagl.Core.Layout;
using Microsoft.Msagl.Routing;
using Microsoft.Msagl.Routing.ConstrainedDelaunayTriangulation;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.IO;
using Microsoft.Msagl.Routing.Spline.Bundling;
using Microsoft.Msagl.GraphViewerGdi;


namespace Microsoft.Msagl.Core.DataStructures {
    [TestClass]
    public class RbTreeTests {
        [TestMethod]
        public void RbTree_Insert() {
            var sut = new RbTree<int>(Comparer<int>.Default);
            sut.Insert(1);
            Assert.AreEqual(1, sut.Count);
            Assert.IsNotNull(sut.FindFirst(a => a == 1));
            Assert.IsNull(sut.FindFirst(a => a == 2));
            sut.Insert(2);
            Assert.AreEqual(2, sut.Count);
            Assert.IsNotNull(sut.FindFirst(a => a == 1));
            Assert.IsNotNull(sut.FindFirst(a => a == 2));
            Assert.IsNull(sut.FindFirst(a => a == 3));
        }
    }
}
