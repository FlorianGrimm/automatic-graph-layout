using System;
using System.Collections.Generic;
using System.Linq;
using Microsoft.Msagl.DebugHelpers.Persistence;

namespace Microsoft.Msagl.DebugHelpers {
    internal class CurveStream {
        private string data;
        private CurveStreamElement[] curveStreamElements;
        private int offset;

        internal CurveStream(string curveData) {
            this.data =curveData;
            this.Init();
        }

        private void Init() {
            this.data = this.data.Trim();
            var blocks = this.data.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
            this.curveStreamElements = RefineBlocks(blocks).ToArray();

        }

        private static IEnumerable<CurveStreamElement> RefineBlocks(string[] blocks) {
            foreach (string block in blocks) {
                var ch = block[0];
                if (Char.IsLetter(ch)) {
                    yield return new CharStreamElement(ch);
                    if (block.Length > 1) {
                        double res;
                        if (Double.TryParse(block.Substring(1), out res)) {
                            yield return new DoubleStreamElement(res);
                        } else {
                            yield return null;
                        }
                    }
                } else {
                    double res;
                    if (Double.TryParse(block, out res)) {
                        yield return new DoubleStreamElement(res);
                    } else {
                        yield return null;
                    }
                }
            }
        }

        internal CurveStreamElement GetNextCurveStreamElement() {
            if (this.offset >= this.curveStreamElements.Length) {
                return null;
            }

            return this.curveStreamElements[this.offset++];
        }

        internal CurveStreamElement PickNextCurveStreamElement() {
            if (this.offset >= this.curveStreamElements.Length) {
                return null;
            }

            return this.curveStreamElements[this.offset];
        }
    }
}