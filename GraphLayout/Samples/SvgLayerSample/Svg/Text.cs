using System.Xml;
using SystemDrawing = System.Drawing;

namespace SvgLayerSample.Svg {
    public class Text : SvgElement {
        public string Content { get; set; }
        public double TextWidth { get; set; }
        public double TextHeight { get; set; }

        private SystemDrawing.Font _font = new SystemDrawing.Font(SystemDrawing.FontFamily.GenericSerif, 16);
        public SystemDrawing.Font Font { get { return _font; }
            set {
                _font = value;
                (this.TextWidth, this.TextHeight) = CalculateTextWidth(this.Content);
            }
        } 

        public Text(string text) {
            this.Content = text;
            (this.TextWidth, this.TextHeight) = CalculateTextWidth(this.Content);
        }

        public override string ToString() {
            return $"<text x='{X}' y='{Y}'>{Content}</text>";
        }

        private (float Width, float Height) CalculateTextWidth(string text) {
            using (SystemDrawing.Bitmap bmp = new SystemDrawing.Bitmap(1, 1))
            using (SystemDrawing.Graphics g = SystemDrawing.Graphics.FromImage(bmp)) {
                var measurements = g.MeasureString(text, this.Font);
                return (measurements.Width, measurements.Height);
            }
        }

        public override void WriteTo(XmlWriter writer) {
            writer.WriteRaw(this.ToString());
        }
    }
}
