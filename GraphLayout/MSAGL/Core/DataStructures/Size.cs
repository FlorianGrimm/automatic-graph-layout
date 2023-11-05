namespace Microsoft.Msagl.Core.DataStructures
{


  /// <summary>
  /// Size structure
  /// </summary>
  public struct Size
  {
        private double width;
    /// <summary>
    /// width
    /// </summary>
    public double Width
    {
      get { return this.width; }
      set { this.width = value; }
    }

        private double height;
    /// <summary>
    /// Height
    /// </summary>
    public double Height
    {
      get { return this.height; }
      set { this.height = value; }
    }

    /// <summary>
    /// constructor
    /// </summary>
    /// <param name="width"></param>
    /// <param name="height"></param>
    public Size(double width, double height)
    {
      this.width = width;


      this.height = height;

    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="s"></param>
    /// <param name="d"></param>
    /// <returns></returns>
    public static Size operator /(Size s, double d) { return new Size(s.Width / d, s.Height / d); }


    /// <summary>
    /// 
    /// </summary>
    /// <param name="s"></param>
    /// <param name="d"></param>
    /// <returns></returns>
    public static Size operator *(Size s, double d) { return new Size(s.Width * d, s.Height * d); }


      /// <summary>
      /// padding the size ( from both sides!)
      /// </summary>
      /// <param name="padding"></param>
      public void Pad(double padding) {
            this.width += 2*padding;
            this.height += 2*padding;
      }
  }


}
