using System;

namespace Microsoft.Msagl.GraphmapsWithMesh
{

    /*
     * This class is not used for the current implementation
     * This was written in the previous one when we were trying to route with 
     * regular grid and then recursively found the stainer trees for the 
     * components of different zoomlevel
     */

    public class ComponentCollection
    {
        internal int NumOfComponents;
        public int NumOfAliveComponents;
        public Component[] C; //componnent holder

        public ComponentCollection(Vertex[] w, int numOfv, Component givenComponent, int givenLevel)
        {   //initially every vertex is a single component
            int k = 0;
            this.C = new Component[numOfv + 1];
            for (int index = 1; index <= numOfv; index++)
            {

                w[index].CId = 0;
                if (w[index].Weight == 0 || w[index].ZoomLevel != givenLevel) {
                    continue;
                }

                //NODE OVARLAPS WITH THE RAILS// SO IGNORE THE NODE
                if (givenComponent != null && givenComponent.V.Contains(w[index])) {
                    continue;
                }

                k++;
                this.C[k] = new Component { CId = k };
                this.C[k].V.Add(w[index]);
                this.C[k].Dead = false;
                w[index].CId = this.C[k].CId;
                this.NumOfComponents++;
            }
            if (givenComponent != null)
            {
                k++;
                this.C[k] = givenComponent;
                this.C[k].CId = k;
                this.C[k].Dist = 0;
                this.C[k].Dead = false;
                foreach (Vertex x in this.C[k].V)
                {
                    x.CId = this.C[k].CId;
                }
                this.NumOfComponents++;
            }
            this.NumOfAliveComponents = this.NumOfComponents;

        }

    }
}
