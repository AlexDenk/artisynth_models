// Where the model resides in the project tree. Models, that are located 
// within artisynth.models are already visible to the Artisynth compiler.
package artisynth.models.diss.Tests;

// Every additional data that needs to be included to run the code
import java.awt.Color;
import java.io.IOException;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemFactory.FemElementType;
import artisynth.core.femmodels.FemMarker;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.mechmodels.*;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.probes.*;
import artisynth.core.renderables.*;
import artisynth.core.workspace.RootModel;

import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

// The application model (here TestFEM) is a subclass of RootModel
public class FEMTest extends RootModel {
   // ---------------------------Fields of the
   // class----------------------------//

   /*
    * At first, the global properties (fields) of the upcoming model are to be
    * defined. This includes for example the dimensions of the given structure,
    * its material, the number of elements, simulation properties and so on.
    */

   // Define a basic model, that includes all components (FE structures,
   // rigid bodies, ...).
   public MechModel mech = new MechModel ("mech");
   // Define the simulation control properties of this model.
   public MechSystemSolver mechSystemSolver = new MechSystemSolver (null);
   // Create a FEM Model as an instance of the class FemModel3d, which is a
   // subclass of MechModel. The model "testBeam" itself is empty at first.
   public FemModel3d testBeam = new FemModel3d ("testBeam");
   // Create a colourbar, so that stress can be visualised on the mesh surface.
   public ColorBar cbar = new ColorBar ();
   // FEM Structure size (length, width, height) in mm.
   public double[] size = { 100, 10, 10 };
   // FEM structure resolution (number of elements into each dimension).
   public int[] res = { 50, 5, 5 };
   // Create a reference point in the model (FemMarker), that the external
   // force will be applied to.
   public FemMarker mkr = new FemMarker (75, 0, 0);
   // Density of the material (kg/mm^3).
   public double density = 7.85E-6;
   // Material itself with Young's modulus (N/mm^2) and Poisson's ratio.
   public LinearMaterial mat = new LinearMaterial (200E+3, 0.3);
   // Damping coefficient, that is related to particle damping.
   public double damp = 0.1;
   // --------------------------Methods of the class--------------------------//

   // The build method (overrides the RootModel method) to do the model
   // construction. Crucial for the application, when building models.
   @Override // Annotation to the compiler, that the method is overwritten.
   public void build (String[] args) throws IOException {

      /*
       * Calls the inherited method from the superclass. Here it is not
       * necessary, but particularly useful, if a subclass is to be
       * instantiated, that is similar to the superclass, so that the same
       * fields and methods do not need to be defined twice.
       */
      // --------------------------Model building-----------------------------//
      super.build (args);
      // Add the MechModel mech to the list of models in the root model.
      addModel (mech);
      // Set Solver and integrator properties, if needed.
      mechSystemSolver.setIntegrator (Integrator.Trapezoidal);
      mechSystemSolver.setStabilization (PosStabilization.GlobalStiffness);
      // Generate the actual structure for the instance testBeam. Here a beam
      // is created, ranging from -50 to 50 (x) and from -5 to 5 (y and z).
      FemFactory
         .createGrid (
            testBeam, FemElementType.QuadHex, size[0], size[1], size[2], res[0],
            res[1], res[2]);
      // Set the properties of the testBeam instance. Necessary to avoid the
      // cannot create instance from class exception.
      testBeam.setDensity (density);
      testBeam.setParticleDamping (damp);
      testBeam.setMaterial (mat);
      // Set the boundary conditions of testBeam. Fix the most left nodes in
      // space with the lowest x values.
      for (FemNode3d node : testBeam.getNodes ()) {
         if (node.getPosition ().x <= -49.95) {
            node.setDynamic (false);
            RenderProps.setSphericalPoints (node, 0.5, Color.CYAN);
         }
      }
      // Add the instance testBeam to the list of models, but as a component
      // of mech.
      mech.addModel (testBeam);
      // Add the marker to the FEM model as a component of testBeam.
      testBeam.addMarker (mkr);
      // ---------------------------Define Probes-----------------------------//
      // Create an input probe, that will contain the external force data.
      NumericInputProbe exForce =
         new NumericInputProbe (
            testBeam, "markers/0:externalForce",
            PathFinder.getSourceRelativePath (this, "FEMTest/testInput.txt"));
      // Set the name of the input probe and add it to the model.
      exForce.setName ("Force");
      addInputProbe (exForce);
      // Create an output probe, that will contain the mkr displacement data.
      NumericOutputProbe disp =
         new NumericOutputProbe (
            testBeam, "markers/0:position",
            PathFinder.getSourceRelativePath (this, "FEMTest/testOutput.txt"),
            0.1);
      // Set the name of the output probe.
      disp.setName ("Displacement");
      // Set the range of the y-axis in the output diagram.
      disp.setDefaultDisplayRange (0, 3);
      // Set the stopping time for the output probe.
      disp.setStopTime (5);
      // Add the output probe to the model.
      addOutputProbe (disp);
      // -----------------------------Rendering-------------------------------//
      // Render the stress state on the surface
      setRenderProperties (mkr);
      testBeam.setSurfaceRendering (SurfaceRender.Stress);
      testBeam.setStressPlotRanging (Ranging.Auto);
      // Set the properties of the corresponding colourbar.
      // Name of the colourbar.
      cbar.setName ("colorBar");
      // Set the display to a float number with 2 decimal places.
      cbar.setNumberFormat ("%.2f");
      // Initialize the colourbar with 10 ticks.
      cbar.populateLabels (0.0, 0.1, 10);
      // Define, where the colour bar is to be shown (x and y position and also
      // width and height.
      cbar.setLocation (-100, 0.1, 20, 0.8);
      addRenderable (cbar);
   }

   private void setRenderProperties (FemMarker mkr) {
      RenderProps.setSphericalPoints (mkr, 0.5, Color.YELLOW);
   }

   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      // Synchronize color bar/values in case they are changed. Do this *after*
      // super.prerender(), in case values are changed there.
      ColorBar cbar = (ColorBar)(renderables ().get ("colorBar"));
      cbar.setColorMap (testBeam.getColorMap ());
      DoubleInterval range = testBeam.getStressPlotRange ();
      cbar.updateLabels (range.getLowerBound (), range.getUpperBound ());
   }

   // ------------------------All below is unused-----------------------------//
   // auto-generated constructor stub, that has no arguments.
   // Crucial for the application, when reading models from a file.
   public FEMTest () {

   }

   public FEMTest (String name) throws IOException {
      super (name);
      // TODO Auto-generated constructor stub
   }

   public static void main (String[] args) {
      // TODO Auto-generated method stub
   }
}