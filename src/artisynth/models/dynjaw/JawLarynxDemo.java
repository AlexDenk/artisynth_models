package artisynth.models.dynjaw;

import java.awt.Color;
import java.io.IOException;

import artisynth.core.gui.ControlPanel;
import artisynth.core.mechmodels.BodyConnector;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.mechmodels.SegmentedPlanarConnector;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.workspace.DriverInterface;
import artisynth.core.workspace.RootModel;
import maspack.matrix.Point3d;
import maspack.matrix.VectorNi;
import maspack.render.RenderProps;
import maspack.util.NumberFormat;

public class JawLarynxDemo extends JawDemo {

   /**
    * Monitor that lets us look at how the engagement of the
    * unilateral constraints evolves over time. Used for debugging
    * and simulation tuning.
    */
   protected class EngagedMonitor extends MonitorBase {
      VectorNi myEngaged = new VectorNi();

      VectorNi computeEngaged() {
         VectorNi engaged = new VectorNi();
         for (BodyConnector c : myJawModel.bodyConnectors()){
            if (c instanceof PlanarConnector) {
               engaged.append (Math.abs(((PlanarConnector)c).getEngaged()));
            }
            else if (c instanceof SegmentedPlanarConnector) {
               engaged.append (Math.abs(((SegmentedPlanarConnector)c).getEngaged()));
            }
         }
         return engaged;
      }

      EngagedMonitor() {
         myEngaged = computeEngaged();
      }
      
      public void apply (double t0, double t1) {
         VectorNi engaged = computeEngaged();
         NumberFormat fmt = new NumberFormat("%6.4f");
         if (!myEngaged.equals(engaged)) {
            System.out.println (fmt.format(t1) + "  " + engaged);
            myEngaged = engaged;
         }
      }
   }

   public JawLarynxDemo() {
      super();
   }

   @Override
   public void build (String[] args) throws IOException {
      super.build (args);

      //setIncisorVisible();
      String[] hiddenBodies = new String[] { "cranium", "pharynx", "vertebrae" };
      for (int i = 0; i < hiddenBodies.length; i++) {
	 RigidBody body = myJawModel.rigidBodies().get(hiddenBodies[i]);
	 if (body != null) RenderProps.setVisible(body, false);
      }

      RenderProps.setFaceColor(myJawModel.bodyConnectors().get("RTMJ"),
	    Color.PINK);
      RenderProps.setFaceColor(myJawModel.bodyConnectors().get("LTMJ"),
	    Color.PINK);

      //setIncisorVisible();
   }

   public void setupJawModel()
   {
      loadBoluses();
      
      myJawModel.showMeshFaces(true);

      myJawModel.setThMemStiffness(385.6);
      myJawModel.setCtrMemStiffness(385.6);
      
      myJawModel.setIntegrator(Integrator.SymplecticEuler);
      myJawModel.setMaxStepSize(0.0001);
      
      setDampingParams(10, 100, 0.001);
      setJawDamping (20, 200);

   }	
   
   protected void loadModel() {
      // create jaw model
      try {
	 myJawModel = new JawModel("jawmodel", /* fixedLaryngeal */false, /* useComplexJoint */
	       true, /* curvJoint */true);
      } catch (Exception e) {
	 e.printStackTrace();
	 System.err.println("RootJaw() - error to instantiating AmiraJaw");
      }

      pokesToControl = new String[] {
	    // "lCondylarAngle",
	    // "rCondylarAngle",
	    // "lCondylarCant",
	    // "rCondylarCant",
	    "lMedWallAngle", "rMedWallAngle", "lPostWallAngle",
	    "rPostWallAngle", "lBiteAngle", "rBiteAngle", "lBiteCant",
	    "rBiteCant" };
   }

   public void loadControlPanel(RootModel root) {
      String panelNames[] = new String[] { "misc", "damping", "muscles",
	    "joints" };
      loadControlPanel(root, panelNames);
   }
   
   public void addJawOptions(ControlPanel panel)
   {
	  if (panel == null)
		 return;
	  JawPanel.createJawLarynxPanel(myJawModel, panel);
   }

   public void attach(DriverInterface driver) {
      workingDirname = "data/rightchew/";
      probesFilename = "rightchew.art";

      super.attach(driver);

      this.setViewerEye(new Point3d(0.0, -268.0, -23.0));
      this.setViewerCenter(new Point3d(0.0, 44.0, 55.0));
      setIncisorVisible();

      addBreakPoint(0.575);
   }

}
