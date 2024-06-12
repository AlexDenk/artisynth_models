package artisynth.models.diss.Tests;

import java.awt.Color;
import java.io.File;
import java.io.IOException;

import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.diss.MOTReader;
import artisynth.models.diss.MOTReader.ForceData;
import artisynth.models.diss.MotionTargetController;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.render.RenderProps;

/**
 * Test class to check, whether COP projection of {@link MotionTargetController}
 * works as expected.
 */
@Deprecated
public class COPProjectionTest extends OpenSimTest {
   MechModel mech = new MechModel ();
   RigidBody box;
   FrameMarker copRef1;
   FrameMarker copRef2;
   FrameMarker src;
   MotionTargetController controller;
   ForceData myForces;

   public COPProjectionTest () {

   }

   public COPProjectionTest (String name) throws IOException {
      super (name);
   }

   public static void main (String[] args) throws IOException {

   }

   public void build (String[] args) throws IOException {
      mech.setName ("Model");
      addModel (mech);

      box = RigidBody.createBox ("box", 0.1, 0.1, 0.1, 100);
      RigidTransform3d TCW = new RigidTransform3d (0.4, 0.05, 0.1);
      box.setPose (TCW);
      mech.addRigidBody (box);

      copRef1 = mech.addFrameMarkerWorld (box, new Point3d (0, 0, 0));
      copRef1.setName ("Marker 1");
      copRef2 =
         mech.addFrameMarkerWorld (box, new Point3d (0.378, -0.0075, 0.1277));
      copRef2.setName ("Marker 2");

      src = new FrameMarker ("source");
      src.setFrame (box);
      src.setLocation (new Point3d (-0.05, -0.05, -0.05));
      mech.addFrameMarker (src);

      setupController ();
      setupRenderProps ();
   }

   private ForceData readMOTFile () throws IOException {
      String name = "OpenSimTest/Input/OpenSimTest_forces.mot";
      String path = ArtisynthPath.getSrcRelativePath (OpenSimTest.class, name);
      MOTReader motReader = new MOTReader (new File (path));
      motReader.readData ();
      return motReader.getForceData ();
   }

   private void setupController () throws IOException {
      myForces = readMOTFile ();
      controller =
         new MotionTargetController (mech, "controller", "COPPRojectionTest");
      //controller.addForceData (myForces);
      //controller.addCOPReference (copRef1);
      //controller.addCOPReference (copRef2);
      controller.addMotionTarget (src);
      controller.createProbesAndPanel (this);
      addController (controller);
      // Input probe motion
      NumericInputProbe motion =
         (NumericInputProbe)getInputProbes ().get ("target positions");
      motion.setActive (false);

      // Input probe forces
      NumericInputProbe forces1 = new NumericInputProbe ();
      forces1.setModel (mech);
      forces1.setName ("external force 1");
      double dur =
         myForces.getFrameTime (myForces.numFrames () - 1)
         - myForces.getFrameTime (0);
      forces1.setStartStopTimes (0.0, dur);
      Property[] props = new Property[1];
      props[0] = copRef1.getProperty ("externalForce");
      forces1.setInputProperties (props);

      NumericInputProbe forces2 = new NumericInputProbe ();
      forces2.setModel (mech);
      forces2.setName ("external force 2");
      forces2.setStartStopTimes (0.0, dur);
      props[0] = copRef2.getProperty ("externalForce");
      forces2.setInputProperties (props);

      for (int i = 0; i < myForces.numFrames (); i++) {
         VectorNd force = new VectorNd (3);
         double time = myForces.getFrameTime (i);
         force.add (0, myForces.getData (i, "Right GRF").x);
         force.add (1, myForces.getData (i, "Right GRF").y);
         force.add (2, myForces.getData (i, "Right GRF").z);
         forces1.addData (time, force);
         force.set (0, myForces.getData (i, "Left GRF").x);
         force.set (1, myForces.getData (i, "Left GRF").y);
         force.set (2, myForces.getData (i, "Left GRF").z);
         forces2.addData (time, force);
      }
      addInputProbe (forces1);
      //controller.addInputProbe (forces1);
      forces1.setActive (false);
      addInputProbe (forces2);
      //controller.addInputProbe (forces2);
      forces2.setActive (false);
   }

   private void setupRenderProps () {
      RenderProps.setSphericalPoints (copRef1, 0.01, Color.CYAN);
      RenderProps.setSphericalPoints (copRef2, 0.01, Color.CYAN);
      RenderProps.setSphericalPoints (src, 0.01, Color.PINK);
      controller.getTargetPoints ().forEach (t -> {
         RenderProps.setSphericalPoints (t, 0.01, Color.WHITE);
      });
      // COPRenderer cop = new COPRenderer(1);
      // getMainViewer().addRenderable(cop);
   }
}