package artisynth.models.diss.Tests;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.Collectors;

import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;

import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.ConnectorForceRenderer;
import artisynth.core.inverse.ForceTarget;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.FrameExciter.WrenchComponent;
import artisynth.core.inverse.MotionTargetTerm;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.Thelen2003AxialMuscle;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionBehaviorList;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.FrameTarget;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.probes.DataFunction;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericControlProbe;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.TRCReader;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.diss.ContactMonitor;
import artisynth.models.diss.CoordinateData;
import artisynth.models.diss.MOTReader;
import artisynth.models.diss.ForceData;
import artisynth.models.diss.MarkerMapping;
import artisynth.models.diss.MotionTargetController;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.interpolation.Interpolation;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.render.Viewer.RotationMode;
import maspack.spatialmotion.Wrench;
import maspack.util.Clonable;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

/**
 * @author Alexander Denk Copyright (c) 2023-2024 <br>
 * (UDE) University of Duisburg-Essen <br>
 * Chair of Mechanics and Robotics <br>
 * alexander.denk@uni-due.de
 */

public class OpenSimTest extends RootModel {
   // ----------------------------Instance Fields-------------------------------
   // All rigid bodies in the model
   RenderableComponentList<RigidBody> myBodies = null;
   // Calculated generalized angles
   CoordinateData myCoords;
   // Experimental force data
   ForceData myForces;
   // A list of all joints in the model
   RenderableComponentList<JointBase> myJoints;
   // All markers in the model
   RenderableComponentList<FrameMarker> myMarkers = null;
   // Current model
   MechModel myMech = new MechModel ();
   // All finite element meshes in the model
   RenderableComponentList<FemModel3d> myMeshes = null;
   // Experimental and model marker names and weights
   MarkerMapping myMap;
   // Experimental marker trajectories
   MarkerMotionData myMotion;
   // A list of each muscle in the model
   List<MultiPointMuscle> myMuscles = new ArrayList<MultiPointMuscle> ();
   // Name of the current working directory
   String myName = null;
   // Scale of the model.
   int myScale;

   // ----------------------------Nested classes--------------------------------

   public class MomentArmFunction implements DataFunction, Clonable {
      // Frame used for moment arm calculation
      Frame myFrame;
      // cop identifier (left or right)
      String mySide;
      // PrintWriter to write computed wrenches to a file
      PrintWriter writer;
      // Name of the file, where computed wrenches are written to
      String msgName;
      // Path to the file, where computed wrenches are written to
      String msgPath;

      public MomentArmFunction (Frame frame, String side) {
         this.myFrame = frame;
         this.mySide = side;
         // initialize writer
         this.msgName = myName + "/Output/" + myName + "_message_file.txt";
         this.msgPath =
            ArtisynthPath
               .getSrcRelativePath (OpenSimTest.class, msgName).toString ();
         try {
            writer = new PrintWriter (new FileWriter (msgPath, true));
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      }

      public Object clone () throws CloneNotSupportedException {
         return super.clone ();
      }

      @Override
      public void eval (VectorNd vec, double t, double trel) {
         Vector3d ref = myFrame.getPosition ();
         int frame = myForces.getFrame (t);
         // calculate moment arm from current calcn position to cop
         Vector3d cop = myForces.getData (frame, mySide + " COP");
         Vector3d arm = new Vector3d ();
         arm.sub (cop, ref);
         // calculate resulting moment
         Vector3d grf = new Vector3d ();
         vec.getSubVector (new int[] { 0, 1, 2 }, grf);
         Vector3d grm = new Vector3d ();
         vec.getSubVector (new int[] { 3, 4, 5 }, grm);
         Vector3d momRes = new Vector3d ();
         momRes.cross (arm, grf);
         momRes.add (grm);
         // apply wrench
         Wrench wrench =
            new Wrench (
               vec.get (0), vec.get (1), vec.get (2), momRes.x, momRes.y,
               momRes.z);
         myFrame.setExternalForce (wrench);
         // add wrench to message file
         writeToFile (wrench);
      }

      private void writeToFile (Wrench wrench) {
         StringBuilder message = new StringBuilder ();
         message
            .append ("\nCOMPUTED WRENCH FOR: ")
            .append (myFrame.getName ().toUpperCase ()).append ("\n");
         message.append (wrench.toString ("%.3f")).append ("\n");
         writer.print (message);
         writer.flush ();
      }
   }

   // -----------------------------Constructors---------------------------------
   public OpenSimTest () {
   }

   public OpenSimTest (String name) throws IOException {
      super (name);
   }

   // --------------------------Static Methods----------------------------------
   public static void main (String[] args) throws IOException {
   }

   // -------------------------Instance Methods---------------------------------
   
   //TODO: Subdivide step 1 (gen coords) and step 2 (dynamics) in two subclasses
   
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      // Get model name specifier from user
      //JFileChooser fc = new JFileChooser ();
      //myName = getNameFromFileDiaglog (fc);
      myName = "OpenSimTest";
      myMech.setName (myName);
      addModel (myMech);
      setSimulationProperties ();
      initializeOsim (myName, myScale);
      // initializeFEM(myName, myScale);
      CollisionManager collMan = myMech.getCollisionManager ();
      setContactProps (collMan);
      defineIOProbes (myName, myScale);
      setRenderProps (collMan, myScale);
      writeInputToFile (myName);
   }

   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      // Synchronize color bar/values in case they are changed. Do this *after*
      // super.prerender(), in case values are changed there.
      // ColorBar cbar = (ColorBar)(renderables().get("colorBar"));
      // cbar.setColorMap(femur.getColorMap());
      // DoubleInterval range = femur.getStressPlotRange();
      // cbar.updateLabels(range.getLowerBound(), range.getUpperBound());
   }

   /**
    * Sets the rendering properties of every body within the root model.
    */
   public void setBodyRenderProps () {
      // Hide each rigid body personal coordinate system.
      myBodies.forEach (body -> {
         // CS is stored within the first entry of the underlying mesh
         // component list "frame_geometry"
         MeshComponentList<?> components = body.getMeshComps ();
         components.forEach (c -> {
            if (c.getName ().equals ("frame_geometry")) {
               RenderProps.setVisible (c, false);
            }
         });
      });
      // Set smooth shading for rigid bodies
      RenderProps.setShading (myBodies, Shading.SMOOTH);
   }

   /**
    * Sets the rendering properties of every body connector within the root
    * model.
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setConnectorRenderProps (int scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      List<PlanarConnector> con = new ArrayList<PlanarConnector> ();
      myMech.bodyConnectors ().forEach (c -> {
         con.add ((PlanarConnector)c);
      });
      List<ConnectorForceRenderer> rend =
         new ArrayList<ConnectorForceRenderer> ();
      RenderProps props = new RenderProps ();
      props.setLineStyle (LineStyle.CYLINDER);
      props.setLineRadius (25 * scale / 1000);
      props.setLineColor (Color.GREEN);
      con.forEach (c -> {
         RenderProps.setSphericalPoints (c, 0.01 * scale, Color.CYAN);
         RenderProps.setFaceStyle (c, FaceStyle.FRONT);
         rend.add (new ConnectorForceRenderer (c));
         int end = rend.size ();
         rend.get (end - 1).setRenderProps (props);
         rend.get (end - 1).setArrowSize (scale / 1000);
         addMonitor (rend.get (end - 1));
      });
   }

   /**
    * Sets the rendering properties of every collision response within the root
    * model.
    * 
    * @param coll
    * {@link CollisionManager}
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setContactRenderProps (CollisionManager coll, int scale) {
      if (!(coll instanceof CollisionManager))
         throw new IllegalArgumentException (
            "Must use an object of class CollisionManager");
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      // Define body - body contact render props
      coll.setDrawIntersectionPoints (true);
      coll.setDrawContactForces (true);
      coll.setDrawFrictionForces (true);
      coll.setContactForceLenScale (scale / 1000);
      RenderProps.setVisible (coll, true);
      RenderProps.setSolidArrowLines (coll, 0.01 * scale, Color.BLUE);
      RenderProps.setSphericalPoints (coll, 0.01 * scale, Color.CYAN);
   }

   /**
    * Sets the rendering properties of every model marker within the root model.
    * 
    */
   public void setMarkerRendering () {
      myMarkers.forEach (m -> {
         RenderProps.setPointColor (m, Color.PINK);
      });
      // Access source and target points of the motion target controller
      ComponentListView<Controller> controllers = getControllers ();
      if (controllers.size () == 0)
         return;
      TrackingController controller =
         (TrackingController)controllers.get ("Motion controller");
      controller.getTargetPoints ().forEach (c -> {
         RenderProps.setPointColor (c, Color.WHITE);
      });

   }

   /**
    * Sets the rendering properties of every muscle within the root model.
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMuscleRenderProps (int scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      myMuscles.forEach (msc -> {
         RenderProps.setLineColor (msc, Color.RED.darker ());
         RenderProps.setShading (msc, Shading.SMOOTH);
         RenderProps.setLineStyle (msc, LineStyle.SPINDLE);
         RenderProps.setLineRadius (msc, 0.005 * scale);
         msc.setExcitationColor (Color.GREEN);
      });
      myMech.setMaxColoredExcitation (1.0);
   }

   /**
    * Sets the render properties of the current model regarding bodies, muscles,
    * meshes, contact and viewer props.
    * 
    * @param coll
    * {@link CollisionManager}
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setRenderProps (CollisionManager coll, int scale) {
      if (!(coll instanceof CollisionManager))
         throw new IllegalArgumentException (
            "Must use an object of class CollisionManager");
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      setContactRenderProps (coll, scale);
      setBodyRenderProps ();
      setMuscleRenderProps (scale);
      setMarkerRendering ();
      // setSurfaceRenderProps ();
      setViewerProps ();
   }

   /**
    * Sets the render properties of the colour bars within the root model.
    */
   public void setSurfaceRenderProps () {
      ColorBar cbar = new ColorBar (null);
      // Stress surface rendering
      myMeshes.forEach (mesh -> {
         mesh.setSurfaceRendering (SurfaceRender.Stress);
         mesh.setStressPlotRanging (Ranging.Auto);
      });
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

   /**
    * Sets the viewer properties of the current viewer.
    */
   public void setViewerProps () {
      getMainViewer ().setOrthographicView (true);
      getMainViewer ().setRotationMode (RotationMode.CONTINUOUS);
      setDefaultViewOrientation (AxisAlignedRotation.X_Y);
      mergeAllControlPanels (true);
   }

   /**
    * Writes all model parameters to a {@code myName_input.txt} in the Output
    * directory under the current working directory.
    * 
    * @param myName
    * Name specifier of the current working directory
    * @throws IOException
    * if there's an issue writing to the file
    */
   public void writeInputToFile (String myName) throws IOException {
      if (!(myName instanceof String)) {
         throw new IllegalArgumentException ("Input must be of type String");
      }
      // Write the model Input to the input file via the writer.
      String inputName = myName + "/Output/" + myName + "_input_file.txt";
      String inputPath =
         ArtisynthPath.getSrcRelativePath (this, inputName).toString ();
      try (PrintWriter writer =
         new PrintWriter (new FileWriter (inputPath, false))) {
         // Append header
         StringBuilder output = new StringBuilder ();
         output
            .append (
               "%%------------------------ INPUT FILE ------------------------%%\n")
            .append (
               "%% Author: Alexander Denk, Copyright (c) 2024                 %%\n")
            .append (
               "%% (UDE) University of Duisburg-Essen                         %%\n")
            .append (
               "%% Chair of Mechanics and Robotics                            %%\n")
            .append (
               "%% alexander.denk@uni-due.de                                  %%\n")
            .append (
               "%%------------------------------------------------------------%%\n");
         MechSystemSolver solver = myMech.getSolver ();
         writeSolverInfo (output, solver);
         writePhysicsInfo (output, myMech);
         String modelPath = myName + "/gait2392_simbody_scaled.osim";
         writeModelInfo (output, modelPath, myBodies);
         writeJointInfo (output, myJoints);
         writeMuscleInfo (output, myMuscles);
         writeFEMInfo (output, myMeshes);
         TrackingController controller =
            (TrackingController)getControllers ().get ("Motion controller");
         if (controller != null) {
            writeProbesInfo (
               output, controller, myMotion, myMap, myForces, myMarkers);
         }
         CollisionManager coll = myMech.getCollisionManager ();
         CollisionBehaviorList behav = coll.behaviors ();
         if (coll != null) {
            writeContactInfo (output, coll, behav);
         }
         writer.print (output.toString ());
      }
      catch (IOException ex) {
         System.err.println(ex.getMessage());
      }
   }

   // --------------------------Private Instance Methods------------------------
   /**
    * Creates a {@link NumericInputProbe} for each {@link JointBase}
    * {@code joint} coordinate specified by {@code prop} and fills it with the
    * angles in {@code coords}.
    * 
    * @param joint
    * member of {@code myJoints}
    * @param coords
    * {@link CoordinateData}
    * @param prop
    * joint coordinate
    * @param start
    * probe start time
    * @param stop
    * probe stop time
    */
   private void addCoordsInputProbe (
      JointBase joint, CoordinateData coords, String prop, double start,
      double stop) {
      NumericInputProbe angle =
         new NumericInputProbe (joint, prop, start, stop);
      angle.setModel (myMech);
      angle.setName (prop);
      for (int i = 0; i < coords.numFrames (); i++) {
         double time = coords.getFrameTime (i);
         double[] coord = new double[1];
         coord[0] = coords.getData (i, prop);
         angle.addData (time, coord);

      }
      angle.setActive (true);
      addInputProbe (angle);
   }

   /**
    * Adds the data stored in {@code myCoords} as numeric input probe per joint
    * if available.
    * 
    * @param coords
    * {@link CoordinateData}
    */
   private void addCoordsInputProbes (CoordinateData coords) {
      if (coords != null) {
         double start = coords.getFrameTime (0);
         double stop = coords.getFrameTime (coords.numFrames () - 1);
         myJoints.forEach (jt -> {
            for (int j = 0; j < jt.numCoordinates (); j++) {
               addCoordsInputProbe (
                  jt, coords, jt.getCoordinateName (j), start, stop);
            }
         });
      }
   }

   /**
    * Adds all model muscles and three frame exciters each for pelvis and both
    * calcanei to the controller.
    * 
    * @param controller
    * {@link TrackingController}
    */
   private void addExcitersToController (
      TrackingController controller, ForceData forces) {
      // Muscles
      myMuscles.forEach (msc -> {
         controller.addExciter (msc);
      });
      if (forces == null)
         return;
      // Frame exciters
      RigidBody calcnR = myBodies.get ("calcn_r");
      double maxForce = forces.getMaxForce ("Right GRF");
      double maxMoment = forces.getMaxMoment ("Right GRM");
      createAndAddFrameExciters (
         controller, myMech, calcnR, maxForce, maxMoment);
      RigidBody calcnL = myBodies.get ("calcn_l");
      maxForce = forces.getMaxForce ("Left GRF");
      maxMoment = forces.getMaxMoment ("Left GRM");
      createAndAddFrameExciters (
         controller, myMech, calcnL, maxForce, maxMoment);
      RigidBody pelvis = myBodies.get ("pelvis");
      maxForce = 1000;
      maxMoment = 20;
      createAndAddFrameExciters (
         controller, myMech, pelvis, maxForce, maxMoment);
   }

   /**
    * Creates a {@link NumericControlProbe} for the specified body {@code frame}
    * and fills it with the force data in {@code forces}, based on the
    * identifier {@code side} (i.e. left or right).
    * 
    * @param forces
    * {@link ForceTarget}
    * @param frame
    * Rigid body to apply the input probe to
    * @param side
    * left or right
    * 
    */
   private void addForceInputProbe (
      ForceData forces, Frame frame, String side) {
      NumericControlProbe grf = new NumericControlProbe ();
      grf.setModel (myMech);
      grf.setName (frame.getName () + " ground reaction forces");
      double duration =
         forces.getFrameTime (forces.numFrames () - 1)
         - forces.getFrameTime (0);
      grf.setStartStopTimes (0.0, duration);
      grf.setInterpolationOrder (Interpolation.Order.Cubic);
      grf.setDataFunction (new MomentArmFunction (frame, side));
      grf.setVsize (6);

      for (int i = 0; i < forces.numFrames (); i++) {
         VectorNd force = new VectorNd (6);
         double time = forces.getFrameTime (i);
         force.add (0, forces.getData (i, side + " GRF").x);
         force.add (1, forces.getData (i, side + " GRF").y);
         force.add (2, forces.getData (i, side + " GRF").z);
         force.add (3, forces.getData (i, side + " GRM").x);
         force.add (4, forces.getData (i, side + " GRM").y);
         force.add (5, forces.getData (i, side + " GRM").z);
         grf.addData (time, force);
      }
      grf.setActive (true);
      addInputProbe (grf);
   }

   /**
    * Adds the data stored in {@code myForces} as individual input probes if
    * available.
    * 
    * @param forces
    * {@link ForceData}
    */
   private void addForceInputProbes (ForceData forces) {
      if (forces != null) {
         RigidBody calcnR = myBodies.get ("calcn_r");
         addForceInputProbe (forces, calcnR, "Right");
         RigidBody calcnL = myBodies.get ("calcn_l");
         addForceInputProbe (forces, calcnL, "Left");
      }
   }

   /**
    * Adds the data stored in {@code myMotion} and frame orientations/positions
    * to the input probes of the TrackingController, if available.
    * 
    * @param controller
    * TrackingController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param motion
    * Experimental marker trajectories
    */
   private void addMotionInputProbes (
      MarkerMotionData motion, MarkerMapping map,
      TrackingController controller) {
      if (motion == null) {
         return;
      }
      NumericInputProbe targetProbe =
         (NumericInputProbe)getInputProbes ().get ("target positions");
      if (targetProbe == null) {
         return;
      }
      // Collect FrameTarget input data
      ArrayList<VectorNd[]> orientations = new ArrayList<VectorNd[]> ();
      ArrayList<VectorNd[]> positions = new ArrayList<VectorNd[]> ();
      myBodies.forEach (b -> {
         try {
            orientations.add (collectInputProbeData (b, motion, "orientation"));
            positions.add (collectInputProbeData (b, motion, "position"));
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      });
      // Generate inline probe data for all sources
      ArrayList<MotionTargetComponent> sources = controller.getMotionSources ();
      for (int i = 0; i < motion.numFrames (); i++) {
         VectorNd data = new VectorNd ();
         int numUsedFrameTargets = 0;
         int numUsedMotionTargets = 0;
         for (int j = 0; j < sources.size (); j++) {
            if (sources.get (j) instanceof RigidBody) {
               // Collect all FrameTarget input data
               data.append (positions.get (numUsedFrameTargets)[i]);
               data.append (orientations.get (numUsedFrameTargets)[i]);
               numUsedFrameTargets++;
            }
            else {
               // Collect all MotionTarget input data
               String name = sources.get (numUsedMotionTargets).getName ();
               String label = map.getExpLabelFromModel (name);
               data.append (motion.getMarkerPosition (i, label).x);
               data.append (motion.getMarkerPosition (i, label).y);
               data.append (motion.getMarkerPosition (i, label).z);
               numUsedMotionTargets++;
            }
         }
         double time = motion.getFrameTime (i);
         if (data.size () == targetProbe.getVsize ()) {
            targetProbe.addData (time, data);
         }
      }
      targetProbe.setActive (true);
   }

   /**
    * Defines the Targets of the {@link MotionTargetTerm} of the
    * {@link TrackingController}.
    * 
    * @param controller
    * TrackingController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    */
   private void addMotionTargetsToController (
      TrackingController controller, MarkerMapping map) {
      myBodies.forEach (b -> {
         controller.addMotionTarget (b);
      });
      myMarkers.forEach (mkr -> {
         if (map.getExpLabelFromModel (mkr.getName ()) != null) {
            Double weight = map.getMarkerWeight (mkr.getName ());
            controller.addMotionTarget (mkr, weight);
         }
      });
   }

   /**
    * Creates output probes for and fills a control panel if specified
    * 
    * @param motion
    * {@link MarkerMotionData}
    */
   private void addNumOutputProbesAndPanel (
      MarkerMotionData motion, TrackingController controller) {
      // Joint angle output probes and panel
      ControlPanel jointPanel = new ControlPanel ("Joint Coordinates");
      double start = motion.getFrameTime (0);
      double stop = motion.getFrameTime (motion.numFrames () - 1);
      double step = getMaxStepSize ();
      myJoints.forEach (jt -> {
         for (int i = 0; i < jt.numCoordinates (); i++) {
            createProbeAndPanel (
               jt, jointPanel, jt.getCoordinateName (i), start, stop, step);
         }
      });
      addControlPanel (jointPanel);
      // Muscle excitations output probes and panel
      ControlPanel musclePanel = new ControlPanel ("Muscle Excitations");
      if (controller != null) {
         controller.getExciters ().forEach (e -> {
            if (e instanceof FrameExciter) {
               createProbeAndPanel (e, null, "excitation", start, stop, step);
            }
            else {
               musclePanel.addWidget (e.getName (), e, "excitation");
            }
         });
         addControlPanel (musclePanel);
      }
      // Body pose output probes
      //myBodies.forEach (b -> {
      //   createProbeAndPanel (b, null, "position", start, stop, step);
      //   createProbeAndPanel (b, null, "orientation", start, stop, step);
      //});
   }

   /**
    * Adjusts the export paths for the default {@link NumericOutputProbe}s of
    * the {@link MotionTargetController} "tracked positions", "source positions"
    * and "computed excitations" to the current working directory.
    */
   private void adjustDefaultProbePaths () {
      NumericOutputProbe track =
         (NumericOutputProbe)getOutputProbes ().get ("tracked positions");
      String path =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/tracked positions.txt");
      if (track != null) {
         track.setAttachedFileName (path);
      }
      NumericOutputProbe source =
         (NumericOutputProbe)getOutputProbes ().get ("source positions");
      path =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/source positions.txt");
      if (source != null) {
         source.setAttachedFileName (path);
      }
      NumericOutputProbe excite =
         (NumericOutputProbe)getOutputProbes ().get ("computed excitations");
      path =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/computed excitations.txt");
      if (excite != null) {
         excite.setAttachedFileName (path);
      }
   }
   
   /**
    * Generates a dummy input probe to read in the specified data in the
    * associated file.
    * 
    * @param frame
    * component related to the input probe
    * @param motion
    * experimental motion data containing the time series
    * @param prop
    * property name
    * @return VectorNd array containing the probe data
    * @throws IOException if attached file doesn't exist
    */
   private VectorNd[] collectInputProbeData (
      Frame frame, MarkerMotionData motion, String prop) throws IOException {
      String file =
         ArtisynthPath
            .getSrcRelativePath (
               this,
               myName + "/Input/" + frame.getName () + " " + prop + ".txt");
      NumericInputProbe probe = new NumericInputProbe (frame, prop, file);
      VectorNd[] data = new VectorNd[motion.numFrames ()];
      for (int i = 0; i < data.length; i++) {
         double time = motion.getFrameTime (i);
         data[i] = probe.getData (time);
      }
      return data;
   }

   /**
    * Creates a complete set of FrameExciters for a given frame and adds them to
    * a MechModel and a tracking controller.
    *
    * @param ctrl
    * tracking controller to add the exciters to
    * @param mech
    * MechModel to add the exciters to
    * @param frame
    * frame for which the exciters should be created
    * @param maxForce
    * maximum translational force along any axis
    * @param maxMoment
    * maximum moment about any axis
    * 
    * @author John Lloyd
    */
   private FrameExciter[] createAndAddFrameExciters (
      TrackingController ctrl, MechModel mech, Frame frame, double maxForce,
      double maxMoment) {
      FrameExciter[] exs = new FrameExciter[6];
      exs[0] = new FrameExciter (null, frame, WrenchComponent.FX, maxForce);
      exs[1] = new FrameExciter (null, frame, WrenchComponent.FY, maxForce);
      exs[2] = new FrameExciter (null, frame, WrenchComponent.FZ, maxForce);
      exs[3] = new FrameExciter (null, frame, WrenchComponent.MX, maxMoment);
      exs[4] = new FrameExciter (null, frame, WrenchComponent.MY, maxMoment);
      exs[5] = new FrameExciter (null, frame, WrenchComponent.MZ, maxMoment);
      // if the frame has a name, use this to create names for the exciters
      if (frame.getName () != null) {
         WrenchComponent[] wcs = WrenchComponent.values ();
         for (int i = 0; i < 6; i++) {
            exs[i]
               .setName (
                  frame.getName () + "_" + wcs[i].toString ().toLowerCase ());
         }
      }
      Double[] weights = new Double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
      for (int i = 0; i < 6; i++) {
         mech.addForceEffector (exs[i]);
         ctrl.addExciter (weights[i], exs[i]);
      }
      return exs;
   }

   /**
    * Adds a collision response {@code collResp} and behavior {@code collBehav}
    * object for the compliant contact between {@code bodyA} and {@code bodyB},
    * based on the given parameters {@code comp} and {@code damp}.
    * 
    * @param bodyA
    * @param bodyB
    * @param comp
    * Compliance coefficient
    * @param damp
    * Damping coefficient
    */
   private void createCollision (
      RigidBody bodyA, RigidBody bodyB, double comp, double damp) {
      CollisionBehavior behavior;
      behavior = myMech.setCollisionBehavior (bodyA, bodyB, true);
      behavior.setCompliance (comp);
      behavior.setDamping (damp);
      myMech.setCollisionResponse (bodyA, bodyB);
   }

   /**
    * Creates a {@link NumericOutputProbe} for the given property {@code prop}
    * of the model component {@code comp} with the parameters {@code start},
    * {@code stop} and {@code step}. Adds a widget to a {@link ControlPanel} for
    * each {@code prop}, if {@code panel} is not null.
    * 
    * @param comp
    * Model component
    * @param panel
    * Contro panel
    * @param prop
    * Name of the property
    * @param start
    * Start time of the probe
    * @param stop
    * Stop time of the probe
    * @param step
    * Output interval of the probe
    */
   private void createProbeAndPanel (
      ModelComponent comp, ControlPanel panel, String prop, double start,
      double stop, double step) {
      String filepath =
         PathFinder
            .getSourceRelativePath (
               this, "/" + myName + "/Output/" + comp.getName () + " " + prop
               + ".txt");
      NumericOutputProbe probe =
         new NumericOutputProbe (comp, prop, filepath, step);

      if (panel != null) {
         panel.addWidget (comp, prop);
         probe.setName (prop);
      }
      else {
         probe.setName (comp.getName () + " " + prop);
      }
      probe.setStartStopTimes (start, stop);
      addOutputProbe (probe);
   }

   /**
    * Defines a tracking controller that calculates muscle activations based on
    * trajectories and sets its properties.
    * 
    * @param motion
    * {@link MarkerMotionData}
    * @param map
    * {@link MarkerMapping}
    * @param name
    * Name specifier of the current working directory
    */
   private TrackingController defineControllerAndProps (
      MarkerMotionData motion, MarkerMapping map, String name) {
      TrackingController motcon =
         new TrackingController (myMech, "Motion controller");
      // MotionTargetController motcon =
      // new MotionTargetController (myMech, "Motion controller", name);
      // motcon.addMotionData (motion, map);
      motcon.addL2RegularizationTerm ();
      // Calculate the duration in seconds from the number of frames
      double duration =
         motion.getFrameTime (motion.numFrames () - 1)
         - motion.getFrameTime (0);
      motcon.setProbeDuration (duration);
      motcon.getMotionTargetTerm ().setUsePDControl (true);
      motcon.getMotionTargetTerm ().setKd (0.5);
      motcon.getMotionTargetTerm ().setKp (1.0);
      return motcon;
   }

   /**
    * Defines all in- and outgoing probes for the model. Ingoing probes can be
    * experimental marker trajectories, forces or generalized coordinates,
    * outgoing probes can be all joint angles (if not already specified as
    * generalized coordinates), muscle and frame exciter excitations.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @throws IOException
    * if specified files or file paths are invalid
    */
   private void defineIOProbes (String name, int scale) throws IOException {
      // Read all input data
      myMotion = readTRCFile (name, scale);
      myForces = readForceFile (name);
      //myCoords = readCoordsFile (name);
      myMap = readMarkerFile (name);
      // Create controller and periphery
      TrackingController controller =
         defineControllerAndProps (myMotion, myMap, name);
      addExcitersToController (controller, myForces);
      addMotionTargetsToController (controller, myMap);
      setMotionTargetsPositions (controller);
      controller.createProbesAndPanel (this);
      adjustDefaultProbePaths ();
      addController (controller);
      // Add any input probe after controller.createProbesAndPanel
      addForceInputProbes (myForces);
      addCoordsInputProbes (myCoords);
      addMotionInputProbes (myMotion, myMap, controller);
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
      // Add output probes
      addNumOutputProbesAndPanel (myMotion, controller);
      addBreakPoint (myMotion.getFrameTime (myMotion.numFrames () - 1));
   }

   /**
    * Queries all {@link RigidBody} objects from the current {@link MechModel}
    * and stores them in a global variable.
    * 
    * @return list of rigid bodies
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<RigidBody> getBodiesFromOsim () {
      myBodies = (RenderableComponentList<RigidBody>)myMech.get ("bodyset");
      // Ensure COM and CS are coinciding
      //myBodies.forEach (b -> {
      //   b.centerPoseOnCenterOfMass ();
      //});
      // Check for unclosed meshes
      //myBodies.forEach (b -> {
      //   PolygonalMesh mesh = b.getCollisionMesh ();
      //   System.out.print (b.getName () + ": read ");
      //   System.out.println (mesh.getFaces ().size () + " faces.");
      //   if (!mesh.isClosed ()) {
      //      System.err.println ("Warning: Mesh not closed!");
      //   }
      //});
      return myBodies;
   }

   /**
    * Defines joint constraints based on jointsets or by defining the joints
    * based on the rigid bodies, that are part of the model.
    * 
    * @param myBodies
    * list of rigid bodies
    * @return list of {@link OpenSimCustomJoint}
    */
   private RenderableComponentList<JointBase> getJointsFromOsim (
      RenderableComponentList<RigidBody> myBodies) {
      double comp = 1e-5;
      if (myMech.contains (myMech.get ("jointset"))) {
         System.out.println ("Generated joint constraints from jointset.");
         return getJointsFromJointset (comp);

      }
      else {
         System.out
            .println (
               "Generated joint constraints from ridig body connectors.");
         return getJointsFromBodyset (comp);
      }
   }

   /**
    * Defines joints by accessing the connectors of every body and writing them
    * to a separate list. Connectors are defined per rigid body, a body that is
    * connected to 3 bodies, has therefore three different connectors. The
    * connectors in each rigid body are ordered in such a way, that each joint
    * is exactly addressed once, if the first index of the connector list
    * (getConnectors.get(0)) in every rigid body is accessed.
    * 
    * @param compMagnitude
    * compliance magnitude
    * @return list of {@link OpenSimCustomJoint}
    */
   private RenderableComponentList<JointBase> getJointsFromBodyset (
      double compMagnitude) {
      // Ground shares no joint with any rigid body else than the hip
      // so skip that, since hip is going to be addressed either way.
      myBodies.forEach (rb -> {
         if (rb.getName ().equals ("ground")) {
            return;
         }
         else {
            // Write all joints to a joint list
            myJoints.add ((JointBase)rb.getConnectors ().get (0));
            int end = myJoints.size ();
            setJointCompliance (myJoints.get (end), compMagnitude);
            DoubleInterval range = new DoubleInterval ();
            switch (rb.getName ()) {
               // specify the joint constraints for each joint individually
               // by addressing the respective dof (int idx) and its range.
               case "pelvis":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  range.set (-5, 5);
                  myJoints.get (end - 1).setCoordinateRange (3, range);
                  range.set (-1, 2);
                  myJoints.get (end - 1).setCoordinateRange (4, range);
                  range.set (-3, 3);
                  myJoints.get (end - 1).setCoordinateRange (5, range);
                  break;
               case "femur_r":
                  range.set (-120.0, 120.0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "tibia_r":
                  range.set (-120.0, 10);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "talus_r":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "calcn_r":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "toes_r":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "femur_l":
                  range.set (-120.0, 120.0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "tibia_l":
                  range.set (-120.0, 10);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "talus_l":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "calcn_l":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "toes_l":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "torso":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
            }
         }
      });
      return myJoints;
   }

   /**
    * Defines joints by accessing the jointsets predefined by the .osim file.
    * 
    * @param compMagnitude
    * compliance magnitude
    * @return list of {@link OpenSimCustomJoint}
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<JointBase> getJointsFromJointset (
      double compMagnitude) {
      myJoints = (RenderableComponentList<JointBase>)myMech.get ("jointset");
      DoubleInterval range = new DoubleInterval ();
      myJoints.forEach (jt -> {
         setJointCompliance (jt, compMagnitude);
         // Define joint limits for each joint constraint
         switch (jt.getName ()) {
            case "ground_pelvis":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               range.set (-5, 5);
               jt.setCoordinateRange (3, range);
               range.set (-1, 2);
               jt.setCoordinateRange (4, range);
               range.set (-3, 3);
               jt.setCoordinateRange (5, range);
               break;
            case "hip_r":
               range.set (-120.0, 120.0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "knee_r":
               range.set (-120.0, 10);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "ankle_r":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "subtalar_r":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "mtp_r":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "hip_l":
               range.set (-120.0, 120.0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "knee_l":
               range.set (-120.0, 10);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "ankle_l":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "subtalar_l":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "mtp_l":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "back":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               jt.setCoordinateRangeDeg (2, range);
               break;
         }
      });
      return myJoints;
   }

   /**
    * Queries all {@link FrameMarker} objects from the current {@link MechModel}
    * and stores them in a global variable.
    * 
    * @return list of frame markers
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<FrameMarker> getMarkerFromOsim () {
      myMarkers =
         (RenderableComponentList<FrameMarker>)myMech.get ("markerset");
      // Overwrite attachments for toe markers, since attached to calcanei
      myMarkers.forEach (m -> {
         RigidBody newFrame = null;
         Vector3d newRef;
         Vector3d pos;
         if (m.getName ().contains ("R_Toe")) {
            newFrame = myBodies.get ("toes_r");
         }
         else if (m.getName ().contains ("L_Toe")) {
            newFrame = myBodies.get ("toes_l");
         }
         else {
            return;
         }
         Point3d newLoc = new Point3d (0, 0, 0);
         newRef = (Vector3d)newFrame.getPosition ();
         pos = (Vector3d)m.getPosition ();
         newLoc.sub (pos, newRef);
         // Set new attachment
         m.setFrame (newFrame);
         // Overwrite refpos vector after setting attachment
         m.setRefPos ((Point3d)newRef);
         // Overwrite location vector after setting attachment
         m.setLocation (newLoc);
         System.out
            .println (
               "Warning: Attachment adjusted for: " + m.getName () + " to: "
               + newFrame.getName ());
      });
      System.out.println ("Model markers: " + myMarkers.size ());
      return myMarkers;
   }

   /**
    * Queries all {@link MultiPointMuscle} objects from the current
    * {@link MechModel} and stores them in a global variable.
    * 
    * @return list of multi point muscles
    */
   @SuppressWarnings("unchecked")
   private List<MultiPointMuscle> getMusclesFromOsim () {
      RenderableComponentList<ModelComponent> forces =
         (RenderableComponentList<ModelComponent>)myMech.get ("forceset");
      forces.forEach (frc -> {
         frc.getChildren ().forEachRemaining (obj -> {
            if (obj instanceof PointList) {
               return;
            }
            if (obj instanceof MultiPointMuscle) {
               myMuscles.add ((MultiPointMuscle)obj);
            }
         });
      });
      return myMuscles;
   }

   /**
    * Returns the name of the current working directory folder.
    * 
    * @param fc
    * @return
    */
   private String getNameFromFileDiaglog (JFileChooser fc) {
      fc.setCurrentDirectory (ArtisynthPath.getSrcRelativeFile (this, myName));
      fc.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      fc.setDialogTitle ("Please select a working directory.");
      fc.showOpenDialog (null);
      return fc.getSelectedFile ().getName ();
   }

   /**
    * Initializes the FEM Meshes
    * 
    * @throws IOException
    */
   private void initializeFEM () throws IOException {
      // Femur
      File femurInput =
         ArtisynthPath.getSrcRelativeFile (this, "Meshes/C01L_Femur.obj");
      PolygonalMesh femurMesh = new PolygonalMesh ();
      FemModel3d femur = new FemModel3d ("femur");
      femurMesh.read (femurInput);
      FemFactory.createFromMesh (femur, femurMesh, 1.2);
      femur.setDensity (2E-6);
      femur.setMaterial (new LinearMaterial (5000, 0.35));
      femur.setParticleDamping (0.1);
      femur.setStiffnessDamping (0.1);
      myMech.addModel (femur);

      // Tibia and Fibula
      File tibfibInput =
         ArtisynthPath.getSrcRelativeFile (this, "Meshes/C01L_TibiaFibula.obj");
      PolygonalMesh tibfibMesh = new PolygonalMesh ();
      FemModel3d tibfib = new FemModel3d ("tibfib");
      tibfibMesh.read (tibfibInput);
      FemFactory.createFromMesh (tibfib, tibfibMesh, 1.2);
      tibfib.setDensity (2E-6);
      tibfib.setMaterial (new LinearMaterial (5000, 0.35));
      tibfib.setParticleDamping (0.1);
      tibfib.setStiffnessDamping (0.1);
      myMech.addModel (tibfib);

      // not used as long as the Ansys reader is not called
      // public String inputNodes = PathFinder.
      // getSourceRelativePath (this,"Input Files/testNodes.node" );
      // public String inputElems =PathFinder.
      // getSourceRelativePath(this,"Input Files/testElems.elem" );
      // Generate all FEM related geometries.
      // AnsysReader.read (femur,inputNodes,inputElems,2E-6,null,0);
   }

   /**
    * Imports an OpenSim model from the current working directory, defines joint
    * constraints and stores important model components in variables.
    * 
    * @param myName
    * Name specifier of the current working directory
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    */
   private void initializeOsim (String myName, int scale) {
      readOsimFile (myName, scale);    
      myBodies = getBodiesFromOsim ();
      myJoints = getJointsFromOsim (myBodies);
      myMuscles = getMusclesFromOsim ();
      myMarkers = getMarkerFromOsim ();
      myMech.scaleDistance (scale);
      myMech.scaleMass (scale); 
      setInitialPose ();
   }

   /**
    * Returns joint angles as generalized coordinates from prior IK calculations
    * or MoCap systems ({@code name_angles.mot}) in the input folder of the
    * current working directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return {@link CoordinateData}
    * @throws IOException
    * if the file or file path are invalid
    */
   private CoordinateData readCoordsFile (String name) throws IOException {
      String motName = name + "/Input/" + name + "_angles.mot";
      File motFile = ArtisynthPath.getSrcRelativeFile (this, motName);
      if (!motFile.exists () || motFile.isDirectory ()) {
         return null;
      }
      MOTReader motReader = new MOTReader (motFile);
      motReader.readData ();
      // Print reading details to the console
      System.out
         .println (
            "Calculated generalized coordinates: "
            + motReader.getNumCoordLabels ());
      System.out
         .println (
            "MOT file: read " + motReader.getNumCoordFrames () + " frames");
      return motReader.getCoordinateData ();
   }

   /**
    * Returns ground reaction forces from the available experimental force data
    * ({@code name_forces.mot}) in the input folder of the current working
    * directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return {@link ForceData}
    * @throws IOException
    * if the file or file path are invalid
    */
   private ForceData readForceFile (String name) throws IOException {
      String motName = name + "/Input/" + name + "_forces.mot";
      File motFile = ArtisynthPath.getSrcRelativeFile (this, motName);
      if (!motFile.exists () || motFile.isDirectory ()) {
         return null;
      }
      MOTReader motReader = new MOTReader (motFile);
      motReader.readData ();
      // Print reading details to the console
      System.out
         .println (
            "Experimental force data: " + motReader.getNumForceLabels ());
      System.out
         .println (
            "MOT file: read " + motReader.getNumForceFrames () + " frames");
      return motReader.getForceData ();
   }

   /**
    * Returns a map of marker names based on an input file. It accepts files in
    * a specific format: No head line, all lines divided by a tab. 1st column:
    * Name of the model marker 2nd column: Name of the experimental marker 3rd
    * Column: Marker weighting for the inverse solver If no experimental marker
    * exists for a given model marker, leave out the 2nd and 3rd column and
    * continue with next line.
    * 
    * @param name
    * Name specifier of the model working directory
    * @return {@link MarkerMapping}
    * @throws IOException
    * if the file or file path are invalid
    */
   private MarkerMapping readMarkerFile (String name) throws IOException {
      String mapName = name + "/Input/" + name + "_markers.txt";
      File mapFile = ArtisynthPath.getSrcRelativeFile (this, mapName);
      if (!mapFile.exists () || mapFile.isDirectory ()) {
         return null;
      }
      BufferedReader reader = new BufferedReader (new FileReader (mapFile));
      ArrayList<String> modelLabels = new ArrayList<String> ();
      ArrayList<String> expLabels = new ArrayList<String> ();
      ArrayList<Double> weights = new ArrayList<Double> ();
      String line;
      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         assert line.length () != 0;
         String[] markers = line.split ("\t");
         if (markers.length == 1) {
            System.out
               .println (
                  "Warning: Marker " + markers[0]
                  + ": No corresponding experimental marker detected.");
            modelLabels.add (markers[0]);
            expLabels.add (null);
            weights.add (null);
         }
         else {
            modelLabels.add (markers[0]);
            expLabels.add (markers[1]);
            weights.add (Double.parseDouble (markers[2]));
         }
      }
      reader.close ();
      return new MarkerMapping (modelLabels, expLabels, weights);
   }

   /**
    * Reads in the .osim file in the specified working folder and creates a
    * corresponding model the current {@link MechModel}.
    * 
    * @param name
    * Name specifier for the current working directory
    * @param scale
    */
   private void readOsimFile (String name, int scale) {
      String modelPath = myName + "/gait2392_simbody_scaled.osim";
      File osimFile = ArtisynthPath.getSrcRelativeFile (this, modelPath);
      String geometryPath = myName + "/Geometry/";
      File geometryFile = ArtisynthPath.getSrcRelativeFile (this, geometryPath);
      if (osimFile.exists () && geometryFile.exists ()) {
         OpenSimParser parser = new OpenSimParser (osimFile);
         parser.setGeometryPath (geometryFile);
         parser.createModel (myMech);
      }
   }

   /**
    * Returns marker trajectories from the available experimental motion data
    * ({@code name_positions.trc}) in the input folder of the current working
    * directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @return {@link MarkerMotionData}
    * @throws IOException
    * if the file or file path are invalid
    */
   private MarkerMotionData readTRCFile (String name, int scale)
      throws IOException {
      String trcName = name + "/Input/" + name + "_positions.trc";
      File trcFile = ArtisynthPath.getSrcRelativeFile (this, trcName);
      if (!trcFile.exists () || trcFile.isDirectory ()) {
         return null;
      }
      TRCReader trcReader = new TRCReader (trcFile);
      trcReader.readData ();
      System.out
         .println (
            "Experimental markers: " + trcReader.getMarkerLabels ().size ());
      System.out
         .println ("TRC file: read " + trcReader.getNumFrames () + " frames");
      MarkerMotionData motion = trcReader.getMotionData ();
      // Scale the marker trajectories individually, since there is no
      // general .scale () method for marker positions
      for (int i = 0; i <= motion.numFrames () - 1; i++) {
         motion.getMarkerPositions (i).forEach (p -> {
            p.x = p.x * scale / 1000;
            p.y = p.y * scale / 1000;
            p.z = p.z * scale / 1000;
         });
      }
      return motion;
   }

   /**
    * Defines global and individual contact properties of the entire model.
    * Individual contact is enforced by defining a collision behavior object for
    * each interface (joints, ground contact, etc.), that contains compliant
    * contact properties. To monitor contact events, a collision response object
    * is created for each interface vice versa. The list of collision responses
    * is handed to the contact monitor class object.
    * 
    * @param coll
    * {@link CollisionManager}
    * @throws IOException
    */
   private void setContactProps (CollisionManager coll) throws IOException {
      coll.setName ("Collision manager");
      // Handle overconstrained contact
      coll.setReduceConstraints (true);
      coll.setBilateralVertexContact (false);
      myJoints.forEach (jt -> {
         if (jt.getName ().contains ("pelvis")) {
            return;
         }
         RigidBody bodyA = (RigidBody)jt.getBodyA ();
         RigidBody bodyB = (RigidBody)jt.getBodyB ();
         // Calculate compliant contact properties
         double comp = 0.1;
         double mass = bodyA.getMass () + bodyB.getMass ();
         double damp = 2 * 1 * Math.sqrt (1 / comp * mass);
         createCollision (bodyA, bodyB, comp, damp);
      });
      // Initialize the contact monitor to handle all individual collision
      // responses
      ContactMonitor contMonitor =
         new ContactMonitor (coll.responses (), myName);
      contMonitor.setName ("Contact monitor");
      contMonitor.setUseFullReport (true);
      addMonitor (contMonitor);
   }

   /**
    * Sets the initial pose of the OpenSim model.
    */
   private void setInitialPose () {
      // pelvis
      myJoints.get ("ground_pelvis").setCoordinateDeg (0, 5.31);
      myJoints.get ("ground_pelvis").setCoordinateDeg (1, -5.38);
      myJoints.get ("ground_pelvis").setCoordinateDeg (2, -4.59);
      myJoints.get ("ground_pelvis").setCoordinate (3, 0.61);
      myJoints.get ("ground_pelvis").setCoordinate (4, 1.01);
      myJoints.get ("ground_pelvis").setCoordinate (5, 0.041);
      // right hip
      myJoints.get ("hip_r").setCoordinateDeg (0, -21.24);
      myJoints.get ("hip_r").setCoordinateDeg (1, 5.09);
      myJoints.get ("hip_r").setCoordinateDeg (2, 7.22);
      // right knee
      myJoints.get ("knee_r").setCoordinateDeg (0, -9.83);
      // right ankle
      myJoints.get ("ankle_r").setCoordinateDeg (0, 13.91);
      //left hip
      myJoints.get ("hip_l").setCoordinateDeg (0, 18.42);
      myJoints.get ("hip_l").setCoordinateDeg (1, -7.26);
      myJoints.get ("hip_l").setCoordinateDeg (2, -5.024);
      // left knee
      myJoints.get ("knee_l").setCoordinateDeg (0, -4.70);
      // left ankle
      myJoints.get ("ankle_l").setCoordinateDeg (0, -0.812);
      // back
      myJoints.get ("back").setCoordinateDeg (0, -17.85);
      myJoints.get ("back").setCoordinateDeg (1, 2.98);
      myJoints.get ("back").setCoordinateDeg (2, 3.93);
   }
   
   /**
    * Adjusts the initial positions of all target markers. TODO: to be removed,
    * as soon as MotionTargetController is used again.
    * 
    * @param controller
    */
   private void setMotionTargetsPositions (TrackingController controller) {
      if (myMap == null)
         return;
      PointList<TargetPoint> myTargets = controller.getTargetPoints ();
      if (myTargets == null)
         return;
      ArrayList<MotionTargetComponent> mySources =
         controller.getMotionSources ();
      for (int i = 0; i < mySources.size(); i++) {
         String label =
            myMap.getExpLabelFromModel (mySources.get (i).getName ());
         if (label != null) {
            Point3d position;
            position = (Point3d)myMotion.getMarkerPosition (0, label);
            myTargets.get (i).setPosition (position);
         }
      }
   }

   /**
    * Define compliance for the provided joint to prevent overconstraints.
    * 
    * @param jt
    * {@link JointBase} object
    * @param compMagnitude
    * compliance magnitude
    */
   private void setJointCompliance (JointBase jt, double compMagnitude) {
      VectorNd comp = new VectorNd (jt.numConstraints ());
      VectorNd damp = new VectorNd (jt.numConstraints ());
      for (int i = 0; i < jt.numConstraints (); i++) {
         comp.set (i, compMagnitude);
         Frame bodyA = (Frame)jt.getBodyA ();
         Frame bodyB = (Frame)jt.getBodyB ();
         double mass = bodyA.getEffectiveMass () + bodyB.getEffectiveMass ();
         damp.set (i, 2 * 1 * Math.sqrt (mass / comp.get (i)));
      }
      jt.setCompliance (comp);
      jt.setDamping (damp);
   }

   /**
    * Sets the following simulation properties: 1. Integrator, 2. Stabilization
    * method, 3. Adaptive time stepping, 4. maximum step size, 5. unit scale of
    * the model 6. gravity
    */
   private void setSimulationProperties () {
      // Solver properties
      MechSystemSolver solver = myMech.getSolver ();
      solver.setIntegrator (Integrator.ConstrainedBackwardEuler);
      solver.setStabilization (PosStabilization.GlobalStiffness);
      solver.setMaxIterations (100);
      solver.setTolerance (1e-5);
      setAdaptiveStepping (true);
      setMaxStepSize (0.017); 
      // Damping properties
      myMech.setFrameDamping (0.07);
      myMech.setRotaryDamping (5.0);
      myMech.setInertialDamping (2.8);
      myMech.setPointDamping (10.0);
      // Define scale (mm = 1000, or m = 1)
      myScale = 1;
      myMech.setGravity (new Vector3d (0, -9.81, 0));
      if (myMech.getGravity ().equals (new Vector3d (0, 0, 0))) {
         JFrame frame = new JFrame ("Warning");
         frame.add (new JLabel ("Warning: Zero gravitation!", JLabel.CENTER));
         frame.setSize (300, 100);
         frame.setLocationRelativeTo (getMainFrame ());
         // Visibility always at last
         frame.setVisible (true);
      }
   }

   /**
    * Adds contact information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param coll
    * {@link CollisionManager} of the current {@link MechModel}
    * @param behav
    * list of {@link CollisionBehaviorList} objects
    */
   private void writeContactInfo (
      StringBuilder output, CollisionManager coll,
      CollisionBehaviorList behav) {
      output
         .append ("\n%%CONTACT%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nGLOBAL\n").append ("Contact Method: ")
         .append (coll.getMethod ().toString ()).append ("\n")
         .append ("Contact Region Detection Algorithm (Collision Type): ")
         .append (coll.getColliderType ().toString ()).append ("\n")
         .append ("Reduce overconstrained contact: ")
         .append (coll.getReduceConstraints ()).append ("\n")
         .append ("Number of contact interfaces: ")
         .append (coll.behaviors ().size ()).append ("\n")
         .append ("\nINDIVIDUAL\n");
      // Access each individual contact behavior for each model component
      behav.forEach (cb -> {
         output
            .append ("Contact Interface: ").append (cb.getName ()).append ("\n")
            .append ("Bilateral vertex contact: ")
            .append (cb.getBilateralVertexContact ()).append ("\n")
            .append ("Body a: ")
            .append (cb.getCollidablePair ().get (0).getName ()).append ("\t")
            .append ("Body or Group B: ")
            .append (cb.getCollidablePair ().get (1).getName ()).append ("\n")
            .append ("Penetration Tolerance: ")
            .append (String.format ("%.3f", cb.getPenetrationTol ()))
            .append ("\n").append ("Contact compliance: ")
            .append (String.format ("%.3f", cb.getCompliance ())).append ("\n")
            .append ("Contact damping: ")
            .append (String.format ("%.3f", cb.getDamping ())).append ("\n\n");
      });
   }

   /**
    * Adds FEM information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param meshes
    * list of {@link FemModel3d} objects
    */
   private void writeFEMInfo (
      StringBuilder output, RenderableComponentList<FemModel3d> meshes) {
      output
         .append ("%%FINITE ELEMENTS%%\n").append (
            "%%------------------------------------------------------------%%\n");
      // .append ("Number of meshes: ").append (meshes.size ()).append ("\n");
   }

   /**
    * Adds joint information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param joints
    * list of {@link JointBase} objects
    */
   private void writeJointInfo (
      StringBuilder output, RenderableComponentList<JointBase> joints) {
      output
         .append ("JOINTS\n").append ("Number of joints: ")
         .append (joints.size ()).append ("\n");
      String format = "%d DOF %-20s%-14s\tType: %-6s%n";
      joints.forEach (jt -> {
         output
            .append ("Name: ").append (jt.getName ()).append ("\t")
            .append ("Body A: ").append (jt.getBodyA ().getName ())
            .append ("\t").append ("Body B: ")
            .append (jt.getBodyB ().getName ()).append ("\n")
            .append ("Compliance: ").append (jt.getCompliance ().toString ())
            .append ("\n").append ("Damping: ")
            .append (jt.getDamping ().toString ("%.3f")).append ("\n");
         for (int i = 0; i < jt.numCoordinates (); i++) {
            double min = jt.getCoordinateRangeDeg (i).getLowerBound ();
            double max = jt.getCoordinateRangeDeg (i).getUpperBound ();
            output
               .append (
                  String
                     .format (
                        format, i, jt.getCoordinateName (i),
                        String.format ("[%.1f, %.1f]", min, max),
                        jt.getCoordinateMotionType (i).toString ()));
         }
         output.append ("\n");
      });
   }

   /**
    * Adds rigid body information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param path
    * Path to the used .osim file
    * @param bodies
    * list of {@link RidigBody} objects
    */
   private void writeModelInfo (
      StringBuilder output, String path,
      RenderableComponentList<RigidBody> bodies) {
      output
         .append ("\n%%MODELBASE%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nBODIES\n").append ("Model: ").append (path).append ("\n")
         .append ("Number of bodies: ").append (bodies.size ()).append ("\n")
         .append ("Total model mass: ")
         .append (String.format ("%.1f", myMech.getActiveMass ()))
         .append (" kg\n\n");
      // Identify unclosed meshes before appending individual info about each
      // body
      bodies.forEach (rb -> {
         if (!rb.getCollisionMesh ().isClosed ()) {
            output
               .append ("Unclosed Mesh found: ").append (rb.getName ())
               .append ("\n");
         }
      });
      output.append ("\n");
      // append individual info for each body
      bodies.forEach (rb -> {
         if (rb.getName ().equals ("ground")) {
            return;
         }
         output
            .append (rb.getName () + "\t")
            .append (String.format ("%.3f", rb.getMass ())).append ("\tkg\n");
         ArrayList<Vertex3d> vertices = rb.getSurfaceMesh ().getVertices ();
         vertices.forEach (vt -> {
            output
               .append (vt.getIndex ()).append ("\t").append (vt.pnt)
               .append ("\n");
         });
         ArrayList<Face> faces = rb.getSurfaceMesh ().getFaces ();
         faces.forEach (f -> {
            output
               .append (f.idx).append ("\t")
               .append (f.getVertex (0).getIndex ()).append ("\t")
               .append (f.getVertex (1).getIndex ()).append ("\t")
               .append (f.getVertex (2).getIndex ()).append ("\t")
               .append ("\n");
         });
         output.append ("\n");
      });
      // In case it is desired to export the Mesh to a separate file.
      // String inRBMeshString = ArtisynthPath.getSrcRelativePath
      // (this,"/Input Files/" + myName + "_" +rb.getName () +
      // "_mesh.obj");
      // rb.getSurfaceMesh().print (inRBMeshString);
      // Get all vertices and faces of each rigid body and print its
      // index and parameters
   }

   /**
    * Adds muscle information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param muscles
    * list of {@link MultiPointMuscle} objects
    */
   private void writeMuscleInfo (
      StringBuilder output, List<MultiPointMuscle> muscles) {
      output
         .append ("MUSCLES\n").append ("Number of muscles: ")
         .append (muscles.size ()).append ("\n\n");

      muscles.forEach (msc -> {
         Thelen2003AxialMuscle mat = (Thelen2003AxialMuscle)msc.getMaterial ();
         output
            .append ("Name: ").append (msc.getName ()).append ("\n")
            .append ("Material: ").append (mat.getClass ().getSimpleName ())
            .append ("\n").append ("Max isometric force: ")
            .append (mat.getMaxIsoForce ()).append ("\n")
            .append ("Rest length: ")
            .append (String.format ("%.3f", msc.getRestLength ())).append ("\n")
            .append ("Opt fiber length: ")
            .append (String.format ("%.3f", mat.getOptFibreLength ()))
            .append ("\n").append ("Tendon slack length: ")
            .append (String.format ("%.3f", mat.getTendonSlackLength ()))
            .append ("\n").append ("Number of points: ")
            .append (msc.numPoints ()).append ("\n");

         for (int i = 0; i < msc.numPoints (); i++) {
            output
               .append ("Point ").append (i).append ("\t")
               .append (msc.getPoint (i).getPosition ().toString ("%.3f"))
               .append ("\n");
         }
         output
            .append ("Number of segments: ").append (msc.numSegments ())
            .append ("\n").append ("Number of muscle wrappings: ")
            .append (msc.numWrappables ()).append ("\n");

         for (int i = 0; i < msc.numWrappables (); i++) {
            output
               .append ("Wrapping: ").append (i).append ("\t")
               .append (msc.getWrappableRange (i)).append ("\n");
         }
         output.append ("\n");
      });
   }

   /**
    * Adds physics information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param mech
    * current {@link MechModel}
    */
   private void writePhysicsInfo (StringBuilder output, MechModel mech) {
      output
         .append ("\nPHYSICS\n").append ("Frame Damping: ")
         .append (mech.getFrameDamping ()).append ("\n")
         .append ("Rotary Damping: ").append (mech.getRotaryDamping ())
         .append ("\n").append ("Inertial Damping: ")
         .append (mech.getInertialDamping ()).append ("\n")
         .append ("Point Damping: ").append(mech.getPointDamping ())
         .append ("\n").append ("Gravity: ").append (mech.getGravity ())
         .append ("\n");
   }

   /**
    * Adds all probes information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param controller
    * current {@link MotionTargetController} object
    * @param motion
    * {@link MarkerMotionData} object containing all exp trajectories
    * @param map
    * {@link Map} linking the experimental marker names and weights to model
    * markers
    * @param forces
    * {@link ForceData} object containing all exp forces
    * @param marker
    * list of {@link FrameMarker} objects
    */
   private void writeProbesInfo (
      StringBuilder output, TrackingController controller,
      MarkerMotionData motion, MarkerMapping map, ForceData forces,
      RenderableComponentList<FrameMarker> marker) {
      output
         .append ("\n%%PROBES%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nCONTROLLER INFORMATION\n").append ("Use motion targets : ")
         .append (controller.getMotionTargetTerm ().isEnabled ()).append ("\n")
         .append ("Use regularization: ")
         .append (controller.getL2RegularizationTerm ().isEnabled ())
         .append ("\n").append ("Use KKT Factorization: ")
         .append (controller.getUseKKTFactorization ()).append ("\n")
         .append ("Incremental computation: ")
         .append (controller.getComputeIncrementally ()).append ("\n")
         .append ("Normalize H: ").append (controller.getNormalizeH ())
         .append ("\n").append ("Use timestep scaling: ")
         .append (controller.getUseTimestepScaling ()).append ("\n")
         .append ("Integrator: ").append (controller.getIntegrator ())
         .append ("\n\n");
      // Calculate framerate of the marker trajectories
      double framerate =
         (motion.numFrames () - 1)
         / motion.getFrameTime (motion.numFrames () - 1);
      output
         .append (
            String
               .format (
                  "TRC File: %d, start time: %.3f, stop time: %.3f, frame rate: %.2f%n",
                  motion.numFrames (), motion.getFrameTime (0),
                  motion.getFrameTime (motion.numFrames () - 1), framerate));
      // Calculate framerate of the force data
      framerate =
         (forces.numFrames () - 1)
         / forces.getFrameTime (forces.numFrames () - 1);
      output
         .append (
            String
               .format (
                  "MOT File: %d, start time: %.3f, stop time: %.3f, frame rate: %.2f%n",
                  forces.numFrames (), forces.getFrameTime (0),
                  forces.getFrameTime (forces.numFrames () - 1), framerate));
      output
         .append ("Model markers: ").append (marker.size ()).append ("\t")
         .append ("Experimental markers: ").append (motion.numMarkers ())
         .append ("\n");
      // Formatting table header
      String header =
         String
            .format (
               "%-16s%-16s%-16s%-6s", "Model marker", "Exp. marker",
               "Attachment", "Weight");
      output.append (header).append ("\n");
      // Add marker pairs
      marker.forEach (mkr -> {
         // Entry<String,Double> entry = map.get (mkr.getName ());
         try {
            String label = map.getExpLabelFromModel (mkr.getName ());
            if (label == null) {
               output
                  .append (
                     String
                        .format (
                           "%-16s%-46s%n", mkr.getName (),
                           "No corresponding experimental marker detected."));
            }
            else {
               output
                  .append (
                     String
                        .format (
                           "%-16s%-16s%-16s%.1f%n", mkr.getName (), label,
                           mkr.getFrame ().getName (),
                           map.getMarkerWeight (label)));
            }
         }
         catch (Exception e) {
            e.printStackTrace ();
         }
      });
      // Add unassigned markers
      output.append ("\nNot assigned experimental markers:\n");
      // Retrieve all non null experimental marker names.
      List<String> assignedLabels =
         map
            .getExpLabels ().stream ().filter (Objects::nonNull)
            .collect (Collectors.toList ());

      // Check for each experimental markers that are not in the list of
      // assigned labels.
      motion
         .getMarkerLabels ().stream ()
         .filter (label -> !assignedLabels.contains (label))
         .forEach (label -> output.append (label).append ("\n"));
      output
         .append ("\nGenerated ")
         .append (controller.getMotionSources ().size ())
         .append (" marker pairs in total.\n");
   }

   /**
    * Adds solver information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param solver
    * {@link MechSystemSolver} of the current {@link MechModel}
    */
   private void writeSolverInfo (
      StringBuilder output, MechSystemSolver solver) {
      output
         .append ("\n%%GENERAL SIMULATION PARAMETERS%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nSOLVER\n").append ("Integrator: ")
         .append (solver.getIntegrator ().toString ()).append ("\n")
         .append ("Stabilization: ")
         .append (solver.getStabilization ().toString ()).append ("\n")
         .append ("Max Iterations: ").append (solver.getMaxIterations ())
         .append ("\n").append ("Matrix Solver: ")
         .append (solver.getMatrixSolver ().toString ()).append ("\n")
         .append ("Hybrid Solving enabled: ").append (solver.getHybridSolve ())
         .append ("\n");
   }
}