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

import artisynth.core.femmodels.FemElement3dBase;
import artisynth.core.femmodels.FemFactory;
import artisynth.core.femmodels.FemModel.Ranging;
import artisynth.core.femmodels.FemModel.SurfaceRender;
import artisynth.core.femmodels.FemModel3d;
import artisynth.core.femmodels.FemNode3d;
import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.ConnectorForceRenderer;
import artisynth.core.inverse.ForceTarget;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.FrameExciter.WrenchComponent;
import artisynth.core.inverse.FrameExciter.WrenchDof;
import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.inverse.MotionTargetTerm;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.FemMaterial;
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
import artisynth.models.diss.CustomTRCReader;
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
import maspack.matrix.RigidTransform3d;
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
   List<FemModel3d> myMeshes = new ArrayList<FemModel3d> ();
   // Experimental and model marker names and weights
   MarkerMapping myMap;
   // Experimental marker trajectories
   MarkerMotionData myMotion;
   // A list of each muscle in the model
   List<MultiPointMuscle> myMuscles = new ArrayList<MultiPointMuscle> ();
   // Name of the current working directory
   String myName = null;
   // Scale of the model.
   double myScale;

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
      CollisionManager collMan = myMech.getCollisionManager ();
      setContactProps (collMan);
      initializeIOProbes (myName, myScale);
      setRenderProps (collMan, myScale);
      writeInputToFile (myName);
   }

   @Override
   public void prerender (RenderList list) {
      super.prerender (list);
      // Synchronize color bar/values in case they are changed. Do this *after*
      // super.prerender(), in case values are changed there.
      for (int i = 0; i < myMeshes.size (); i++) {
         ColorBar cbar =
            (ColorBar)(renderables ()
               .get (myMeshes.get (i).getName () + "_colorbar"));
         cbar.setColorMap (myMeshes.get (i).getColorMap ());
         DoubleInterval range = myMeshes.get (i).getStressPlotRange ();
         cbar.updateLabels (range.getLowerBound (), range.getUpperBound ());
      }
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
         rend.get (end - 1).setArrowSize (0.001 * scale);
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
   public void setContactRenderProps (CollisionManager coll, double scale) {
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
   public void setMarkerRendering (double scale) {
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
         RenderProps.setPointRadius (c, 0.01 * scale);
      });
   }
   
   /**
    * Sets the rendering properties of every mesh within the root model.
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMeshRendering (double scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      myMeshes.forEach (mesh -> {
         RenderProps.setLineStyle (mesh, LineStyle.LINE);
         RenderProps.setLineWidth (mesh, 1);
         RenderProps.setLineColor (mesh, Color.BLACK);
      });
   }

   /**
    * Sets the rendering properties of every muscle within the root model.
    * 
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMuscleRenderProps (double scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      myMuscles.forEach (msc -> {
         RenderProps.setLineColor (msc, Color.RED.darker ());
         RenderProps.setShading (msc, Shading.SMOOTH);
         //RenderProps.setLineStyle (msc, LineStyle.SPINDLE);
         RenderProps.setLineRadius (msc, 0.005 * scale);
         RenderProps
            .setSpindleLines (myMech, 0.02 * scale, Color.RED.darker ());
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
   public void setRenderProps (CollisionManager coll, double scale) {
      if (!(coll instanceof CollisionManager))
         throw new IllegalArgumentException (
            "Must use an object of class CollisionManager");
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      setContactRenderProps (coll, scale);
      RenderProps.setShading (myBodies, Shading.SMOOTH);
      setMuscleRenderProps (scale);
      setMarkerRendering (scale);
      setMeshRendering(scale);
      setSurfaceRenderProps ();
      setViewerProps ();
   }

   /**
    * Sets the render properties of the colour bars within the root model.
    */
   public void setSurfaceRenderProps () {
      ColorBar[] cbar = new ColorBar[myMeshes.size ()];
      for (int i = 0; i < myMeshes.size (); i++) {
         myMeshes.get (i).setSurfaceRendering (SurfaceRender.Stress);
         myMeshes.get (i).setStressPlotRanging (Ranging.Auto);
         cbar[i] = new ColorBar();
         cbar[i].setName (myMeshes.get (i).getName () + "_colorbar");
         cbar[i].setNumberFormat ("%.2f");
         cbar[i].populateLabels (0.0, 0.1, 10);
         cbar[i].setLocation (-100, 0.1 * i, 20, 0.8);
         addRenderable (cbar[i]);
      }
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
         String modelPath = myName + "/" + myName + "_scaled.osim";
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
    * Defines a tracking controller that calculates muscle activations based on
    * trajectories and sets its controller properties.
    * 
    * @param motion
    * {@link MarkerMotionData}
    * @param map
    * {@link MarkerMapping}
    * @param name
    * Name specifier of the current working directory
    */
   private TrackingController addControllerAndProps (
      MarkerMotionData motion, MarkerMapping map, String name) {
      TrackingController controller =
         new TrackingController (myMech, "Motion controller");
      // MotionTargetController motcon =
      // new MotionTargetController (myMech, "Motion controller", name);
      // motcon.addMotionData (motion, map);
      controller.setUseKKTFactorization (true);
      controller.setComputeIncrementally (false);
      controller.setExcitationDamping ();
      controller.setL2Regularization (10);
      // Adjust MotionTargetTerm properties
      MotionTargetTerm motionTerm = controller.getMotionTargetTerm ();
      motionTerm.setChaseTime (getMaxStepSize ());
      motionTerm.setUsePDControl (true);
      motionTerm.setKd (1 / getMaxStepSize ());
      motionTerm.setKp (1 / (getMaxStepSize () * motionTerm.getChaseTime ()));
      controller.createPanel (this);
      addController (controller);
      return controller;
   }
   
   /**
    * Adds the data stored in {@code myCoords} as a {@link NumericInputProbe}
    * per joint, if available.
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
               createCoordsInputProbe (
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
      // controller.addExciters (myMuscles);
      if (forces == null)
         return;
      myBodies.forEach (body -> {
         double maxForce = 100 * body.getMass ();
         double maxMoment = 5 * body.getMass ();
         createAndAddFrameExciters (
            controller, myMech, body, maxForce, maxMoment);
      });
      // Frame exciters calcaneus right
      // RigidBody calcnR = myBodies.get ("calcn_r");
      // double maxForce = forces.getMaxForce ("Right GRF");
      // double maxMoment = forces.getMaxMoment ("Right GRM");
      // createAndAddFrameExciters (
      // controller, myMech, calcnR, maxForce, maxMoment);
      // Frame exciters calcaneus left
      // RigidBody calcnL = myBodies.get ("calcn_l");
      // maxForce = forces.getMaxForce ("Left GRF");
      // maxMoment = forces.getMaxMoment ("Left GRM");
      // createAndAddFrameExciters (
      // controller, myMech, calcnL, maxForce, maxMoment);
      // Frame exciters pelvis
      // RigidBody pelvis = myBodies.get ("pelvis");
      // maxForce = 1000;
      // maxMoment = 20;
      // createAndAddFrameExciters (
      // controller, myMech, pelvis, maxForce, maxMoment);
   }

   /**
    * Adds the data stored in {@code myForces} as individual {@link NumericInputProbe} if
    * available.
    * 
    * @param forces
    * {@link ForceData}
    */
   private void addForceInputProbes (ForceData forces) {
      if (forces != null) {
         RigidBody calcnR = myBodies.get ("calcn_r");
         createForceInputProbe (forces, calcnR, "Right");
         RigidBody calcnL = myBodies.get ("calcn_l");
         createForceInputProbe (forces, calcnL, "Left");
      }
   }

   /**
    * Defines the frame targets of the {@link MotionTargetTerm} of the
    * {@link TrackingController} and creates a {@link NumericInputProbe} for
    * each target.
    * 
    * @param controller
    * TrackingController
    * @param motion
    * experimental marker trajectories
    */
   private void addFrameTargetsAndProbes (
      TrackingController controller, MarkerMotionData motion) {
      if (motion == null) {
         return;
      }
      myBodies.forEach (b -> {
         TargetFrame target = controller.addFrameTarget (b);
         try {
            createFrameTargetInputProbe (target, motion, "orientation");
            createFrameTargetInputProbe (target, motion, "position");
         }
         catch (IOException e) {
            e.printStackTrace ();
         }
      });
   }

   /**
    * Creates the default {@link NumericOutputProbe} of the
    * {@link TrackingController} and fills a control panel with all
    * corresponding properties if specified
    * 
    * @param motion
    * {@link MarkerMotionData}
    */
   private void addNumOutputProbesAndPanel (
      MarkerMotionData motion, TrackingController controller) {
      double start = motion.getFrameTime (0);
      double stop = motion.getFrameTime (motion.numFrames () - 1);
      // Default controller output probes
      if (controller != null) {
         String path =
            PathFinder
               .getSourceRelativePath (
                  this, myName + "/Output/tracked positions.txt");
         NumericOutputProbe trackedPos =
            InverseManager
               .createOutputProbe (
                  controller, ProbeID.TRACKED_POSITIONS, path, start, stop, -1);
         path =
            PathFinder
               .getSourceRelativePath (
                  this, myName + "/Output/source positions.txt");
         NumericOutputProbe sourcePos =
            InverseManager
               .createOutputProbe (
                  controller, ProbeID.SOURCE_POSITIONS, path, start, stop, -1);
         path =
            PathFinder
               .getSourceRelativePath (
                  this, myName + "/Output/computed excitations.txt");
         NumericOutputProbe compExc =
            InverseManager
               .createOutputProbe (
                  controller, ProbeID.COMPUTED_EXCITATIONS, path, start, stop,
                  -1);
         addOutputProbe (trackedPos);
         addOutputProbe (sourcePos);
         addOutputProbe (compExc);
      }
      // Joint angle output probes and panel
      ControlPanel jointPanel = new ControlPanel ("Joint Coordinates");
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
            musclePanel.addWidget (e.getName (), e, "excitation");
         });
         addControlPanel (musclePanel);
      }
      // Body pose output probes
      myBodies.forEach (b -> {
         createProbeAndPanel (b, null, "position", start, stop, step);
         createProbeAndPanel (b, null, "orientation", start, stop, step);
      });
   }
   
   /**
    * Defines the point targets of the {@link MotionTargetTerm} of the
    * {@link TrackingController} and creates a {@link NumericInputProbe} for each
    * target.
    * 
    * @param controller
    * TrackingController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param motion
    * experimental marker motion data
    */
   private void addPointTargetsAndProbes (
      TrackingController controller, MarkerMapping map,
      MarkerMotionData motion) {
      if (motion == null || map == null)
         return;
      double start = motion.getFrameTime (0);
      double stop = motion.getFrameTime (motion.numFrames () - 1);
      myMarkers.forEach (mkr -> {
         if (map.getExpLabelFromModel (mkr.getName ()) != null) {
            // Add point targets
            Double weight = map.getMarkerWeight (mkr.getName ());
            TargetPoint target = controller.addPointTarget (mkr, weight);
            // Add point target probes
            NumericInputProbe targetProbe =
               new NumericInputProbe (target, "position", start, stop);
            targetProbe.setName (mkr.getName () + "_position");
            for (int i = 0; i < motion.numFrames (); i++) {
               VectorNd data = new VectorNd ();
               String label = map.getExpLabelFromModel (mkr.getName ());
               data.append (motion.getMarkerPosition (i, label).x);
               data.append (motion.getMarkerPosition (i, label).y);
               data.append (motion.getMarkerPosition (i, label).z);
               double time = motion.getFrameTime (i);
               targetProbe.addData (time, data);
               // Adjust initial target position
               if (i == 0) {
                  Point3d pos = (Point3d)motion.getMarkerPosition (i, label);
                  target.setPosition (pos);
               }
            }
            addInputProbe (targetProbe);
         }
      });
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
      FrameExciter[] exs = new FrameExciter[3];
      exs[0] = new FrameExciter (null, frame, WrenchDof.FX, maxForce);
      exs[1] = new FrameExciter (null, frame, WrenchDof.FY, maxForce);
      exs[2] = new FrameExciter (null, frame, WrenchDof.FZ, maxForce);
      //exs[3] = new FrameExciter (null, frame, WrenchDof.MX, maxMoment);
      //exs[4] = new FrameExciter (null, frame, WrenchDof.MY, maxMoment);
      //exs[5] = new FrameExciter (null, frame, WrenchDof.MZ, maxMoment);
      // if the frame has a name, use this to create names for the exciters
      if (frame.getName () != null) {
         WrenchDof[] wcs = WrenchDof.values ();
         for (int i = 0; i < exs.length; i++) {
            exs[i]
               .setName (
                  frame.getName () + "_" + wcs[i].toString ().toLowerCase ());
         }
      }
      Double[] weights = new Double[] { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
      if (mech != null) {
         for (int i = 0; i < exs.length; i++) {
            mech.addForceEffector (exs[i]);
            ctrl.addExciter (1.0, exs[i]);
         }
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
   private void createCoordsInputProbe (
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
   private void createForceInputProbe (
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
    * Generates a {@link NumericInputProbe} of the specified property based on
    * the associated input files from parametric control runs.
    * 
    * @param frame
    * component related to the input probe
    * @param motion
    * experimental motion data containing the time series
    * @param prop
    * property name
    * @return VectorNd array containing the probe data
    * @throws IOException
    * if attached file doesn't exist
    */
   private void createFrameTargetInputProbe (
      Frame frame, MarkerMotionData motion, String prop)
      throws IOException {
      String file =
         ArtisynthPath
            .getSrcRelativePath (
               this,
               myName + "/Input/" + frame.getName () + " " + prop + ".txt");
      NumericInputProbe probe = new NumericInputProbe (frame, prop, file);
      probe.setName (frame.getName () + " " + prop);
      VectorNd[] data = new VectorNd[motion.numFrames ()];
      for (int i = 0; i < data.length; i++) {
         double time = motion.getFrameTime (i);
         data[i] = probe.getData (time);
      }
      addInputProbe (probe);
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
    * Queries all {@link RigidBody} objects from the current {@link MechModel}
    * and stores them in a global variable.
    * 
    * @return list of rigid bodies
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<RigidBody> getBodiesFromOsim () {
      myBodies = (RenderableComponentList<RigidBody>)myMech.get ("bodyset");
      // Check for unclosed meshes
      myBodies.forEach (b -> {
         PolygonalMesh mesh = b.getCollisionMesh ();
         System.out.print (b.getName () + ": read ");
         System.out.println (mesh.getFaces ().size () + " faces.");
         if (!mesh.isClosed ()) {
            System.err.println ("Warning: Mesh not closed!");
         }
      });
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
               case "acromial_r":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-120, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "elbow_r":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radioulnar_r":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radius_hand_r":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  break;
               case "acromial_l":
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  range.set (-120, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  range.set (-90, 90);
                  myJoints.get (end - 1).setCoordinateRangeDeg (2, range);
                  break;
               case "elbow_l":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radioulnar_l":
                  range.set (0, 150);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "radius_hand_l":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  myJoints.get (end - 1).setCoordinateRangeDeg (1, range);
                  break;
               case "r_scapulothoracic":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "r_sternoclavicular":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_scapulothoracic":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
                  break;
               case "l_sternoclavicular":
                  range.set (0, 0);
                  myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
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
            case "acromial_r":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-120, 90);
               jt.setCoordinateRangeDeg (1, range);
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "elbow_r":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radioulnar_r":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radius_hand_r":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               break;
            case "acromial_l":
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (0, range);
               range.set (-120, 90);
               jt.setCoordinateRangeDeg (1, range);
               range.set (-90, 90);
               jt.setCoordinateRangeDeg (2, range);
               break;
            case "elbow_l":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radioulnar_l":
               range.set (0, 150);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "radius_hand_l":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               jt.setCoordinateRangeDeg (1, range);
               break;
            case "r_scapulothoracic":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "r_sternoclavicular":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_scapulothoracic":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "l_sternoclavicular":
               range.set (0, 0);
               jt.setCoordinateRangeDeg (0, range);
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
   private void initializeIOProbes (String name, double scale) throws IOException {
      // Read all input data
      myMap = readMarkerFile (name);
      myMotion = readTRCFile (name, scale, myMap);
      myForces = readForceFile (name);
      myCoords = readCoordsFile (name);
      
      // Setup the tracking controller
      //TrackingController controller =
      //   addControllerAndProps (myMotion, myMap, name);
      //addExcitersToController (controller, myForces);
      //addPointTargetsAndProbes (controller, myMap, myMotion);
      //addFrameTargetsAndProbes (controller, myMotion);
      //addForceInputProbes (myForces);

      // Parametric control
      addCoordsInputProbes (myCoords);
  
      // Add output probes
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
      addNumOutputProbesAndPanel (myMotion, null);
      addBreakPoint (myMotion.getFrameTime (myMotion.numFrames () - 1));
   }

   /**
    * Imports an OpenSim model from the current working directory, defines joint
    * constraints and stores important model components in variables.
    * 
    * @param myName
    * Name specifier of the current working directory
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    * @throws IOException 
    */
   private void initializeOsim (String myName, double scale) throws IOException {
      readOsimFile (myName, scale);    
      myBodies = getBodiesFromOsim ();
      myJoints = getJointsFromOsim (myBodies);
      myMuscles = getMusclesFromOsim ();
      myMarkers = getMarkerFromOsim ();
      myMech.scaleDistance (scale);
      myMech.scaleMass (scale); 
      setInitialPose (myName);
      // Update muscle wrapping after position update
      myMech.updateWrapSegments();
   }

   /**
    * Returns joint angles as generalized coordinates from prior IK calculations
    * or MoCap systems ({@code name_angles.mot}) in the input folder of the
    * current working directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return {@link CoordinateData} object
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
    * @return {@link ForceData} object
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
    * @return {@link MarkerMapping} object
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
      // Skip first line (header)
      reader.readLine (); 
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
    * Reads a mesh file named by label in the specified working folder and creates
    * corresponding models and materials.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param label
    * label of the file, that contains the desired mesh
    * @return {@link FemModel3d} 
    * @throws IOException
    * if the files do not exist
    */
   private FemModel3d readMeshFile (String name, String label, double scale) throws IOException {
      String fileName = name + "/Meshes/C01L_" + label + ".obj";
      File file =
         ArtisynthPath.getSrcRelativeFile (this, fileName);
      PolygonalMesh mesh = new PolygonalMesh ();
      FemModel3d model = new FemModel3d (label);
      mesh.read (file);
      FemFactory.createFromMesh (model, mesh, 1.2);
      myMech.addModel (model);
      model.scaleDistance (scale / 1000);
      model.scaleMass (scale / 1000);
      return model;
   }

   /**
    * Reads in the .osim file in the specified working folder and creates a
    * corresponding model the current {@link MechModel}.
    * 
    * @param name
    * Name specifier for the current working directory
    * @param scale
    * Scaling factor for the current model (m = 1, mm = 1000)
    */
   private void readOsimFile (String name, double scale) {
      String modelPath = myName + "/" + myName + "_scaled.osim";
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
    * @param map
    * model and experimental marker names and weights
    * @return {@link MarkerMotionData} object
    * @throws IOException
    * if the file or file path are invalid
    */
   private MarkerMotionData readTRCFile (
      String name, double scale, MarkerMapping map)
      throws IOException {
      String trcName = name + "/Input/" + name + "_positions.trc";
      File trcFile = ArtisynthPath.getSrcRelativeFile (this, trcName);
      if (!trcFile.exists () || trcFile.isDirectory ()) {
         return null;
      }
      CustomTRCReader trcReader = new CustomTRCReader (trcFile, map);
      trcReader.readData ();
      System.out
         .println (
            "Experimental markers: " + trcReader.getMarkerLabels ().size ());
      System.out
         .println ("TRC file: read " + trcReader.getNumFrames () + " frames");
      MarkerMotionData motion = trcReader.getMotionData ();
      // Scale the marker trajectories individually, since there is no
      // general .scale () method for marker positions if the unit is not
      // already mm
      if (trcReader.getUnits ().equals ("mm")) {
         for (int i = 0; i <= motion.numFrames () - 1; i++) {
            motion.getMarkerPositions (i).forEach (p -> {
               p.x = p.x * scale / 1000;
               p.y = p.y * scale / 1000;
               p.z = p.z * scale / 1000;
            });
         }
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
    * Sets the initial position of the OpenSim model by reading
    * {@code myName_startup_positions.txt} in the input folder.
    * 
    * @param name Name specifier of the current model
    * @throws IOException 
    */
   private void setInitialPose (String name) throws IOException {
      String fileName = name + "/Input/" + name + "_startup_position.txt";
      File file = ArtisynthPath.getSrcRelativeFile (this, fileName);
      if (!file.exists () || file.isDirectory ()) {
         return;
      }
      BufferedReader reader = new BufferedReader (new FileReader (file));
      String line;
      // Skip first line (header)
      reader.readLine ();
      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         assert line.length () != 0;
         String[] tokens = line.split ("\t");
         if (tokens[2].equals ("ROTARY")) {
            myJoints
               .get (tokens[0]).setCoordinateDeg (
                  Integer.parseInt (tokens[1]), Double.parseDouble (tokens[3]));
         }
         else {
            myJoints
               .get (tokens[0]).setCoordinate (
                  Integer.parseInt (tokens[1]), Double.parseDouble (tokens[3]));
         }
      }
      reader.close ();
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
    * Sets the following simulation properties: 1. Solver, 2. Step sizes, 3.
    * global damping parameters, 4. model unit scaling and 6. gravity
    */
   private void setSimulationProperties () {
      // Solver properties
      MechSystemSolver.setHybridSolvesEnabled (false);
      MechSystemSolver solver = myMech.getSolver ();
      solver.setIntegrator (Integrator.ConstrainedBackwardEuler);
      solver.setStabilization (PosStabilization.GlobalStiffness);
      // solver.setMaxIterations (100);
      // solver.setTolerance (1e-5);
      // setMaxStepSize (1e-4);
      setAdaptiveStepping (true);
      // Damping properties
      myMech.setFrameDamping (0.07);
      myMech.setRotaryDamping (5.0);
      myMech.setInertialDamping (2.8);
      myMech.setPointDamping (10.0);
      // Define scale (mm = 1000, or m = 1)
      myScale = 1.0;
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
            .append ("Body or Group A: ")
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
   private void writeFEMInfo (StringBuilder output, List<FemModel3d> meshes) {
      output
         .append ("%%FINITE ELEMENTS%%\n").append (
            "%%------------------------------------------------------------%%\n");
      output
         .append ("\nNumber of Meshes: ").append (myMeshes.size ())
         .append ("\n\n");
      myMeshes.forEach (mesh -> {
         output
            .append (mesh.getName ()).append ("\t")
            .append (String.format ("%.3f", mesh.getMass ())).append ("\tkg\n")
            .append ("Number of Nodes: ").append (mesh.numNodes ())
            .append ("\n").append ("Number of Elements: ")
            .append (mesh.numAllElements ()).append ("\n").append ("Density: ")
            .append (mesh.getDensity ()).append ("\tkg/m^3\n");
         LinearMaterial material = (LinearMaterial)mesh.getMaterial ();
         output
            .append ("Young's modulus: ").append ("\t")
            .append (material.getYoungsModulus ()).append ("\tN/m^2\n")
            .append ("Poisson's ratio: ").append (material.getPoissonsRatio ())
            .append ("\n");

         PointList<FemNode3d> nodes = mesh.getNodes ();
         nodes.forEach (n -> {
            output
               .append (n.getNumber ()).append ("\t")
               .append (n.getPosition ().toString ("%.3f")).append ("\n");
         });

         ArrayList<FemElement3dBase> elements = mesh.getAllElements ();
         elements.forEach (elem -> {
            output.append (elem.getNumber ()).append ("\t");
            FemNode3d[] elemNodes = elem.getNodes ();
            for (FemNode3d en : elemNodes) {
               output.append (en.getNumber ()).append ("\t");
            }
            output.append ("\n");
         });
         output.append ("\n");
      });
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
               .append (vt.getIndex ()).append ("\t")
               .append (vt.getPosition ().toString ("%.3f")).append ("\n");
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
         .append ("Use KKT Factorization: ")
         .append (controller.getUseKKTFactorization ()).append ("\n")
         .append ("Incremental computation: ")
         .append (controller.getComputeIncrementally ()).append ("\n")
         .append ("Use timestep scaling: ")
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
         .append ("Assigned experimental markers: ").append (motion.numMarkers ())
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
         .append ("\n").append ("Max Tolerance: ")
         .append (solver.getTolerance ()).append ("\n")
         .append ("Matrix Solver: ")
         .append (solver.getMatrixSolver ().toString ()).append ("\n")
         .append ("Hybrid Solving enabled: ").append (solver.getHybridSolve ())
         .append ("\n");
   }
}