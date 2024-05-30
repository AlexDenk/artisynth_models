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
import java.util.Random;
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
import artisynth.core.inverse.ForceTargetTerm;
import artisynth.core.inverse.MotionTargetTerm;
import artisynth.core.inverse.TrackingController;
import artisynth.core.materials.LinearMaterial;
import artisynth.core.materials.Thelen2003AxialMuscle;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionBehaviorList;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MechSystemSolver;
import artisynth.core.mechmodels.MechSystemSolver.Integrator;
import artisynth.core.mechmodels.MechSystemSolver.PosStabilization;
import artisynth.core.mechmodels.MeshComponentList;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.PlanarConnector;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.probes.TRCReader;
import artisynth.core.renderables.ColorBar;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.diss.ContactMonitor;
import artisynth.models.diss.MOTReader;
import artisynth.models.diss.MOTReader.ForceData;
import artisynth.models.diss.MarkerMapping;
import artisynth.models.diss.MotionTargetController;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.properties.Property;
import maspack.render.IsRenderable;
import maspack.render.RenderList;
import maspack.render.RenderProps;
import maspack.render.Renderer;
import maspack.render.Renderer.FaceStyle;
import maspack.render.Renderer.LineStyle;
import maspack.render.Renderer.Shading;
import maspack.render.Viewer.RotationMode;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

/**
 * @author Alexander Denk Copyright (c) 2023-2024, by the Author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */

public class OpenSimTest extends RootModel {
   // -------------------------------Static Fields------------------------------
   static double mySystemTime = 0.0;

   // ----------------------------Instance Fields-------------------------------
   // All rigid bodies in the model
   RenderableComponentList<RigidBody> myBodies = null;
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
   // Model and experimental marker names
   MarkerMapping myMap;
   // Experimental marker trajectories
   MarkerMotionData myMotion;
   // A list of each muscle in the model
   List<MultiPointMuscle> myMuscles = new ArrayList<MultiPointMuscle> ();
   // Name of the current working directory
   String myName = null;
   // Scale of the model.
   int myScale;

   // ------------------------------Nested Classes------------------------------

   // -----------------------------Constructors---------------------------------
   public OpenSimTest () {
   }

   public OpenSimTest (String name) throws IOException {
      super (name);
   }

   // --------------------------Static Methods----------------------------------
   public static void main (String[] args) throws IOException {
   }

   /**
    * Updates the current system time ({@code t1}) from any model component,
    * that is executed each time step, such as controllers, monitors. etc..
    * 
    * @param
    */
   public static void updateSystemTime (double t1) {
      mySystemTime = t1;
   }

   // -------------------------Instance Methods---------------------------------
   @Override
   public void build (String[] args) throws IOException {
      super.build (args);
      // Get model name specifier from user
      JFileChooser fc = new JFileChooser ();
      myName = getNameFromFileDiaglog (fc);
      myMech.setName (myName);
      addModel (myMech);
      setSimulationProperties ();
      initializeOsim (myName, myScale);
      // initializeFEM();
      CollisionManager collMan = myMech.getCollisionManager ();
      collMan.setName ("Collision manager");
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
      // disable components coordinate systems rendering
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
      RenderProps.setShading (myBodies, Shading.SMOOTH);
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
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    */
   public void setMarkerRendering (int scale) {
      myMarkers.forEach (m -> {
         RenderProps.setPointColor (m, Color.PINK);
      });
      TrackingController controller =
         (TrackingController)getControllers ().get (0);
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
      setContactRenderProps (coll, scale);
      setBodyRenderProps ();
      setMuscleRenderProps (scale);
      setMarkerRendering (scale);
      // setSurfaceRenderProps ();
      setViewerProps ();
   }

   /**
    * Sets the render properties of the colour bars within the root model.
    */
   public void setSurfaceRenderProps () {
      ColorBar cbar = new ColorBar (null);
      myMeshes.forEach (mesh -> {
         mesh.setSurfaceRendering (SurfaceRender.Stress);
         mesh.setStressPlotRanging (Ranging.Auto);
      });
      cbar.setName ("colorBar");
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
      String inputName = myName + "/Output/" + myName + "_input_file.txt";
      String inputPath =
         ArtisynthPath.getSrcRelativePath (this, inputName).toString ();
      try (PrintWriter writer =
         new PrintWriter (new FileWriter (inputPath, false))) {

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
         writeProbesInfo (
            output, controller, myMotion, myMap, myForces, myMarkers);
         CollisionManager coll = myMech.getCollisionManager ();
         CollisionBehaviorList behav = coll.behaviors ();
         writeContactInfo (output, coll, behav);
         writer.print (output.toString ());
      }
   }

   // --------------------------Private Instance Methods------------------------

   /**
    * Creates an output probe and fills a control panel for each DOF of each
    * joint in the model.
    * 
    * @param motion
    * {@link MarkerMotionData}
    */
   private void addNumOutputProbesAndPanel (MarkerMotionData motion) {
      // Define a control panel and add a widget for each output property
      ControlPanel panel = new ControlPanel ("Joint Coordinates");
      // Define Output Probes
      double start = motion.getFrameTime (0);
      double stop = motion.getFrameTime (motion.numFrames () - 1);
      double step = getMaxStepSize ();
      myJoints.forEach (jt -> {
         switch (jt.getName ()) {
            case "ground_pelvis":
               createProbeandPanel (
                  jt, panel, "pelvis_tilt", start, stop, step);
               createProbeandPanel (
                  jt, panel, "pelvis_list", start, stop, step);
               createProbeandPanel (
                  jt, panel, "pelvis_rotation", start, stop, step);
               createProbeandPanel (jt, panel, "pelvis_tx", start, stop, step);
               createProbeandPanel (jt, panel, "pelvis_ty", start, stop, step);
               createProbeandPanel (jt, panel, "pelvis_tz", start, stop, step);
               break;
            case "hip_r":
               createProbeandPanel (
                  jt, panel, "hip_flexion_r", start, stop, step);
               createProbeandPanel (
                  jt, panel, "hip_adduction_r", start, stop, step);
               createProbeandPanel (
                  jt, panel, "hip_rotation_r", start, stop, step);
               break;
            case "knee_r":
               createProbeandPanel (
                  jt, panel, "knee_angle_r", start, stop, step);
               break;
            case "ankle_r":
               createProbeandPanel (
                  jt, panel, "ankle_angle_r", start, stop, step);
               break;
            case "subtalar_r":
               createProbeandPanel (
                  jt, panel, "subtalar_angle_r", start, stop, step);
               break;
            case "mtp_r":
               createProbeandPanel (
                  jt, panel, "mtp_angle_r", start, stop, step);
               break;
            case "hip_l":
               createProbeandPanel (
                  jt, panel, "hip_flexion_l", start, stop, step);
               createProbeandPanel (
                  jt, panel, "hip_adduction_l", start, stop, step);
               createProbeandPanel (
                  jt, panel, "hip_rotation_l", start, stop, step);
               break;
            case "knee_l":
               createProbeandPanel (
                  jt, panel, "knee_angle_l", start, stop, step);
               break;
            case "ankle_l":
               createProbeandPanel (
                  jt, panel, "ankle_angle_l", start, stop, step);
               break;
            case "subtalar_l":
               createProbeandPanel (
                  jt, panel, "subtalar_angle_l", start, stop, step);
               break;
            case "mtp_l":
               createProbeandPanel (
                  jt, panel, "mtp_angle_l", start, stop, step);
               break;
            case "back":
               createProbeandPanel (
                  jt, panel, "lumbar_extension", start, stop, step);
               createProbeandPanel (
                  jt, panel, "lumbar_bending", start, stop, step);
               createProbeandPanel (
                  jt, panel, "lumbar_rotation", start, stop, step);
               break;
         }
      });
      addControlPanel (panel);
   }

   /**
    * Adds the data stored in {@code motion} to the input probes of the
    * TrackingController, if available.
    * 
    * @param controller
    * TrackingController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param motion
    * Experimental marker trajectories
    */
   private void addProbesToMotionTargets (
      TrackingController controller, MarkerMapping map,
      MarkerMotionData motion) {
      // Get target positions probe to fill with data
      NumericInputProbe expMotion =
         (NumericInputProbe)getInputProbes ().get ("target positions");
      // Add target positions to the probe frame by frame
      if (expMotion != null) {
         int size = controller.getMotionSources ().size ();
         for (int i = 0; i < motion.numFrames (); i++) {
            VectorNd mot = new VectorNd (size * 3);
            for (int j = 0; j < size; j++) {
               String name = controller.getMotionSources ().get (j).getName ();
               String label;
               try {
                  label = map.getExpLabelFromModel (name);
                  mot.add (3 * j, motion.getMarkerPosition (i, label).x);
                  mot.add (3 * j + 1, motion.getMarkerPosition (i, label).y);
                  mot.add (3 * j + 2, motion.getMarkerPosition (i, label).z);
               }
               catch (Exception e) {
                  e.printStackTrace ();
               }
            }
            double time = motion.getFrameTime (i);
            expMotion.addData (time, mot);
         }
         expMotion.setActive (true);
      }
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
      // Additional method to reduce overconstrained contact.
      behavior.setBilateralVertexContact (false);
      myMech.setCollisionResponse (bodyA, bodyB);
   }

   /**
    * Creates a NumericOutPutProbe for the given property {@code prop} of the
    * joint {@code jt} with the parameters {@code start}, {@code stop} and
    * {@code step}. Adds a widget to the control panel {@code panel} for each
    * {@code prop}.
    * 
    * @param jt
    * @param panel
    * @param prop
    * Name of the property
    * @param start
    * Start time of the probe
    * @param stop
    * Stop time of the probe
    * @param step
    * Output interval of the probe
    */
   private void createProbeandPanel (
      JointBase jt, ControlPanel panel, String prop, double start, double stop,
      double step) {
      panel.addWidget (jt, prop);
      String filepath =
         PathFinder.getSourceRelativePath (this, "/" + prop + ".txt");
      NumericOutputProbe probe =
         new NumericOutputProbe (jt, prop, filepath, step);
      probe.setName (prop);
      probe.setStartStopTimes (start, stop);
      addOutputProbe (probe);
   }

   /**
    * Defines a {@link TrackingController} object, that calculates muscle
    * activations based on trajectories. The motion target controller is a self
    * written class, that controls the movement of target markers (based on the
    * experimental trajectories and forces) for the model markers to follow.
    * 
    * @param forces
    * {@link ForceData}
    * @param motion
    * {@link MarkerMotionData}
    * @param map
    * {@link MarkerMapping} links the model marker names to the experimental
    * marker names and their weights
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @throws IOException
    */
   private void defineControllerAndProps (
      MarkerMotionData motion, MarkerMapping map, int scale)
      throws IOException {
      TrackingController motcon =
         new TrackingController (myMech, "Motion controller");
      motcon.addL2RegularizationTerm (1);
      // Calculate the duration in seconds from the number of frames
      double duration =
         motion.getFrameTime (motion.numFrames () - 1)
         - motion.getFrameTime (0);
      motcon.setProbeDuration (duration);
      motcon.setUseKKTFactorization (true);
      // motcon.setComputeIncrementally (true);
      motcon.getMotionTargetTerm ().setUsePDControl (true);
      motcon.getMotionTargetTerm ().setKp (0.5);
      motcon.setDebug (false);
      motcon.setExcitationBounds (0.05, 0.95);
      defineMotionTargets (motcon, map, scale);
      motcon.createProbesAndPanel (this);
      addController (motcon);
      addProbesToMotionTargets (motcon, map, motion);
   }

   /**
    * Defines all in- and outgoing probes for the model. Ingoing probes are
    * experimental marker trajectories and forces, outgoing probes are all joint
    * angles of the model.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @throws IOException
    */
   private void defineIOProbes (String name, int scale) throws IOException {
      myMotion = readTRCFile (name, scale);
      myForces = readMOTFile (name);
      myMap = readMarkerMappingFile (name);
      defineControllerAndProps (myMotion, myMap, scale);
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
      // TODO: Generate an autosave/write method for all output probes
      addNumOutputProbesAndPanel (myMotion);
   }

   /**
    * Defines MotionTargets of the {@link MotionTargetTerm} of the
    * {@link TrackingController}.
    * 
    * @param controller
    * TrackingController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    */
   private void defineMotionTargets (
      TrackingController controller, MarkerMapping map, int scale) {
      myMuscles.forEach (msc -> {
         controller.addExciter (msc);
      });
      myMarkers.forEach (mkr -> {
         try {
            if (map.getExpLabelFromModel (mkr.getName ()) != null) {
               Double mkrWeight = map.getMarkerWeight (mkr.getName ());
               controller.addMotionTarget (mkr, mkrWeight);
            }
         }
         catch (Exception e) {
            e.printStackTrace ();
         }
      });
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
    * based on the rigid bodies, that are part of the model by accessing the
    * connectors of every body and writing them to a separate list. Connectors
    * are defined per rigid body. So a body that is connected to 3 bodies, has
    * therefore three different connectors. The connectors in each rigid body
    * are ordered in such a way, that each joint is exactly addressed once, if
    * the first index of the connector list (getConnectors.get(0)) in every
    * rigid body is accessed.
    * 
    * @param myBodies
    * list of rigid bodies
    * @return list of {@link OpenSimCustomJoint}
    */
   @SuppressWarnings("unchecked")
   private RenderableComponentList<JointBase> getJointsFromOsim (
      RenderableComponentList<RigidBody> myBodies) {
      if (myMech.contains (myMech.get ("jointset"))) {
         System.out.println ("Generated joint constraints from jointset.");
         myJoints = (RenderableComponentList<JointBase>)myMech.get ("jointset");
         DoubleInterval range = new DoubleInterval ();
         myJoints.forEach (jt -> {
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
      }
      else {
         // Ground shares no joint with any rigid body else than the hip
         // so skip that, since hip is going to be addressed either way.
         myBodies.forEach (rb -> {
            if (rb.getName ().equals ("ground")) {
               return;
            }
            else {
               System.out
                  .println (
                     "Generated joint constraints from ridig body connectors.");
               myJoints.add ((JointBase)rb.getConnectors ().get (0));
               int end = myJoints.size ();
               DoubleInterval range = new DoubleInterval ();
               switch (rb.getName ()) {
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
      }
      return myJoints;
   }

   /**
    * Returns a map of marker names based on an input file. It accepts files in
    * a specific format: No head line, all lines divided by a tab. 1st column:
    * Name of the modelmarker 2nd column: Name of the experimental marker 3rd
    * Column: Marker weighting for the inverse solver If no experimental marker
    * exists for a given model marker, leave out the 2nd and 3rd column and
    * continue with next line.
    * 
    * @param name
    * Name specifier of the model working directory
    * @return {@link MarkerMapping} map
    * @throws IOException
    */
   private MarkerMapping readMarkerMappingFile (String name)
      throws IOException {
      String mapName = name + "/Input/" + name + "_markers.txt";
      File mapFile = ArtisynthPath.getSrcRelativeFile (this, mapName);
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
      MarkerMapping map = new MarkerMapping (modelLabels, expLabels, weights);
      return map;
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
      myMarkers.forEach (marker -> {
         RigidBody newFrame = null;
         Vector3d newRef;
         Vector3d pos;
         if (marker.getName ().contains ("R_Toe")) {
            newFrame = myBodies.get ("toes_r");
         }
         else if (marker.getName ().contains ("L_Toe")) {
            newFrame = myBodies.get ("toes_l");
         }
         else {
            return;
         }
         Point3d newLoc = new Point3d (0, 0, 0);
         newRef = (Vector3d)newFrame.getPosition ();
         pos = (Vector3d)marker.getPosition ();
         newLoc.sub (pos, newRef);
         marker.setFrame (newFrame);
         marker.setRefPos ((Point3d)newRef);
         marker.setLocation (newLoc);
         System.out
            .println (
               "Warning: Attachment adjusted for: " + marker.getName ()
               + " to: " + newFrame.getName ());
      });
      // Provide some info in the console
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
      // forces is itself a multicomponent structure and will be split up in
      // its forces and the corresponding attachment points
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
      setInitialPose ();
      setInitialExcitations ();
   }

   /**
    * Returns a {@link ForceData} object from the available experimental force
    * data ({@code name}_forces.mot) in the input folder of the current working
    * directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @return
    * @throws IOException
    */
   private ForceData readMOTFile (String name) throws IOException {
      String motName = name + "/Input/" + name + "_forces.mot";
      String motPath = ArtisynthPath.getSrcRelativePath (this, motName);
      MOTReader motReader = new MOTReader (new File (motPath));
      motReader.readData ();
      System.out
         .println ("Experimental force data: " + motReader.getNumLabels ());
      System.out
         .println ("MOT file: read " + motReader.getNumFrames () + " frames");
      return motReader.getForceData ();
   }

   /**
    * Reads in the .osim file in the specified working folder and creates a
    * corresponding model the current {@link MechModel}.
    * 
    * @param name
    * Name specifier for the current working directory
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    */
   private void readOsimFile (String name, int scale) {
      String modelPath = myName + "/gait2392_simbody_scaled.osim";
      File osimFile = ArtisynthPath.getSrcRelativeFile (this, modelPath);
      String geometryPath = myName + "/Geometry/";
      File geometryFile = ArtisynthPath.getSrcRelativeFile (this, geometryPath);
      OpenSimParser parser = new OpenSimParser (osimFile);
      parser.setGeometryPath (geometryFile);
      parser.createModel (myMech);
      myMech.scaleDistance (scale);
      myMech.setFrameDamping (0.01);
      myMech.setRotaryDamping (0.2);
      // myMech.setInertialDamping (0.1);
   }

   /**
    * Returns a {@link MarkerMotionData} object from the available experimental
    * motion data ({@code name}_positions.trc) in the input folder of the
    * current working directory.
    * 
    * @param name
    * Name specifier of the current working directory
    * @param scale
    * Unit scaling factor for the current model (m = 1, mm = 1000)
    * @return
    * @throws IOException
    */
   private MarkerMotionData readTRCFile (String name, int scale)
      throws IOException {
      String trcName = name + "/Input/" + name + "_positions.trc";
      String trcPath =
         ArtisynthPath.getSrcRelativePath (this, trcName).toString ();
      TRCReader trcReader = new TRCReader (new File (trcPath));
      trcReader.readData ();
      System.out
         .println (
            "Experimental markers: " + trcReader.getMarkerLabels ().size ());
      System.out
         .println ("TRC file: read " + trcReader.getNumFrames () + " frames");
      MarkerMotionData motion = trcReader.getMotionData ();
      // Scale the marker trajectories individually, since there is no general
      // .scale () method for marker positions
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
    * @throws
    */
   private void setContactProps (CollisionManager coll) throws IOException {
      coll.setReduceConstraints (true);
      myJoints.forEach (jt -> {
         if (jt.getName ().contains ("pelvis")) {
            return;
         }
         RigidBody bodyA = (RigidBody)jt.getBodyA ();
         RigidBody bodyB = (RigidBody)jt.getBodyB ();
         double comp = 0.1;
         double mass = bodyA.getMass () + bodyB.getMass ();
         double damp = 2 * 1 * Math.sqrt (1 / comp * mass);
         createCollision (bodyA, bodyB, comp, damp);
      });
      ContactMonitor contMonitor =
         new ContactMonitor (coll.responses (), myName);
      contMonitor.setName ("Contact monitor");
      contMonitor.setUseFullReport (true);
      addMonitor (contMonitor);
   }

   /**
    * Defines a control panel for the excitations of each muscle and sets
    * initial values.
    */
   private void setInitialExcitations () {
      ControlPanel panel = new ControlPanel ("Muscle excitations");
      myMuscles.forEach (muscle -> {
         panel.addWidget (muscle.getName (), muscle, "excitation");
         muscle.setExcitation (new Random ().nextDouble ());
      });
      addControlPanel (panel);
   }

   /**
    * Sets the initial pose of the OpenSim model.
    */
   private void setInitialPose () {
      myJoints.get ("ground_pelvis").setCoordinate (3, 0.61);
      myJoints.get ("ground_pelvis").setCoordinate (4, 0.97);
      myJoints.get ("ground_pelvis").setCoordinate (5, 0.036);
      myJoints.get ("hip_r").setCoordinateDeg (0, -14);
      myJoints.get ("knee_r").setCoordinateDeg (0, -10);
      myJoints.get ("ankle_r").setCoordinateDeg (0, 3);
      myJoints.get ("hip_l").setCoordinateDeg (0, 25);
      myJoints.get ("knee_l").setCoordinateDeg (0, -4);
      myJoints.get ("ankle_l").setCoordinateDeg (0, -9);
      myJoints.get ("back").setCoordinateDeg (0, -17);
   }

   /**
    * Sets the following simulation properties: 1. Integrator, 2. Stabilization
    * method, 3. Adaptive time stepping, 4. maximum step size, 5. unit scale of
    * the model 6. gravity
    */
   private void setSimulationProperties () {
      MechSystemSolver solver = myMech.getSolver ();
      solver.setIntegrator (Integrator.Trapezoidal);
      // Use global stiffness, since more accurate and stable
      solver.setStabilization (PosStabilization.GlobalStiffness);
      setAdaptiveStepping (true);
      setMaxStepSize (0.0017);
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
            .append (jt.getBodyB ().getName ()).append ("\n");
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
         .append ("Number of bodies: ").append (bodies.size ()).append ("\n\n");
      bodies.forEach (rb -> {
         if (!rb.getCollisionMesh ().isClosed ()) {
            output
               .append ("Unclosed Mesh found: ").append (rb.getName ())
               .append ("\n");
         }
      });
      output.append ("\n");

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
         .append (mech.getInertialDamping ()).append ("\n").append ("Gravity: ")
         .append (mech.getGravity ()).append ("\n");
   }

   /**
    * Adds all probes information to the output string.
    * 
    * @param output
    * {@link StringBuilder} string
    * @param controller
    * current {@link TrackingController} object
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
         .append ("\n").append ("Use regularization: ")
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
      String header =
         String
            .format (
               "%-16s%-16s%-16s%-6s", "Model marker", "Exp. marker",
               "Attachment", "Weight");
      output.append (header).append ("\n");
      marker.forEach (mkr -> {
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

      output.append ("\nNot assigned experimental markers:\n");
      List<String> assignedLabels =
         map
            .getExpLabels ().stream ().filter (Objects::nonNull)
            .collect (Collectors.toList ());
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