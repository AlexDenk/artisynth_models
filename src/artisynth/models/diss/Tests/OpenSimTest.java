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
import artisynth.core.inverse.ForceTargetTerm;
import artisynth.core.inverse.MotionTargetTerm;
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
   // Current System time during simulation
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
   // Hashmap, that matches the name of the model marker to the corresponding
   // experimental markers and their corresponding weightings
   // Map<String,Entry<String,Double>> myMkrMap;
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
   /**
    * Renders the experimental COP from ForceData
    * 
    * @author Alexander Denk
    */
   public class COPRenderer implements IsRenderable {
      Vector3d[] copPos = new Vector3d[2];
      Vector3d[] grfPos = new Vector3d[4];
      int scale;
      int f0;
      int f1;

      @Override
      public void prerender (RenderList list) {
         // Get current positions of the left and right COP
         f1 = (int)(mySystemTime / getMaxStepSize ());
         // Execute the following only once per frame
         if (f1 > f0) {
            copPos[0] = myForces.getData (f1, "Right COP");
            copPos[1] = myForces.getData (f1, "Left COP");
            // start point right
            FrameMarker mkrR = (FrameMarker)myMech.get ("cop_ref_1");
            grfPos[0] = mkrR.getPosition ();
            // end point right
            Vector3d buf =
               myForces.getData (f1, "Right GRF").scale (0.001 * scale);
            grfPos[1] = grfPos[1].add (grfPos[0], buf);
            // start point left
            FrameMarker mkrL = (FrameMarker)myMech.get ("cop_ref_2");
            grfPos[2] = mkrL.getPosition ();
            // end point left
            buf = myForces.getData (f1, "Left GRF").scale (0.001 * scale);
            grfPos[3] = grfPos[3].add (grfPos[2], buf);
         }
         f0 = f1;
         // TODO: handle simulation reset
      }

      @Override
      public void render (Renderer renderer, int flags) {
         renderer.setColor (Color.GRAY.brighter ());
         renderer.drawSphere (copPos[0], 0.01 * scale);
         renderer.drawSphere (copPos[1], 0.01 * scale);
         // right grf arrow
         if (getInputProbes ()
            .get ("right ground reaction forces").isActive ()) {
            renderer.drawArrow (grfPos[0], grfPos[1], 0.01 * scale, false);
         }
         // left grf arrow
         if (getInputProbes ()
            .get ("left ground reaction forces").isActive ()) {
            renderer.drawArrow (grfPos[2], grfPos[3], 0.01 * scale, false);
         }
      }

      @Override
      public void updateBounds (Vector3d pmin, Vector3d pmax) {
      }

      @Override
      public int getRenderHints () {
         return 0;
      }

      public COPRenderer (int s) {
         this.scale = s;
         this.f0 = 0;
         // Define initial drawing coordinates
         this.copPos[0] = new Vector3d (0, 0, 0);
         this.copPos[1] = new Vector3d (0, 0, 0);
         this.grfPos[0] = new Vector3d (0, 0, 0);
         this.grfPos[1] = new Vector3d (0, 0, 0);
         this.grfPos[2] = new Vector3d (0, 0, 0);
         this.grfPos[3] = new Vector3d (0, 0, 0);
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
      // Set general simulation properties
      setSimulationProperties ();
      // Import the OpenSim Geometry
      initializeOsim (myName, myScale);
      // Import FE Meshes
      // initializeFEM();
      // Retrieve the collision manager to define contact properties
      CollisionManager collMan = myMech.getCollisionManager ();
      collMan.setName ("Collision manager");
      // Set individual contact properties and initialize the contact monitor
      setContactProps (collMan);
      // Define input and output probes
      defineIOProbes (myName, myScale);
      // Set render properties
      setRenderProps (collMan, myScale);
      // Write the generated Input parameters to a file within the working
      // directory
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
      // Define body - body contact render props
      coll.setDrawIntersectionPoints (true);
      coll.setDrawContactForces (true);
      coll.setDrawFrictionForces (true);
      coll.setContactForceLenScale (scale / 1000);
      // coll.setDrawColorMap (ColorMapType.PENETRATION_DEPTH);
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
      // Access source and target points of the motion target controller
      MotionTargetController controller =
         (MotionTargetController)getControllers ().get (0);
      controller.getTargetPoints ().forEach (c -> {
         RenderProps.setPointColor (c, Color.WHITE);
      });
      // FrameMarker mkrL = (FrameMarker)myMech.get ("cop_ref_1");
      // RenderProps
      // .setSphericalPoints (
      // mkrL, 0.01 * scale, Color.GRAY.darker ().darker ());
      // FrameMarker mkrR = (FrameMarker)myMech.get ("cop_ref_2");
      // RenderProps
      // .setSphericalPoints (
      // mkrR, 0.01 * scale, Color.GRAY.darker ().darker ());
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
      // Add a cop renderer to the list of renderables
      COPRenderer copRenderer = new COPRenderer (scale);
      getMainViewer ().addRenderable (copRenderer);
      // Setup connector rendering
      // setConnectorRenderProps (scale);
      // Setup contact rendering
      setContactRenderProps (coll, scale);
      // disable components coordinate systems rendering
      setBodyRenderProps ();
      // Muscle rendering
      setMuscleRenderProps (scale);
      // Marker rendering
      setMarkerRendering (scale);
      // Stress surface rendering
      // setSurfaceRenderProps ();
      // Viewer properties
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
         // Append solver info
         MechSystemSolver solver = myMech.getSolver ();
         writeSolverInfo (output, solver);
         // Append physics info
         writePhysicsInfo (output, myMech);
         // Append model info
         String modelPath = myName + "/gait2392_simbody_scaled.osim";
         writeModelInfo (output, modelPath, myBodies);
         // Append joint info
         writeJointInfo (output, myJoints);
         // Append muscle info
         writeMuscleInfo (output, myMuscles);
         // Append FEM info
         writeFEMInfo (output, myMeshes);
         // Append probes info
         MotionTargetController controller =
            (MotionTargetController)getControllers ().get ("Motion controller");
         writeProbesInfo (
            output, controller, myMotion, myMap, myForces, myMarkers);
         // Append contact info
         CollisionManager coll = myMech.getCollisionManager ();
         CollisionBehaviorList behav = coll.behaviors ();
         writeContactInfo (output, coll, behav);
         // write to file
         writer.print (output.toString ());
      }
   }

   // --------------------------Private Instance Methods------------------------

   /**
    * Defines n ({@code list.size())} {@link NumericInputProbe}s for the
    * specified bodies in {@code list} and fills it with the force data in
    * {@code forces}. Hands the generated Probes to the
    * {@link MotionTargetController}
    * 
    * @param motcon
    * MotionTargetController
    * @param list
    * List of Components to apply forces to. Needs to implement the property
    * {@code externalForce}
    * @param forces
    * {@link ForceTarget} object containing the experimental forces
    */
   private void addForceProbes (
      MotionTargetController motcon, ArrayList<FrameMarker> list,
      ForceData forces) {
      // Connect experimental data to the input probes generated by the
      // controller.
      if (list != null) {
         NumericInputProbe leftForces = new NumericInputProbe ();
         leftForces.setModel (myMech);
         leftForces.setName ("left ground reaction forces");
         NumericInputProbe rightForces = new NumericInputProbe ();
         rightForces.setModel (myMech);
         rightForces.setName ("right ground reaction forces");
         // Calculate the duration in seconds from the number of frames
         double duration =
            forces.getFrameTime (forces.numFrames () - 1)
            - forces.getFrameTime (0);
         leftForces.setStartStopTimes (0.0, duration);
         rightForces.setStartStopTimes (0.0, duration);
         Property[] props = new Property[1];
         props[0] = list.get (0).getProperty ("externalForce");
         leftForces.setInputProperties (props);
         props[0] = list.get (1).getProperty ("externalForce");
         rightForces.setInputProperties (props);
         for (int i = 0; i < forces.numFrames (); i++) {
            VectorNd force = new VectorNd (3);
            double time = forces.getFrameTime (i);
            force.add (0, forces.getData (i, "Right GRF").x);
            force.add (1, forces.getData (i, "Right GRF").y);
            force.add (2, forces.getData (i, "Right GRF").z);
            rightForces.addData (time, force);
            force.set (0, forces.getData (i, "Left GRF").x);
            force.set (1, forces.getData (i, "Left GRF").y);
            force.set (2, forces.getData (i, "Left GRF").z);
            leftForces.addData (time, force);
         }
         addInputProbe (rightForces);
         motcon.addInputProbe (rightForces);
         rightForces.setActive (false);
         addInputProbe (leftForces);
         motcon.addInputProbe (leftForces);
         leftForces.setActive (false);
      }
   }

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

   @Deprecated
   /**
    * Adds the data stored in {@code forces} to the input probes of the
    * MotionTargetController, if available.
    * 
    * @param forces
    * Experimental forces
    */
   private void addProbesToForceTargets (ForceData forces) {
      // Get target forces probe to fill with data
      NumericInputProbe expForces =
         (NumericInputProbe)getInputProbes ().get ("target forces");
      if (expForces != null) {
         for (int i = 0; i < forces.numFrames (); i++) {
            VectorNd force = new VectorNd (2);
            force.add (0, forces.getData (i, "Right GRF").y);
            force.add (1, forces.getData (i, "Left GRF").y);
            // force.add (2, forces.getData (i, "Right GRF").y);
            // force.add (3, forces.getData (i, "Left GRF").y);
            double time = forces.getFrameTime (i);
            expForces.addData (time, force);
         }
         expForces.setActive (true);
      }
   }

   /**
    * Adds the data stored in {@code motion} to the input probes of the
    * MotionTargetController, if available.
    * 
    * @param controller
    * MotionTargetController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param motion
    * Experimental marker trajectories
    */
   private void addProbesToMotionTargets (
      MotionTargetController controller, MarkerMapping map,
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
      // Add a collision behavior for each connection.
      CollisionBehavior behavior;
      behavior = myMech.setCollisionBehavior (bodyA, bodyB, true);
      // Set compliant contact properties
      behavior.setCompliance (comp);
      behavior.setDamping (damp);
      // Additional method to reduce overconstrained contact.
      behavior.setBilateralVertexContact (false);
      // Add a collision response for the contact history
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
      // Add widget for each joint property
      panel.addWidget (jt, prop);
      // Define, where the NumOutProbe is written to.
      String filepath =
         PathFinder.getSourceRelativePath (this, "/" + prop + ".txt");
      // Define NumericOutPutProbe
      NumericOutputProbe probe =
         new NumericOutputProbe (jt, prop, filepath, step);
      // Set probe properties
      probe.setName (prop);
      probe.setStartStopTimes (start, stop);
      // Add the probe to the controller.
      addOutputProbe (probe);
   }

   /**
    * Defines a {@link MotionTargetController} object, that calculates muscle
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
      ForceData forces, MarkerMotionData motion, MarkerMapping map, String name,
      int scale)
      throws IOException {
      // Initialize controller
      MotionTargetController motcon =
         new MotionTargetController (myMech, "Motion controller", name);
      motcon.addForceData (forces);
      motcon.addMotionData (motion, map);
      motcon.addL2RegularizationTerm (1);
      // Calculate the duration in seconds from the number of frames
      double duration =
         motion.getFrameTime (motion.numFrames () - 1)
         - motion.getFrameTime (0);
      motcon.setProbeDuration (duration);
      // Enable KKT Factorization
      motcon.setUseKKTFactorization (true);
      // Enable debug mode
      motcon.setDebug (false);
      // Define motion targets
      defineMotionTargets (motcon, map, scale);
      // Define force targets, currently legacy code
      // defineForceTargets (motcon, scale);
      // Add controller before populating the input probes
      motcon.createProbesAndPanel (this);
      addController (motcon);
      // Populate probes of motion targets
      addProbesToMotionTargets (motcon, map, motion);
      // Populate probes of force targets, currently legacy code
      // addProbesToForceTargets (forces);

      //Define FrameMarkers for the force input probes
      ArrayList<FrameMarker> list = new ArrayList<FrameMarker> ();
      FrameMarker leftCOP = new FrameMarker ("cop_ref_1");
      leftCOP.setFrame (myBodies.get ("calcn_l"));
      myMech.add (leftCOP);
      list.add (leftCOP);
      motcon.addCOPReference (leftCOP);
      FrameMarker rightCOP = new FrameMarker ("cop_ref_2");
      rightCOP.setFrame (myBodies.get ("calcn_r"));
      myMech.add (rightCOP);
      list.add (rightCOP);
      motcon.addCOPReference (rightCOP);
      addForceProbes (motcon, list, myForces);
   }

   @Deprecated
   /**
    * Defines ForceTargets of the {@link ForceTargetTerm} of the
    * {@link MotionTargetController}.
    * 
    * @param controller
    * MotionTargetController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    */
   private void defineForceTargets (
      MotionTargetController controller, int scale) {
      // Define planar connectors for each rigid body with ground contact
      RigidBody calcnR = myBodies.get ("calcn_r");
      RigidBody calcnL = myBodies.get ("calcn_l");
      RigidBody toesR = myBodies.get ("toes_r");
      RigidBody toesL = myBodies.get ("toes_l");
      // Define transform of the connector to world coordinates
      RigidTransform3d rt = new RigidTransform3d (0, 0, 0);
      // Rotate by -90° around the x axis to align the connector from the xy
      // plane to the xz plane while facing upwards
      rt.R.mulAxisAngle (1, 0, 0, Math.toRadians (-90));
      // Define the distance offset
      Vector3d dOff = new Vector3d (0.0, 0.0, 0.0);
      // Setup the calcn planar connectors
      PlanarConnector conCalcnR = new PlanarConnector (calcnR, dOff, rt);
      conCalcnR.setPlaneSize (3 * scale);
      conCalcnR.setName ("conCalcnR");
      conCalcnR.setUnilateral (true);
      PlanarConnector conCalcnL = new PlanarConnector (calcnL, dOff, rt);
      conCalcnL.setPlaneSize (3 * scale);
      conCalcnL.setName ("conCalcnL");
      conCalcnL.setUnilateral (true);
      // Setup the toes planar connectors
      PlanarConnector conToesR = new PlanarConnector (toesR, dOff, rt);
      conToesR.setPlaneSize (3 * scale);
      conToesR.setName ("conToesR");
      conToesR.setUnilateral (true);
      PlanarConnector conToesL = new PlanarConnector (toesL, dOff, rt);
      conToesL.setPlaneSize (3 * scale);
      conToesL.setName ("conToesL");
      conToesL.setUnilateral (true);
      // Add all connectors to the model
      myMech.addBodyConnector (conCalcnR);
      myMech.addBodyConnector (conCalcnL);
      myMech.addBodyConnector (conToesR);
      myMech.addBodyConnector (conToesL);
      // Add force targets to monitor the forces in those connectors, where
      // the external forces are applied
      ForceTargetTerm forceTerm = controller.addForceTargetTerm ();
      // Define two dummy force targets
      ForceTarget ftR = forceTerm.addForceTarget (conCalcnR);
      ftR.setName ("ft_r");
      ftR.setTargetLambda (new VectorNd (1));
      ftR.setArrowSize (0.1);
      ForceTarget ftL = forceTerm.addForceTarget (conCalcnL);
      ftL.setName ("ft_l");
      ftL.setTargetLambda (new VectorNd (1));
      ftL.setArrowSize (0.1);
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
      // Read experimental trajectories from file
      myMotion = readTRCFile (name, scale);
      // Read experimental force data from file
      myForces = readMOTFile (name);
      // Relate the experimental marker names to the model marker names
      // from file
      myMap = getMapFromFile (name);
      // Generate and populate motion and force targets
      defineControllerAndProps (myForces, myMotion, myMap, name, scale);
      // Define output probes for each joint angle
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
      // TODO: Generate an autosave/write method for all output probes
      addNumOutputProbesAndPanel (myMotion);
   }

   /**
    * Defines MotionTargets of the {@link MotionTargetTerm} of the
    * {@link MotionTargetController}.
    * 
    * @param controller
    * MotionTargetController
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    * @param scale
    * Unit scale factor for the current model (m = 1, mm = 1000)
    */
   private void defineMotionTargets (
      MotionTargetController controller, MarkerMapping map, int scale) {
      // Add muscles to compute the excitations
      myMuscles.forEach (msc -> {
         controller.addExciter (msc);
      });
      // Add each model marker with a corresponding experimental marker to the
      // list of controllable markers ("sources") for the tracking
      // controller. Confusingly, AddMotionTarget adds a source, but also
      // generates a corresponding target point.
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
    * based on the rigid bodies, that are part of the model. By accessing the
    * connectors of every body and writing them to a separate list. Connectors
    * are defined per rigid body, a body that is connected to 3 bodies, has
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
      // Check whether a jointset is already apparent in the model. If so,
      // then define joint constraints based on them. If not, retrieve them
      // from the rigid bodies of the model.
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
               // Write all joints to a joint list
               myJoints.add ((JointBase)rb.getConnectors ().get (0));
               int end = myJoints.size ();
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
   private MarkerMapping getMapFromFile (String name) throws IOException {
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
      // Store model markers
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
      // Store force components temporally
      RenderableComponentList<ModelComponent> forces =
         (RenderableComponentList<ModelComponent>)myMech.get ("forceset");
      // forces is itself a multicomponent structure and will be split up in
      // its forces and the corresponding attachment points
      forces.forEach (frc -> {
         // Address all children of the structure
         frc.getChildren ().forEachRemaining (obj -> {
            // Access the attachment points, currently does nothing
            if (obj instanceof PointList) {
               return;
            }
            // Access the muscles (that also contain the attachments)
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
      // Generate working directory defined by user input.
      fc.setCurrentDirectory (ArtisynthPath.getSrcRelativeFile (this, myName));
      // Set the file dialog to accept directories only
      fc.setFileSelectionMode (JFileChooser.DIRECTORIES_ONLY);
      // Set dialog title to be more specific
      fc.setDialogTitle ("Please select a working directory.");
      // Show file dialog
      fc.showOpenDialog (null);
      // Set name specifier
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
      // Store rigid body components
      myBodies = getBodiesFromOsim ();
      // Define and store joint constraints
      myJoints = getJointsFromOsim (myBodies);
      // Store muscle components
      myMuscles = getMusclesFromOsim ();
      // Store marker components
      myMarkers = getMarkerFromOsim ();
      // Set initial body pose
      setInitialPose ();
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
      // Retrieve the experimental forces from mot file
      // Specifiy mot file to be read
      String motName = name + "/Input/" + name + "_forces.mot";
      String motPath = ArtisynthPath.getSrcRelativePath (this, motName);
      MOTReader motReader = new MOTReader (new File (motPath));
      // Read mot file
      motReader.readData ();
      // Print reading details to the console
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
    */
   private void readOsimFile (String name, int scale) {
      // Define location and name of the loaded osim file
      String modelPath = myName + "/gait2392_simbody_scaled.osim";
      File osimFile = ArtisynthPath.getSrcRelativeFile (this, modelPath);
      String geometryPath = myName + "/Geometry/";
      File geometryFile = ArtisynthPath.getSrcRelativeFile (this, geometryPath);
      // Read the specified osim file
      OpenSimParser parser = new OpenSimParser (osimFile);
      parser.setGeometryPath (geometryFile);
      parser.createModel (myMech);
      // Scale the model from meters to millimeters
      myMech.scaleDistance (scale);
      // Set rigid body damping parameters
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
      // Retrieve experimental marker data from trc
      // Specify trc file name to be read
      String trcName = name + "/Input/" + name + "_positions.trc";
      String trcPath =
         ArtisynthPath.getSrcRelativePath (this, trcName).toString ();
      TRCReader trcReader = new TRCReader (new File (trcPath));
      // Read trc file
      trcReader.readData ();
      // Print reading details to the console
      System.out
         .println (
            "Experimental markers: " + trcReader.getMarkerLabels ().size ());
      System.out
         .println ("TRC file: read " + trcReader.getNumFrames () + " frames");
      // Store marker trajectories and corresponding frames
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
      // Enable reduce overconstrained contact.
      coll.setReduceConstraints (true);
      // Define compliant contact per joint in the OpenSim Model
      myJoints.forEach (jt -> {
         if (jt.getName ().contains ("pelvis")) {
            return;
         }
         // Access the respective bodies, that are connected to each joint.
         RigidBody bodyA = (RigidBody)jt.getBodyA ();
         RigidBody bodyB = (RigidBody)jt.getBodyB ();
         // Calculate compliant contact properties
         double comp = 0.1;
         double mass = bodyA.getMass () + bodyB.getMass ();
         double damp = 2 * 1 * Math.sqrt (1 / comp * mass);
         // Set collision behavior and response
         createCollision (bodyA, bodyB, comp, damp);
      });
      // Define Ground Contact
      // TODO: Ground contact auskonvergieren
      /*
       * RigidBody ground = (RigidBody)myMech.get ("ground"); RigidBody calcnR =
       * myBodies.get ("calcn_r"); RigidBody calcnL = myBodies.get ("calcn_l");
       * RigidBody toesR = myBodies.get ("toes_r"); RigidBody toesL =
       * myBodies.get ("toes_l"); // Calculate compliant contact for the body
       * weight with a softer // contact stiffness double comp = 1; double mass
       * = myMech.getActiveMass (); double damp = 2 * 1 * Math.sqrt (1 / comp *
       * mass); createCollision (ground, calcnR, comp, damp); createCollision
       * (ground, calcnL, comp, damp); createCollision (ground, toesR, comp,
       * damp); createCollision (ground, toesL, comp, damp);
       */
      // Initialize the contact monitor to handle all individual collision
      // responses
      ContactMonitor contMonitor =
         new ContactMonitor (coll.responses (), myName);
      contMonitor.setName ("Contact monitor");
      // Enable FullReportMode
      contMonitor.setUseFullReport (true);
      addMonitor (contMonitor);
      // TODO: Ground collision mesh
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
      // Activate adaptive stepping
      setAdaptiveStepping (true);
      setMaxStepSize (0.0017);
      // Define scale (mm = 1000, or m = 1)
      myScale = 1;
      // Define Gravity
      myMech.setGravity (new Vector3d (0, -9.81, 0));
      // Throw warning message in case of zero gravity
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
      // Access all joints to get their info.
      // String format = "%d DOF [%.1f, %.1f]stop time: %.3f, frame rate:
      // %.2f%n"
      // String format = "%-16s%-16s%-16s%-6s", "Model marker", "Exp. marker",
      // "Attachment", "Weight"
      String format = "%d DOF %-20s%-14s\tType: %-6s%n";
      // "%-16s%-16s%-16s%.1f%n"

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
         .append (mech.getInertialDamping ()).append ("\n").append ("Gravity: ")
         .append (mech.getGravity ()).append ("\n");
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
      StringBuilder output, MotionTargetController controller,
      MarkerMotionData motion, MarkerMapping map, ForceData forces,
      RenderableComponentList<FrameMarker> marker) {
      output
         .append ("\n%%PROBES%%\n")
         .append (
            "%%------------------------------------------------------------%%\n")
         .append ("\nCONTROLLER INFORMATION\n").append ("Use motion targets : ")
         .append (controller.getMotionTargetTerm ().isEnabled ()).append ("\n")
         .append ("Use force targets: ").append (controller.hasForceTargets ())
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