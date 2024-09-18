package artisynth.models.diss.Tests;

import java.awt.Color;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.gui.ControlPanel;
import artisynth.core.inverse.FrameExciter;
import artisynth.core.inverse.FrameExciter.WrenchDof;
import artisynth.core.inverse.InverseManager;
import artisynth.core.inverse.InverseManager.ProbeID;
import artisynth.core.inverse.MotionTargetTerm;
import artisynth.core.inverse.TargetFrame;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionManager;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.JointBase;
import artisynth.core.mechmodels.MechModel;
import artisynth.core.mechmodels.MultiPointMuscle;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ComponentListView;
import artisynth.core.modelbase.Controller;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.modelbase.RenderableComponentList;
import artisynth.core.opensim.OpenSimParser;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.probes.NumericOutputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.diss.ContactMonitor;
import artisynth.models.diss.CoordinateData;
import artisynth.models.diss.CustomTRCReader;
import artisynth.models.diss.MOTReader;
import artisynth.models.diss.MarkerMapping;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.AxisAlignedRotation;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.matrix.VectorNd;
import maspack.render.RenderProps;
import maspack.render.Renderer.Shading;
import maspack.render.Viewer.RotationMode;
import maspack.util.DoubleInterval;
import maspack.util.PathFinder;

/**
 * @author Alexander Denk Copyright (c) 2024, by the Author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */

public class InverseTest extends RootModel {
   // ----------------------------Instance Fields------------------------------
   CoordinateData myCoords;
   MechModel myMech = new MechModel ();
   RenderableComponentList<RigidBody> myBodies = null;
   RenderableComponentList<JointBase> myJoints;
   RenderableComponentList<FrameMarker> myMarkers = null;
   MarkerMapping myMap;
   MarkerMotionData myMotion;
   List<MultiPointMuscle> myMuscles = new ArrayList<MultiPointMuscle> ();
   int myScale;

   // -----------------------------Constructors--------------------------------
   public InverseTest () {

   }

   public InverseTest (String name) throws IOException {
      super (name);
   }

   // --------------------------Static Methods----------------------------------
   public static void main (String[] args) throws IOException {
   }

   // -------------------------Instance Methods---------------------------------
   public void build (String[] args) throws IOException {
      addModel (myMech);
      myMech.setName ("InverseTest");
      myMech.setGravity (new Vector3d (0, 0, -9.81));
      myMech.setFrameDamping (0.01);
      myMech.setRotaryDamping (0.2);
      myScale = 1;
      myName = "InverseTest";
      importOsim ();
      CollisionManager collMan = myMech.getCollisionManager ();
      setContactProps (collMan);
      initializeIOProbes (myName, myScale);
      setRenderProps (collMan, myScale);
   }

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

   public void setMarkerRendering (int scale) {
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

   public void setMuscleRenderProps (int scale) {
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      myMuscles.forEach (msc -> {
         RenderProps.setLineColor (msc, Color.RED.darker ());
         RenderProps.setShading (msc, Shading.SMOOTH);
         // RenderProps.setLineStyle (msc, LineStyle.SPINDLE);
         RenderProps.setLineRadius (msc, 0.005 * scale);
         RenderProps
            .setSpindleLines (myMech, 0.02 * scale, Color.RED.darker ());
         msc.setExcitationColor (Color.GREEN);
      });
      myMech.setMaxColoredExcitation (1.0);
   }

   public void setRenderProps (CollisionManager coll, int scale) {
      if (!(coll instanceof CollisionManager))
         throw new IllegalArgumentException (
            "Must use an object of class CollisionManager");
      if (scale < 0)
         throw new IllegalArgumentException ("Scale factor must be > 0");
      setContactRenderProps (coll, scale);
      RenderProps.setShading (myBodies, Shading.SMOOTH);
      setMuscleRenderProps (scale);
      setMarkerRendering (scale);
      setViewerProps ();
   }

   public void setViewerProps () {
      getMainViewer ().setOrthographicView (true);
      getMainViewer ().setRotationMode (RotationMode.CONTINUOUS);
      setDefaultViewOrientation (AxisAlignedRotation.X_Y);
      mergeAllControlPanels (true);
   }

   // ---------------------Private Instance Methods----------------------------
   private TrackingController addControllerAndProps (
      MarkerMotionData motion, MarkerMapping map, String name) {
      TrackingController controller =
         new TrackingController (myMech, "Motion controller");
      controller.setUseKKTFactorization (false);
      controller.setComputeIncrementally (false);
      controller.setExcitationDamping ();
      controller.setL2Regularization (1);
      // Adjust MotionTargetTerm properties
      MotionTargetTerm motionTerm = controller.getMotionTargetTerm ();
      motionTerm.setUsePDControl (true);
      motionTerm.setKd (100);
      motionTerm.setKp (10000);
      controller.createPanel (this);
      addController (controller);
      return controller;
   }

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

   private void addExcitersToController (TrackingController controller) {
      // Muscles
      controller.addExciters (myMuscles);
      // Frame Exciters
      myBodies.forEach (body -> {
         double maxForce = 1 * body.getMass ();
         double maxMoment = 3 * body.getMass ();
         createAndAddFrameExciters (
            controller, myMech, body, maxForce, maxMoment);
      });
   }

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

   private FrameExciter[] createAndAddFrameExciters (
      TrackingController ctrl, MechModel mech, Frame frame, double maxForce,
      double maxMoment) {
      FrameExciter[] exs = new FrameExciter[6];
      exs[0] = new FrameExciter (null, frame, WrenchDof.FX, maxForce);
      exs[1] = new FrameExciter (null, frame, WrenchDof.FY, maxForce);
      exs[2] = new FrameExciter (null, frame, WrenchDof.FZ, maxForce);
      exs[3] = new FrameExciter (null, frame, WrenchDof.MX, maxMoment);
      exs[4] = new FrameExciter (null, frame, WrenchDof.MY, maxMoment);
      exs[5] = new FrameExciter (null, frame, WrenchDof.MZ, maxMoment);
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

   private void createCollision (
      RigidBody bodyA, RigidBody bodyB, double comp, double damp) {
      CollisionBehavior behavior;
      behavior = myMech.setCollisionBehavior (bodyA, bodyB, true);
      behavior.setCompliance (comp);
      behavior.setDamping (damp);
      myMech.setCollisionResponse (bodyA, bodyB);
   }

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

   private RenderableComponentList<JointBase> getJointsFromBodyset (
      double compMagnitude) {
      myBodies.forEach (body -> {
         myJoints.add ((JointBase)body.getConnectors ().get (0));
         int end = myJoints.size ();
         setJointCompliance (myJoints.get (end), compMagnitude);
         DoubleInterval range = new DoubleInterval ();
         switch (body.getName ()) {
            case "r shoulder":
               range.set (-90, 180);
               myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
               break;
            case "r elbow flex":
               range.set (0, 130);
               myJoints.get (end - 1).setCoordinateRangeDeg (0, range);
               break;
         }
      });
      return myJoints;
   }

   @SuppressWarnings("unchecked")
   private RenderableComponentList<JointBase> getJointsFromJointset (
      double compMagnitude) {
      myJoints = (RenderableComponentList<JointBase>)myMech.get ("jointset");
      DoubleInterval range = new DoubleInterval ();
      myJoints.forEach (jt -> {
         setJointCompliance (jt, compMagnitude);
         switch (jt.getName ()) {
            case "r_shoulder":
               range.set (-90, 180);
               jt.setCoordinateRangeDeg (0, range);
               break;
            case "r_elbow":
               range.set (0, 130);
               jt.setCoordinateRangeDeg (0, range);
               break;
         }
      });
      return myJoints;
   }

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

   @SuppressWarnings("unchecked")
   private RenderableComponentList<FrameMarker> getMarkerFromOsim () {
      myMarkers =
         (RenderableComponentList<FrameMarker>)myMech.get ("markerset");
      System.out.println ("Model markers: " + myMarkers.size ());
      return myMarkers;
   }

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

   private void importOsim () {
      readOsimFile ();
      myBodies = getBodiesFromOsim ();
      myJoints = getJointsFromOsim (myBodies);
      myMuscles = getMusclesFromOsim ();
      myMarkers = getMarkerFromOsim ();
      myMech.scaleDistance (myScale);
      myMech.scaleMass (myScale);
      myMech.updateWrapSegments ();
   }

   private void initializeIOProbes (String name, int scale) throws IOException {
      // Read all input data
      myMap = readMarkerFile (name);
      myMotion = readTRCFile (name, scale, myMap);
      myCoords = readCoordsFile (name);

      // Setup the tracking controller
      TrackingController controller =
         addControllerAndProps (myMotion, myMap, name);
      addExcitersToController (controller);
      //addPointTargetsAndProbes (controller, myMap, myMotion);
      addFrameTargetsAndProbes (controller, myMotion);

      // Parametric control
      //addCoordsInputProbes (myCoords);

      // Add output probes
      // TODO: Numeric Monitor Probes for later mesh evaluation (for cases,
      // where the data is not simply collected but generated by a function
      // within the probe itself
      addNumOutputProbesAndPanel (myMotion, controller);
      addBreakPoint (myMotion.getFrameTime (myMotion.numFrames () - 1));
   }

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

   private void readOsimFile () {
      String modelPath = myName + "/" + "arm26.osim";
      File osimFile = ArtisynthPath.getSrcRelativeFile (this, modelPath);
      String geometryPath = myName + "/Geometry/";
      File geometryFile = ArtisynthPath.getSrcRelativeFile (this, geometryPath);
      if (osimFile.exists () && geometryFile.exists ()) {
         OpenSimParser parser = new OpenSimParser (osimFile);
         parser.setGeometryPath (geometryFile);
         parser.createModel (myMech);
      }
   }

   private MarkerMotionData readTRCFile (
      String name, int scale, MarkerMapping map)
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

   private void setContactProps (CollisionManager coll) throws IOException {
      coll.setName ("Collision manager");
      // Handle overconstrained contact
      coll.setReduceConstraints (true);
      coll.setBilateralVertexContact (false);
      myJoints.forEach (jt -> {
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
}