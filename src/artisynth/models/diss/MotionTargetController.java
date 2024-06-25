package artisynth.models.diss;

import java.awt.Color;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.driver.Main;
import artisynth.core.inverse.ForceTargetTerm;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechSystemBase;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.Point;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.core.workspace.RootModel;
import artisynth.models.diss.ForceData;
import artisynth.models.diss.Tests.*;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;
import maspack.render.IsRenderable;
import maspack.render.RenderList;
import maspack.render.Renderer;

/**
 * The MotionTargetController is a controller (see artisynth manual for further
 * details) to handle the optimization problems of inverse simulation, by
 * defining motion and/or force targets. The controller also handles the
 * projection of COPs onto the model surface for force application. All
 * projection and inverse simulation details are written to a *_message_file.txt
 * in the current working directory.
 * <p>
 * Changelog: Search for comments with my name. Visibility of
 * getVelocityJacobian method in MotionTargetTerm from private to public. Added
 * getFrame method to MarkerMotionData. Rewrote an own isPointInside method
 * based on the isPointInside method of Face.
 * 
 * @author Alexander Denk Copyright (c) 2024, by the author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */
public class MotionTargetController extends TrackingController {
   // -------------------------------Static Fields------------------------------
   // Current System time during simulation
   static double[] mySystemTimes = new double[2];
   // ----------------------------Instance Fields------------------------------
   // Name of the file, where the inverse solver history is written to
   String msgName = null;
   // Path to the file, where the inverse solver history is written to
   String msgPath = null;
   // Message file string builder
   StringBuilder message = new StringBuilder ();
   // Writer variable, that tracks, whether the writer was active before
   boolean isWriterActive;
   // Experimental force data, necessary for the COP
   ForceData myForces;
   // Map, that matches the name of the model marker to the corresponding
   // experimental markers and their corresponding weights
   MarkerMapping myMap;
   // Experimental position data for all target points
   MarkerMotionData myMotion;
   // List of all source points, that are moved by the controller
   List<MotionTargetComponent> mySources;
   // List of all target points, that define the target position
   PointList<TargetPoint> myTargets;
   // Writer object, that writes data to file
   PrintWriter writer;
   // List of COP references, that serve as COP projection
   ArrayList<ModelComponent> copRefList = new ArrayList<ModelComponent> ();
   // List of NumericInputProbes, to be set (in-)active depending on COP
   ArrayList<NumericInputProbe> probeList = new ArrayList<NumericInputProbe> ();

   // ------------------------------Nested Classes------------------------------
   /**
    * Renders the experimental COP from ForceData.
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
         f1 = (int)(mySystemTimes[1] / (mySystemTimes[1] - mySystemTimes[0]));
         if (f1 > f0) {
            copPos[0] = myForces.getData (f1, "Right COP");
            copPos[1] = myForces.getData (f1, "Left COP");
            // right
            grfPos[0] =
               (Vector3d)((FrameMarker)copRefList.get (0)).getPosition ();
            Vector3d buf =
               myForces.getData (f1, "Right GRF").scale (0.001 * scale);
            grfPos[1] = grfPos[1].add (grfPos[0], buf);
            // left
            grfPos[2] =
               (Vector3d)((FrameMarker)copRefList.get (1)).getPosition ();
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
         if (probeList.get (0).isActive ()) {
            renderer.drawArrow (grfPos[0], grfPos[1], 0.01 * scale, false);
         }
         if (probeList.get (1).isActive ()) {
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
         this.copPos[0] = new Vector3d (0, 0, 0);
         this.copPos[1] = new Vector3d (0, 0, 0);
         this.grfPos[0] = new Vector3d (0, 0, 0);
         this.grfPos[1] = new Vector3d (0, 0, 0);
         this.grfPos[2] = new Vector3d (0, 0, 0);
         this.grfPos[3] = new Vector3d (0, 0, 0);
      }
   }

   // -------------------------------Constructors------------------------------
   public MotionTargetController (MechSystemBase myMech, String myName,
   String fileName) throws IOException {
      // Needs to be specifically written like this, because if done
      // otherwise, the tracking controller apply method cannot refer to the
      // myMech object and throws a NullPointerException
      super ();
      setMech (myMech);
      setName (myName);
      initializeWriter (fileName);
      //initializeCOPRenderer (scale);
      this.initComponents ();
   }

   // -----------------------------Static Methods------------------------------
   /**
    * Updates the current system times ({@code t1} and {@code t0}.
    * 
    * @param t0
    * @param t1
    */
   public static void updateSystemTime (double t0, double t1) {
      mySystemTimes[0] = t0;
      mySystemTimes[1] = t1;
   }

   // -------------------------------Instance Methods--------------------------
   @Override
   public void initialize (double t0) {
      super.initialize (t0);
      try {
         updateTargetPoints ();
      }
      catch (Exception e) {
         e.printStackTrace ();
      }
      if (isWriterActive) {
         writer.close ();
      }
      isWriterActive = true;
      clearMessageFile (msgPath);
      message.delete (0, message.length ());
      writeHeader ();
   }

   @Override
   public void apply (double t0, double t1) {
      updateSystemTime (t0, t1);
      // Provide message header
      StringBuilder header = new StringBuilder ();
      header
         .append ("\n").append ("------------------------- TIME\t").append (t1)
         .append (" --------------------------").append ("\n");
      message.append (header.toString ());
      message
         .append ("\n").append ("PROCEED WITH INVERSE OPTIMIZATION")
         .append ("\n");
      super.apply (t0, t1);
      writeMessageToFile ();
   }

   /**
    * Adds experimental motion data to this controller.
    * 
    * @param mot
    * MarkerMotionData
    * @param map
    * links the model marker names to the experimental marker names and their
    * weights
    */
   public void addMotionData (MarkerMotionData mot, MarkerMapping map) {
      this.myMotion = mot;
      this.myMap = map;
   }

   /**
    * Clears the content of the message file if it is not empty.
    * 
    * @param path
    * path to the message file
    */
   private void clearMessageFile (String path) {
      File file = new File (path);
      if (file.length () != 0) {
         try (PrintWriter pw = new PrintWriter (path)) {
         }
         catch (FileNotFoundException e) {
            e.printStackTrace ();
         }
      }
   }

   /**
    * Initializes PrintWriter from constructor.
    * 
    * @param fileName
    * Name specifier of the current working directory
    * @throws
    */
   private void initializeWriter (String fileName) throws IOException {
      this.msgName = fileName + "/Output/" + fileName + "_message_file.txt";
      this.msgPath =
         ArtisynthPath
            .getSrcRelativePath (OpenSimTest.class, msgName).toString ();
      FileWriter fw = new FileWriter (msgPath, true);
      this.writer = new PrintWriter (fw);
      this.isWriterActive = false;
   }

   /**
    * Updates all target point coordinates before the first frame.
    * 
    * @throws Exception
    */
   private void updateTargetPoints () throws Exception {
      myTargets = this.getTargetPoints ();
      mySources = this.getMotionSources ();
      if (myMap != null) {
         int end = myTargets.size ();
         for (int i = 0; i < end; i++) {
            String name =
               myMap.getExpLabelFromModel (mySources.get (i).getName ());
            Point3d position;
            position = (Point3d)myMotion.getMarkerPosition (0, name);
            myTargets.get (i).setPosition (position);
         }
      }
   }

   /**
    * Writes the message file header to file
    */
   private void writeHeader () {
      message
         .append (
            "%%----------------------- MESSAGE FILE -----------------------%%\n")
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
   }

   /**
    * Writes the generated messages to file.
    */
   private void writeMessageToFile () {
      writer.print (message.toString ());
      writer.flush ();
      message.delete (0, message.length ());
   }
}