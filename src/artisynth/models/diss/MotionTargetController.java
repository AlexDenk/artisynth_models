package artisynth.models.diss;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;

import artisynth.core.inverse.ForceTargetTerm;
import artisynth.core.inverse.TargetPoint;
import artisynth.core.inverse.TrackingController;
import artisynth.core.mechmodels.Frame;
import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.MechSystemBase;
import artisynth.core.mechmodels.MotionTargetComponent;
import artisynth.core.mechmodels.PointList;
import artisynth.core.mechmodels.RigidBody;
import artisynth.core.modelbase.ModelComponent;
import artisynth.core.probes.MarkerMotionData;
import artisynth.core.probes.NumericInputProbe;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.diss.MOTReader.ForceData;
import artisynth.models.diss.Tests.*;

import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.geometry.Vertex3d;
import maspack.matrix.Point3d;
import maspack.matrix.Vector3d;

/**
 * The MotionTargetController is a controller (see artisynth manual for further
 * details) to handle the optimization problems of inverse simulation, by
 * defining motion and/or force targets. The controller also handles the
 * projection of COPs onto the model surface for force application. All
 * projection and inverse simulation details are written to a *_message_file.txt
 * in the current working directory.
 *  <p>
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
   // ----------------------------Instance Fields------------------------------
   // Name of the file, where the inverse solver history is written to
   String msgName = null;
   // Path to the file, where the inverse solver history is written to
   String msgPath = null;
   // Message file string builder
   StringBuilder message = new StringBuilder ();
   // Writer variable, that tracks, whether the writer was active before
   boolean isWriterActive;
   // Check if force targets are used by this controller
   boolean hasForceTargets = false;
   // Experimental force data, necessary for the COP
   ForceData myForces;
   // Map, that matches the name of the model marker to the corresponding
   // experimental markers and their corresponding weightings
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
   // ------------------------------------Nested Classes-----------------------

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
      this.initComponents ();
   }

   // -------------------------------Instance Methods--------------------------
   @Override
   public void initialize (double t0) {
      super.initialize (t0);
      // Initialize target coords from experimental data
      try {
         updateTargetPoints ();
      }
      catch (Exception e) {
         e.printStackTrace ();
      }
      // Disable the ForceTargetTerm per default if available
      ForceTargetTerm forceTerm = getForceTargetTerm ();
      if (forceTerm != null) {
         forceTerm.setEnabled (false);
         hasForceTargets = true;
      }
      // Assert that the sizes of InputProbes and COP ref points are consistent
      assert probeList.size () == copRefList
         .size () : "Error: Dimension mismatch between probes and COPs";

      // Ensure that the writer is closed if it was previously active
      if (isWriterActive) {
         writer.close ();
      }
      isWriterActive = true;
      // Clear message file, if it contains text from earlier runs
      clearMessageFile (msgPath);
      message.delete (0, message.length ());
      writeHeader ();
   }

   @Override
   public void apply (double t0, double t1) {
      // Update system time in the model for COP rendering
      OpenSimTest.updateSystemTime (t1);
      // Provide message header
      String newLine = System.lineSeparator ();
      StringBuilder header = new StringBuilder ();
      header
         .append (newLine).append ("------------------------- TIME\t")
         .append (t1).append (" --------------------------").append (newLine)
         .append (newLine).append ("PROJECT COPs FOR FORCE APPLICATION")
         .append (newLine);
      message.append (header.toString ());
      // Collect cop positions based on the specified force tolerance
      ArrayList<Point3d> cops = getCurrentCOPs (t1, 3.0, probeList);
      // Make sure there are not more cops than ref markers
      assert cops.size () <= copRefList
         .size () : "Error: Dimension mismatch between listed and found COPs";
      // Find nearest body to the found COPs
      for (int i = 0; i < cops.size (); i++) {
         // Query the closest frame relative to COP
         Frame frame = findClosestBody (t1, cops.get (i));
         // Take one of the cop references from the list and adjust it
         FrameMarker copRef = (FrameMarker)copRefList.get (i);
         // project COP coordinates to frame surface
         try {
            // projectToFrameVertex (frame, copRef, cops.get (i), 0.005);
            projectToFrameSurface (frame, copRef, cops.get (i));
         }
         catch (Exception e) {
            e.printStackTrace ();
         }
      }
      message
         .append (newLine).append ("PROCEED WITH INVERSE OPTIMIZATION")
         .append (newLine);
      super.apply (t0, t1);
      writeMessageToFile ();
   }

   /**
    * Adds the model component {@code comp} to the list of cop references.
    * 
    * @param comp
    * Model component
    */
   public void addCOPReference (ModelComponent comp) {
      if (comp.getProperty ("externalForce") != null) {
         this.copRefList.add (comp);
      }
   }

   /**
    * Adds experimental force data to this controller.
    * 
    * @param frcs
    * ForceData
    */
   public void addForceData (ForceData frcs) {
      this.myForces = frcs;
   }

   /**
    * Adds a {@link NumericInputProbe} to the controller.
    * 
    * @param probe
    * NumericInputProbe
    */
   public void addInputProbe (NumericInputProbe probe) {
      this.probeList.add (probe);

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
    * Queries of the current controller makes use of force targets.
    * 
    * @return
    */
   public boolean hasForceTargets () {
      return hasForceTargets;
   }

   /**
    * Adds COP based on the specified force application site.
    * 
    * @param side
    * @param frame
    * @param tol
    * @param cops
    * @param probes
    */
   private void addCOPIfForcePresent (
      String side, int frame, Double tol, ArrayList<Point3d> cops,
      ArrayList<NumericInputProbe> probes) {
      Vector3d force = myForces.getData (frame, side + " GRF");
      if (!force.epsilonEquals (new Vector3d (0, 0, 0), tol)) {
         Vector3d buf = myForces.getData (frame, side + " COP");
         cops.add (new Point3d (buf));
         probeList.forEach (p -> {
            if (p.getName ().contains (side.toLowerCase ())) {
               p.setActive (true);
            }
         });
         message
            .append (
               String
                  .format (
                     "FOUND %s COP AT\t%.3f, %.3f, %.3f%n", side.toUpperCase (),
                     buf.x, buf.y, buf.z));
      }
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
    * Finds the closest point from a list to a reference point. If the list is
    * empty
    * 
    * @param cop
    * @param candidates
    * @return
    */
   private Point3d findClosestPoint (
      Point3d cop, ArrayList<Point3d> candidates) {
      Point3d clst = new Point3d (0, 0, 0);
      double minDist = Double.MAX_VALUE;
      for (Point3d pnt : candidates) {
         double dist = pnt.distance (cop);
         if (dist < minDist) {
            minDist = dist;
            clst = pnt;
            message
               .append (pnt.toString ("%.3f")).append (System.lineSeparator ());
         }
      }
      return clst;
   }

   /**
    * Finds the closest body relative to {@code cop} by comparing the distances
    * of {@code cop} to the model marker positions during {@code t1}. It then
    * finds the attached body to the closest experimental marker.
    * 
    * @param t1
    * current simulation time
    * @param cop
    * {@link Point3d} COP coordinates
    * @return closest body frame related to the COP position
    */
   private Frame findClosestBody (Double t1, Point3d cop) {
      double minDist = Double.MAX_VALUE;
      FrameMarker nearest = null;
      // Distance calculation and comparison
      for (MotionTargetComponent src : mySources) {
         FrameMarker marker = (FrameMarker)src;
         Point3d pos = marker.getPosition ();
         Vector3d distVec = new Vector3d (0, 0, 0);
         double dist = distVec.sub (cop, pos).norm ();
         if (dist < minDist) {
            minDist = dist;
            nearest = marker;
         }
      }
      // Get attachment of nearest frame marker
      if (nearest != null) {
         Frame body = (Frame)nearest.getFrame ();
         message
            .append (System.lineSeparator ()).append ("FOUND CLOSEST BODY FOR ")
            .append (cop.toString ("%.3f")).append (":\t")
            .append (body.getName ()).append (System.lineSeparator ());

         return body;
      }
      return null;
   }

   /**
    * Retrieves the current Centers of Pressure (COPs) based on the forces
    * applied at simulation time {@code t1}. It considers forces active if they
    * exceed the specified tolerance {@code tol}. Active COPs are then added to
    * the list, and corresponding input probes are set to active.
    * 
    * @param t1
    * current simulation time
    * @param tol
    * tolerance value in Newtons (N)
    * @param probes
    * list of NumericInputProbes
    * @return
    */
   private ArrayList<Point3d> getCurrentCOPs (
      Double t1, Double tol, ArrayList<NumericInputProbe> probes) {
      ArrayList<Point3d> cops = new ArrayList<Point3d> ();
      int frame = myForces.getFrame (t1);
      // Perform force tolerance check for the right
      addCOPIfForcePresent ("Right", frame, tol, cops, probes);
      // Perform force tolerance check for the left
      addCOPIfForcePresent ("Left", frame, tol, cops, probes);

      return cops;
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
    * Determines if a point is inside a triangle based on the sum of the areas
    * of sub-triangles formed with the triangle's vertices. This method
    * calculates the areas using cross products to establish if the point falls
    * within the triangle.
    *
    * @param f
    * the triangle face represented by {@link Face}
    * @param sec,
    * {@link Point3d} to test for containment within the triangle
    * @param ref,
    * {@link Point3d} reference used for translating triangle vertices to world
    * coordinates.
    * @return true if the point is inside the triangle; false otherwise
    */
   private boolean isPointInside (Face f, Point3d sec, Point3d ref) {
      // Get all vertices in world coordinates
      Point3d x0 = f.getPoint (0);
      Point3d x0W = new Point3d (0, 0, 0);
      x0W.add (x0, ref);
      Point3d x1 = f.getPoint (1);
      Point3d x1W = new Point3d (0, 0, 0);
      x1W.add (x1, ref);
      Point3d x2 = f.getPoint (2);
      Point3d x2W = new Point3d (0, 0, 0);
      x2W.add (x2, ref);
      // Generate vectors from point sec to each vertex of the triangle
      // signs don't matter, since magnitude is calculated for area
      Point3d vec0 = (Point3d)x0W.sub (sec);
      Point3d vec1 = (Point3d)x1W.sub (sec);
      Point3d vec2 = (Point3d)x2W.sub (sec);
      // Calculate the areas of sub-triangles using cross products
      Point3d c0 = new Point3d (0, 0, 0);
      double area0 = c0.cross (vec0, vec1).norm () / 2;
      Point3d c1 = new Point3d (0, 0, 0);
      double area1 = c1.cross (vec1, vec2).norm () / 2;
      Point3d c2 = new Point3d (0, 0, 0);
      double area2 = c2.cross (vec0, vec2).norm () / 2;
      // Compare calculated sum of all sub areas to the area of f
      double area = f.getPlanarArea ();
      double sum = area0 + area1 + area2;
      sum = sum - area;
      // Point is inside if the area difference is within tolerance.
      final double insideTriangleTolerance = 1e-13;
      return Math.abs (sum) < insideTriangleTolerance;
   }

   /**
    * Projects the coordinates of {@code cop} to closest vertex on the surface
    * of {@code frame}, if possible. Sets the coords of {@code copRef}
    * accordingly.
    * 
    * @param frame
    * {@link RigidBody} to project onto
    * @param copRef
    * {@link FrameMarker} being projected
    * @param point3d
    * {@link Point3d} coords projected from
    * @param tol
    * search tolerance value
    * @throws Exception
    */
   private void projectToFrameVertex (
      Frame frame, FrameMarker copRef, Point3d cop, Double tol)
      throws Exception {
      if (frame instanceof RigidBody) {
         PolygonalMesh mesh = ((RigidBody)frame).getCollisionMesh ();
         // Verify that the cop is not inside of the body
         assert mesh
            .pointIsInside (
               cop) != 1 : "Error projecting COP: COP inside of frame surface";
         // Search for all vertices within tolerance to COP
         ArrayList<Vertex3d> vertices = mesh.getVertices ();
         ArrayList<Point3d> points = new ArrayList<Point3d> ();
         ArrayList<Integer> indexes = new ArrayList<Integer> ();
         message
            .append (
               System.lineSeparator () + "ATTEMPT TO FIND VERTICES ON "
               + frame.getName () + " WITHIN " + tol + " m TOLERANCE."
               + System.lineSeparator ());
         vertices.forEach (v -> {
            // Use world point instead of position for actual position
            // in world coords
            Point3d pos = v.getWorldPoint ();
            if (pos.x >= cop.x - tol && pos.x <= cop.x + tol) {
               if (pos.z >= cop.z - tol && pos.z <= cop.z + tol) {
                  points.add (pos);
                  indexes.add (v.getIndex ());
               }
            }
         });
         // Initialize dummy points
         Point3d clst;
         Point3d ref = new Point3d (frame.getPosition ());
         // Proceed with no points found
         if (points.size () == 0) {
            clst = new Point3d (0, 0, 0);
            mesh.distanceToPoint (clst, cop);
            message
               .append (
                  "WARNING: NO VERTEX FOUND WITHIN RANGE."
                  + System.lineSeparator ());
            message
               .append (
                  "TAKE NEAREST VERTEX INSTEAD: " + clst.toString ("%.3f")
                  + System.lineSeparator ());
            clst.sub ((Vector3d)ref);
         }
         // Proceed with points found
         else {
            message
               .append (
                  "FOUND " + points.size () + " VERTICES WITHIN RANGE."
                  + System.lineSeparator ());
            // Search for the closest member in points
            ArrayList<Double> distances = new ArrayList<Double> ();
            points.forEach (p -> {
               Vector3d dist = new Vector3d ();
               dist.x = p.x - cop.x;
               dist.y = p.y - cop.y;
               dist.z = p.z - cop.z;
               distances.add (dist.norm ());
               message.append (p.toString ("%.3f") + System.lineSeparator ());
            });
            // find the index of min distance within the stream object of
            // distances
            int idx =
               distances
                  .indexOf (distances.stream ().min (Double::compare).get ());
            // get the position of the clostest vertex
            clst = mesh.getVertex (indexes.get (idx)).getWorldPoint ();
            message.append ("FOUND CLOSEST VERTEX FOR PROJECTION: ");
            message.append (clst.toString ("%.3f") + System.lineSeparator ());
            // Calculate location vector
            clst.sub ((Vector3d)ref);
         }
         // Attach on of the cop ref markers to the frame
         copRef.setFrame (frame);
         // Set ref pos and location of the frame marker to match clst
         copRef.setRefPos (ref);
         copRef.setLocation (clst);
      }
      else {
         throw new Exception (
            "Error projecting COP: SurfaceMesh not available.");
      }
   }

   /**
    * Projects the coordinates of {@code cop} onto the surface of {@code frame},
    * if possible, and updates {@code copRef} accordingly.
    * 
    * @param frame
    * {@link RigidBody} to project onto
    * @param copRef
    * {@link FrameMarker} being projected
    * @param cop
    * {@link Point3d} position to project from
    * @throws Exception
    * if the frame is not a RigidBody or no projection is possible
    */
   private void projectToFrameSurface (
      Frame frame, FrameMarker copRef, Point3d cop)
      throws Exception {
      if (!(frame instanceof RigidBody)) {
         throw new Exception (
            "Error projecting COP: SurfaceMesh not available.");
      }
      PolygonalMesh mesh = ((RigidBody)frame).getCollisionMesh ();
      // Define projection direction
      Vector3d projection = new Vector3d (0, 1, 0);
      // Query reference frame position for world coordinates
      Point3d ref = new Point3d (frame.getPosition ());
      ArrayList<Point3d> candidates = new ArrayList<Point3d> ();
      message
         .append (System.lineSeparator ())
         .append ("ATTEMPT TO FIND INTERSECTIONS ON ").append (frame.getName ())
         .append (System.lineSeparator ());
      for (Face f : mesh.getFaces ()) {
         Vector3d norm = f.getNormal ();
         // Check for planes nearly parallel to the projection vector
         if (Math.abs (projection.dot (norm)) < 1e-6) {
            continue;
         }
         // Query point v_1 on plane (in local coords)
         Point3d v1Local = f.getPoint (0);
         // world coords
         Point3d v1World = new Point3d (0, 0, 0);
         v1World.add (v1Local, ref);
         // project distance between cop and v_1 on projection vector
         Point3d pnt = new Point3d (0, 0, 0);
         Double t = pnt.sub (v1World, cop).dot (norm) / projection.dot (norm);
         // Calculate intersection point
         Point3d intersection = new Point3d (cop.x, cop.y, cop.z);
         intersection.scaledAdd (t, projection);
         // Check if the point is within the face
         if (isPointInside (f, intersection, ref)) {
            candidates.add (intersection);
         }
      }
      Point3d clst = findClosestPoint (cop, candidates);
      if (clst == null) {
         clst = new Point3d (0, 0, 0);
         mesh.distanceToPoint (clst, cop);
         clst.sub ((Vector3d)ref);
         message
            .append ("WARNING: NO INTERSECTION FOUND.")
            .append (System.lineSeparator ())
            .append ("TAKE NEAREST VERTEX INSTEAD: ")
            .append (clst.toString ("%.3f")).append (System.lineSeparator ());
      }
      else {
         message
            .append ("FOUND CLOSEST INTERSECTION FOR PROJECTION: ")
            .append (clst.toString ("%.3f")).append (System.lineSeparator ());
      }

      // Update the FrameMarker to the calculated location
      copRef.setFrame (frame);
      copRef.setRefPos (ref);
      copRef.setLocation (clst);
      message
         .append ("PROJECTED COP TO: ")
         .append (copRef.getPosition ().toString ("%.3f"))
         .append (System.lineSeparator ());
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
            // Get the name of each experimental marker from the name
            // of the corresponding model marker (source)
            String name =
               myMap.getExpLabelFromModel (mySources.get (i).getName ());
            // String name = myMap.get (mySources.get (i).getName ()).getKey ();
            // Set the position of the corresponding target point
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