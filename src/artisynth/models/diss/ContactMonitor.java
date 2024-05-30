package artisynth.models.diss;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import artisynth.core.materials.ContactForceBehavior;
import artisynth.core.mechmodels.Collidable;
import artisynth.core.mechmodels.CollisionBehavior;
import artisynth.core.mechmodels.CollisionBehaviorList;
import artisynth.core.mechmodels.CollisionResponse;
import artisynth.core.mechmodels.CollisionResponseList;
import artisynth.core.mechmodels.ContactData;
import artisynth.core.modelbase.ContactPoint;
import artisynth.core.modelbase.MonitorBase;
import artisynth.core.util.ArtisynthPath;
import artisynth.models.diss.MOTReader.ForceData;
import artisynth.models.diss.Tests.*;
import maspack.matrix.Vector3d;

/**
 * The contact monitor acts as a monitor (see artisynth manual for further
 * details), to supervise contact events during simulation. Contact is monitored
 * per joint with the contact parameters written to a *_message_file.txt in the
 * current working directory.
 * 
 * @author Alexander Denk Copyright (c) 2023, by the Author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */

public class ContactMonitor extends MonitorBase {
   // ----------------------------Instance Fields------------------------------
   // Name of the file, where the contact history is written to
   String msgName = null;
   // Path to the file, where the contact history is written to
   String msgPath = null;
   // Experimental data
   ForceData myForces;
   // Writer object, that writes data to file
   PrintWriter writer;
   // Writer variable, that tracks, whether the writer was active before
   boolean isActive;
   // List of all collision behaviors
   CollisionBehaviorList behaviors;
   // List of all available collision responses
   CollisionResponseList collisionsAll;
   // List of all converging collision responses
   CollisionResponseList collisionsActive = new CollisionResponseList ();
   // Map to match behaviors with responses
   HashMap<CollisionResponse,CollisionBehavior> collisionMap =
      new HashMap<CollisionResponse,CollisionBehavior> ();

   // ----------------------------Nested Classes ------------------------------

   public class CustomContactForce extends ContactForceBehavior {
      String myHostName;
      double myStiffness;
      double myDamping;
      List<CollisionBehavior> matches;
      HashMap<CollisionBehavior,Vector3d> myAccumulatedContact =
         new HashMap<CollisionBehavior,Vector3d> ();

      public CustomContactForce () {
         new CustomContactForce (0.1, 0.1);
      }

      public CustomContactForce (double comp, double damping) {
         this.myStiffness = 1 / comp;
         this.myDamping = damping;
         if (this.matches != null) {
            this.matches.clear ();
         }
      }

      @Override
      public void computeResponse (
         double[] fres, double dist, ContactPoint cpnt0, ContactPoint cpnt1,
         Vector3d normal, double contactArea, int flags) {

         fres[0] = dist * myStiffness; // contact force
         fres[1] = 1 / myStiffness; // compliance as inverse of stiffness
         fres[2] = myDamping; // damping

         CollisionBehavior host = (CollisionBehavior)this.myPropHost;
         matches =
            behaviors
               .stream ().filter (s -> s.equals (host))
               .collect (Collectors.toList ());

         assert matches.size () == 1 : "Error: ambiguous collision behaviors"
         + " for current response computation.";

         // host name wird immer null wenn neues contact interface
         if (myHostName == null && myAccumulatedContact.get (host) != null) {
            //Vector3d force = myAccumulatedContact.get (host);
            // which time take data from?
            //int frame = myForces.getFrame (0);
            // which side are we at?
            //String side = "";
            //Vector3d refForce = myForces.getData (frame, side + " GRF");

            //if (!force.epsilonEquals (refForce, 10)) {
               // update compliance accordingly.
               //this.myStiffness = myStiffness * 1.01;
               // all contact forces of all points need to be updated and summed
               // again
               // but distance is apparently the same for each contact interface
               // which would mean forces are equal and could be scaled
            //}

         }
         Vector3d force = new Vector3d (0, 0, 0);
         force.scaledAdd (fres[0], normal);
         myAccumulatedContact.merge (host, force, Vector3d::add);
         myHostName = host.getName ();
      }
   }

   // -----------------------------Constructors--------------------------------
   public ContactMonitor (String name) throws IOException {
      super ();
      initializwWriter (name);
   }

   // ----------------------------Instance Methods-----------------------------
   @Override
   public void initialize (double t0) {
      closeWriter ();
      addActiveResponses ();
      try {
         createCollisionMap ();
      }
      catch (Exception e) {
         e.printStackTrace ();
      }
      super.initialize (t0);
   }

   public void apply (double t0, double t1) {
      // Needs to be changed back to collisionsActive after testing
      collisionsAll.forEach (cr -> {
         if (cr.inContact ()) {

            // now unnecessary
            List<ContactData> cdata = cr.getContactData ();
            Vector3d force = calculateContactForces (cdata);
            int frame = myForces.getFrame (t1);

            // Which side are we at?
            String side;
            if (cr.getName ().contains ("_r")) {
               side = "Right";
            }
            else {
               side = "Left";
            }
            if (!force
               .epsilonEquals (myForces.getData (frame, side + " GRF"), 10)) {
               // percentage based tolerance
               CollisionBehavior behavior = collisionMap.get (cr);
               double comp = behavior.getCompliance () * 1.5; // What update?
               behavior.setCompliance (comp);

               // get new contact forces...
            }

         }
      });

      writeContactToFile (t0);
   }

   public void addForceData (ForceData forces) {
      this.myForces = forces;
   }

   public void addCollisionBehaviors (CollisionBehaviorList behaviors) {
      this.behaviors = behaviors;
   }

   public void addCollisionResponses (CollisionResponseList responses) {
      this.collisionsAll = responses;
   }

   // ------------------------Private Instance Methods-------------------------

   /**
    * Adds all ground related contact responses to the list of mutable contact
    * interfaces.
    */
   private void addActiveResponses () {
      // Search for all responses regarding toes and calcanei
      for (CollisionResponse cr : collisionsAll) {
         if (cr.getName ().contains ("ground")
         && cr.getName ().contains ("toes")) {
            collisionsActive.add (cr);
         }
         else if (cr.getName ().contains ("ground")
         && cr.getName ().contains ("calcn")) {
            collisionsActive.add (cr);
         }
      }
   }

   /**
    * Sums up all found contact forces in the contact data
    * 
    * @param cdata
    * {@link ContactData} list
    */
   private Vector3d calculateContactForces (List<ContactData> cdata) {
      Vector3d sum = new Vector3d (0, 0, 0);
      cdata.forEach (cd -> {
         sum.add (cd.getContactForce ());
      });
      return sum;
   }

   /**
    * Closes the PrintWriter if it was priorly active.
    */
   private void closeWriter () {
      if (isActive) {
         writer.close ();
      }
      isActive = true;
   }

   /**
    * Filters and matches the collision responses and behaviors, that are
    * referring to the same contact interface by comparing the collision
    * components of both objects.
    * 
    * @throws Exception
    * if no shared {@link Collidable}s are found
    * 
    */
   private void createCollisionMap () throws Exception {
      for (CollisionResponse response : collisionsAll) {
         Collidable bodyA = response.getCollidable (0);
         Collidable bodyB = response.getCollidable (1);

         List<CollisionBehavior> match =
            behaviors
               .stream ().filter (c -> c.getCollidable (0) == bodyA)
               .filter (c -> c.getCollidable (1) == bodyB)
               .collect (Collectors.toList ());

         if (match != null && match.size () == 1) {
            collisionMap.put (response, match.get (0));
         }
         else {
            throw new Exception (
               "Error: behavior and response collidables are inconsistent.");
         }
      }
   }

   /**
    * Writes the generated messages to file.
    * 
    * @param t0
    */
   private void writeContactToFile (double t0) {
      StringBuilder contactEvents = new StringBuilder ();
      contactEvents.append ("\nDETECT CONTACT EVENTS\n");

      collisionsAll.forEach (cr -> {

         contactEvents
            .append ("\nCOLLISION INTERFACE: ").append (cr.getName ())
            .append ("\n");

         if (cr.inContact ()) {
            List<ContactData> cdata = cr.getContactData ();
            if (cdata.size () > 0) {
               contactEvents
                  .append ("FOUND ").append (cdata.size ())
                  .append (" CONTACT EVENTS.\n");
               cdata.forEach (cd -> {
                  String row =
                     String
                        .format (
                           "%-20s%-6s", "POSITION",
                           cd.getPosition0 ().toString ("%.3f"));
                  contactEvents.append (row).append ("\n");
                  row =
                     String
                        .format (
                           "%-20s%-6s", "CONTACT FORCE (N)",
                           cd.getContactForce ().toString ("%.3f"));
                  contactEvents.append (row + "\n");
                  row =
                     String
                        .format (
                           "%-20s%-6s", "FRICTION FORCE (N)",
                           cd.getFrictionForce ().toString ("%.3f"));
                  contactEvents.append (row + "\n");
               });
            }
         }
         else {
            contactEvents.append ("NO CONTACT DETECTED." + "\n");
         }
      });
      writer.print (contactEvents.toString ());
      writer.flush ();
      contactEvents.delete (0, contactEvents.length ());
   }

   /**
    * Initializes PrintWriter from constructor.
    * 
    * @param name
    * Name specifier for the current working directory
    * @throws IOException
    */
   private void initializwWriter (String name) throws IOException {
      this.msgName = name + "/Output/" + name + "_message_file.txt";
      this.msgPath =
         ArtisynthPath
            .getSrcRelativePath (OpenSimTest.class, msgName).toString ();
      writer = new PrintWriter (new FileWriter (msgPath, true));
      this.isActive = false;
   }
}