package artisynth.models.diss;

import java.io.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import maspack.matrix.*;

/**
 * @author Alexander Denk Copyright (c) 2023, by the Author: Alexander Denk
 * (UDE) University of Duisburg-Essen Chair of Mechanics and Robotics
 * alexander.denk@uni-due.de
 */

public class MOTReader {

   protected InputStream myIstream;
   protected File myFile;
   protected ForceData myForces = new ForceData ();
   protected boolean inDegrees;

   public class ForceData {
      protected ArrayList<ArrayList<Vector3d>> myForceData = new ArrayList<> ();
      protected ArrayList<String> myForceLabels = new ArrayList<String> ();
      protected ArrayList<Double> myFrameTimes = new ArrayList<Double> ();
      protected int myColumns;

      public ForceData () {
      }

      public void addData (double time, Collection<Vector3d> forces) {
         if (numLabels () == 0) {
            throw new IllegalArgumentException ("No force labels defined.");
         }
         myFrameTimes.add (time);
         ArrayList<Vector3d> frcs = new ArrayList<Vector3d> ();
         forces.forEach (f -> {
            frcs.add (f);
         });
         myForceData.add (frcs);
      }

      public ArrayList<Vector3d> getData (int frame) {
         ArrayList<Vector3d> forces = myForceData.get (frame);
         return forces;
      }

      public Vector3d getData (int frame, int idx) {
         Vector3d force = myForceData.get (frame).get (idx);
         return force;
      }

      public Vector3d getData (int frame, String name)
         throws IllegalArgumentException {
         if (getForceLabels ().contains (name)) {
            int num = getForceLabels ().indexOf (name);
            Vector3d force = this.getData (frame, num);
            return force;
         }
         else {
            throw new IllegalArgumentException (name + " is unknown.");
         }
      }

      public ArrayList<String> getForceLabels () {
         return myForceLabels;
      }

      public int getFrame (double t1) {
         if (t1 > myFrameTimes.get (numFrames () - 1)) {
            throw new IllegalArgumentException ("Time t1 out of range.");
         }
         int frame = 0;
         for (Double time : myFrameTimes) {
            if (time >= t1) {
               frame = myFrameTimes.indexOf (time);
               break;
            }
         }
         return frame;
      }

      public double getFrameTime (int frame) {
         return myFrameTimes.get (frame);
      }

      public double getMaxForce (String label) {
         int num = getForceLabels ().indexOf (label);
         ArrayList<Vector3d> forces = new ArrayList<Vector3d> ();
         myFrameTimes.forEach (t -> {
            Vector3d force = this.getData (this.getFrame (t), num);
            forces.add (force);
         });
         double max = Double.MIN_VALUE;
         for (Vector3d f : forces) {
            if (f.maxElement () > max) {
               max = f.maxElement ();
            }
         }
         return max;
      }

      public double getMaxMoment (String label) {
         int num = getForceLabels ().indexOf (label);
         ArrayList<Vector3d> moments = new ArrayList<Vector3d> ();
         myFrameTimes.forEach (t -> {
            Vector3d moment = this.getData (this.getFrame (t), num);
            moments.add (moment);
         });
         double max = Double.MIN_VALUE;
         for (Vector3d m : moments) {
            if (m.maxElement () > max) {
               max = m.maxElement ();
            }
         }
         return max;
      }

      public int numColumns () {
         return myColumns;
      }

      public int numLabels () {
         return myForceLabels.size ();
      }

      public int numFrames () {
         return myFrameTimes.size ();
      }

      public void setColumns (int size) {
         myColumns = size;
      }

      public void setForceLabels (ArrayList<String> labels) {
         myForceLabels = labels;
      }
   }

   private void closeQuietly (InputStream in) {
      if (in != null) {
         try {
            in.close ();
         }
         catch (IOException e) {
         }
      }
   }

   public void close () {
      closeQuietly (myIstream);
   }

   public ForceData getForceData () {
      return myForces;
   }

   public int getNumFrames () {
      return myForces.numFrames ();
   }

   public int getNumLabels () {
      return myForces.numLabels ();
   }

   public boolean getInDegrees () {
      return inDegrees;
   }

   public int scanDataHeader (BufferedReader reader) throws IOException {
      int numLine = 0;
      int numFrames = 0;
      String line;

      while (numLine <= 6) {
         if ((line = reader.readLine ()) != null) {
            line = line.trim ();
            if (line.contains ("nRows")) {
               String[] tokens = line.split ("=");
               numFrames = Integer.parseInt (tokens[1]);
            }
            else if (line.contains ("nColumns")) {
               String[] tokens = line.split ("=");
               myForces.setColumns (Integer.parseInt (tokens[1]));
            }
            else if (line.contains ("Degrees")) {
               String[] tokens = line.split ("=");
               if (tokens[1].contains ("yes")) {
                  inDegrees = true;
               }
            }
            else if (line.contains ("time")) {
               String[] tokens = line.split ("\t");
               ArrayList<String> labels = new ArrayList<String> ();
               // Combine each column header triple (x,y,z) to a single force
               // label, skip time entry so start at 1.
               for (int i = 1; i < tokens.length; i++) {
                  // check if token belongs to the left foot
                  if (tokens[i].contains ("1")) {
                     // check if token is a force vector
                     if (tokens[i].contains ("v")) {
                        // check whether label is already contained
                        if (labels.contains ("Left GRF")) {
                           continue;
                        }
                        labels.add ("Left GRF");
                     }
                     // check if token is a point
                     else if (tokens[i].contains ("p")) {
                        if (labels.contains ("Left COP")) {
                           continue;
                        }
                        labels.add ("Left COP");
                     }
                     // check if token is a torque
                     else if (tokens[i].contains ("torque")) {
                        if (labels.contains ("Left GRM")) {
                           continue;
                        }
                        labels.add ("Left GRM");
                     }
                  }
                  // perform similar checks for the right foot
                  if (tokens[i].contains ("v")) {
                     if (labels.contains ("Right GRF")) {
                        continue;
                     }
                     labels.add ("Right GRF");
                  }
                  else if (tokens[i].contains ("p")) {
                     if (labels.contains ("Right COP")) {
                        continue;
                     }
                     labels.add ("Right COP");
                  }
                  else if (tokens[i].contains ("torque")) {
                     if (labels.contains ("Right GRM")) {
                        continue;
                     }
                     labels.add ("Right GRM");
                  }
               }
               myForces.setForceLabels (labels);
            }
         }
         numLine++;
      }
      return numFrames;
   }

   public void scanForceData (BufferedReader reader) throws IOException {
      int numLine = 7;
      String line;

      while ((line = reader.readLine ()) != null) {
         line = line.trim ();
         if (line.length () != 0) {
            String[] data = line.split ("\t");
            // Read time first
            double time = Double.parseDouble (data[0]);
            // Proof if each force label has an entry in the data list
            if (data.length != myForces.numColumns ()) {
               throw new IOException (
                  "Line " + numLine + ": detected " + data.length
                  + " fields; expected " + myForces.numColumns ());
            }
            // Read all data entries in that line into a force vector
            List<Vector3d> forces = new ArrayList<Vector3d> ();
            for (int i = 1; i < data.length; i++) {
               Vector3d force = new Vector3d ();
               force.x = Double.parseDouble (data[i]);
               force.y = Double.parseDouble (data[++i]);
               force.z = Double.parseDouble (data[++i]);
               forces.add (force);
            }
            myForces.addData (time, forces);
         }
         numLine++;
      }
   }

   public void readData () throws IOException {
      BufferedReader reader =
         new BufferedReader (new InputStreamReader (myIstream));
      // ReaderTokenizer rtok = new ReaderTokenizer (reader);
      // rtok.eolIsSignificant (true);
      scanDataHeader (reader);
      scanForceData (reader);
      reader.close ();
   }

   public MOTReader () {
   }

   public MOTReader (InputStream is) {
      myIstream = is;
   }

   public MOTReader (File file) throws IOException {
      this (new FileInputStream (file));
      myFile = file;
   }

   public static void main (String[] args) {

   }
}