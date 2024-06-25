package artisynth.models.diss;

import java.io.*;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

import maspack.matrix.*;

/**
 * @author Alexander Denk Copyright (c) 2023-2024 <br>
 * (UDE) University of Duisburg-Essen <br>
 * Chair of Mechanics and Robotics <br>
 * alexander.denk@uni-due.de
 */
public class MOTReader {
   // ----------------------------Instance Fields------------------------------
   protected InputStream myIstream;
   protected File myFile;
   protected ForceData myForces = new ForceData ();
   protected CoordinateData myCoords = new CoordinateData ();
   protected boolean coordsInDegrees;
   protected boolean forcesInDegrees;

   // ------------------------------Constructors-------------------------------
   public MOTReader () {
   }

   public MOTReader (InputStream is) {
      myIstream = is;
   }

   public MOTReader (File file) throws IOException {
      this (new FileInputStream (file));
      myFile = file;
   }

   // --------------------------Static Methods---------------------------------
   public static void main (String[] args) {

   }

   // -------------------------Instance Methods--------------------------------
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

   public CoordinateData getCoordinateData () {
      return myCoords;
   }

   public ForceData getForceData () {
      return myForces;
   }

   public int getNumCoordFrames () {
      return myCoords.numFrames ();
   }

   public int getNumForceFrames () {
      return myForces.numFrames ();
   }

   public int getNumCoordLabels () {
      return myCoords.numLabels ();
   }

   public int getNumForceLabels () {
      return myForces.numLabels ();
   }

   public boolean getCoordsInDegrees () {
      return coordsInDegrees;
   }

   public boolean getForcesInDegrees () {
      return forcesInDegrees;
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
                  forcesInDegrees = true;
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

   public void scanMOTData (BufferedReader reader) throws IOException {
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
      scanDataHeader (reader);
      scanMOTData (reader);
      reader.close ();
   }
}