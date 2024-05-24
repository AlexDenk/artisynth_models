package artisynth.models.diss.Tests;

import java.util.ArrayList;

import artisynth.core.mechmodels.FrameMarker;
import artisynth.core.mechmodels.RigidBody;
import maspack.geometry.Face;
import maspack.geometry.PolygonalMesh;
import maspack.matrix.Point3d;
import maspack.matrix.RigidTransform3d;
import maspack.util.UnitTest;

public class MotionTargetControllerTest extends UnitTest {

   public void testSetFrame () {
   }

   public void testIsPointInside () {
      RigidBody box = RigidBody.createBox ("box", 1, 1, 1, 100);
      Point3d ref = new Point3d (box.getPosition ());
      PolygonalMesh mesh = box.getCollisionMesh ();
      Face f = mesh.getFace (0);

      ArrayList<Point3d> points = new ArrayList<Point3d> ();
      points.add (new Point3d (0, 0, -0.5));
      points.add (new Point3d (1, 0, -0.5));
      points.add (new Point3d (0, 1, -0.5));
      points.add (new Point3d (1, 1, -0.5));
      points.add (new Point3d (0.5, 0.5, -0.5));
      points.add (new Point3d (0.5, -0.5, -0.5));
      points.add (new Point3d (-0.5, 0, -0.5));
      points.add (new Point3d (0, 0.5, -0.5));
      points.add (new Point3d (10, 10, -0.5));
      points.add (new Point3d (100, 0, -0.5));
      points.add (new Point3d (0.3, 0.3, -0.5));
      points.add (new Point3d (0.3, -0.3, -0.5));
      points.add (new Point3d (0, -0.000000000001, -0.5));
      points.add (new Point3d (0, -0.0000000000001, -0.5));
      points.add (new Point3d (0, -0.00000000000001, -0.5));

      double insideTriangleTolerance = 1e-13;

      ArrayList<Integer> expected = new ArrayList<Integer> ();
      expected.add (0);
      expected.add (1);
      expected.add (1);
      expected.add (1);
      expected.add (0);
      expected.add (1);
      expected.add (0);
      expected.add (0);
      expected.add (1);
      expected.add (1);
      expected.add (0);
      expected.add (1);
      expected.add (1);
      expected.add (0);
      expected.add (0);

      ArrayList<Integer> results = new ArrayList<Integer> ();
      for (Point3d sec : points) {
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

         if (Math.abs (sum) < insideTriangleTolerance) {
            results.add (0);
         }
         else {
            results.add (1);
         }
         checkEquals ("Point inside", results.get (0), expected.get (0));
      }
   }

   // add other tests you want to be performed

   public void test () {
      testSetFrame ();
      testIsPointInside ();
   }

   public static void main (String[] args) {
      MotionTargetControllerTest tester = new MotionTargetControllerTest ();
      tester.runtest ();
   }
}