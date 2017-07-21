package artisynth.tools.batchsim.manager;

import java.util.NoSuchElementException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import artisynth.tools.batchsim.manager.DistributionSampler.Distribution;
import static artisynth.tools.batchsim.manager.PropertySpecification.SpecificationType.*;
import maspack.properties.Property;
import maspack.util.IndentingPrintWriter;

/**
 * A property specification consists of a property path and a collection.
 * Property specifications can be either {@link SpecificationType#COMBINATORIAL
 * combinatorial} or {@link SpecificationType#PROBABILISTIC probabilistic}. The
 * type must be specified as an argument to the constructor, and cannot be
 * changed later, but can be queried via {@link #getSpecificationType()}. If a
 * property specification is combinatorial, then its collection is a set of
 * values, specified as strings, that the associated {@link Property} can be set
 * to. On the other hand, if a property specification is probabilistic, then its
 * collection is a list of integers representing {@link Distribution
 * distribution} identifiers that have been registered with a
 * {@link DistributionSampler}. Each distribution identifier acts as a handle to
 * a specific {@code Distribution}, and the probabilistic property
 * specification's list of distribution identifiers serves as a sort of handle
 * on the associated {@code Property}'s distribution vector. Refer to the Batch
 * Simulation Framework {@link artisynth.tools.batchsim documentation} for
 * details.
 * 
 * @author Francois Roewer-Despres
 * @version 1.0
 */
public class PropertySpecification implements Cloneable, Printable {

   /**
    * The type of a {@link PropertySpecification}, which can be either
    * {@link #COMBINATORIAL}, or {@link #PROBABILISTIC}.
    *
    * @author Francois Roewer-Despres
    * @version 1.0
    */
   public static enum SpecificationType {
      /**
       * A combinatorial property specification is one having a finite value
       * set.
       */
      COMBINATORIAL,
      /**
       * A probabilistic property specification is one having a distribution
       * vector.
       */
      PROBABILISTIC;
   }

   public static class DistributionPrinter {
      protected HashMap<String,Double> myPmf;
      protected Distribution myDistribution;
      protected List<Double> myParams;

      public DistributionPrinter (HashMap<String,Double> pmf) {
         myPmf = pmf;
      }

      public DistributionPrinter (Distribution dist, List<Double> params) {
         myDistribution = dist;
         myParams = params;
      }

      public void print (IndentingPrintWriter writer) {
         if (myPmf != null) {
            for (String value : myPmf.keySet ()) {
               if (value == null) {
                  writer.print ("prob(<null>");
               }
               else {
                  writer.print ("prob(" + value);
               }
               writer.println (") = " + myPmf.get (value));
            }
         }
         else {
            writer.print (myDistribution + "(");
            int counter = 0;
            for (Double param : myParams) {
               if (counter++ > 0) {
                  writer.print (", ");
               }
               writer.print (param);
            }
            writer.println (")");
         }
      }
   }

   protected static class Redef implements Printable {
      CombinationChecker checker;
      PropertySpecification propSpec;

      public Redef (CombinationChecker checker,
      PropertySpecification propSpec) {
         this.checker = checker;
         this.propSpec = propSpec;
      }

      @Override
      public void print (IndentingPrintWriter writer) {
         writer.println ("redefined as");
         writer.addIndentation (2);
         propSpec.print (writer, false);
         writer.removeIndentation (2);
         writer.println ("whenever");
         writer.addIndentation (2);
         checker.print (writer);
         writer.removeIndentation (2);
      }

      @Override
      public String toString () {
         return Utils.printableToString (this);
      }
   }

   protected static int indexCounter = 0;

   protected int myIndex;
   protected String myPropertyPath;
   protected List<Object> myValueSetOrDistIdList;
   protected List<DistributionPrinter> myDistributionPrinters;
   protected SpecificationType mySpecificationType;
   protected List<Redef> myRedefs;
   protected LinkedList<PropertySpecification> myRedefStack;

   /**
    * Creates a new property specification for the given property path, of the
    * given specification type, and with the given index.
    * 
    * @param propertyPath
    * the property path of this property specification
    * @param specificationType
    * the specification type of this property specification
    * @param index
    * the index of this property specification
    */
   public PropertySpecification (String propertyPath,
   SpecificationType specificationType, int index) {
      myIndex = index;
      myPropertyPath = propertyPath;
      myValueSetOrDistIdList = new LinkedList<> ();
      myDistributionPrinters = new LinkedList<> ();
      mySpecificationType = specificationType;
      myRedefs = new LinkedList<> ();
      myRedefStack = new LinkedList<> ();
   }

   /**
    * Creates a new property specification for the given property path, of the
    * given specification type, and with an automatic index.
    * 
    * @param propertyPath
    * the property path of this property specification
    * @param specificationType
    * the specification type of this property specification
    */
   public PropertySpecification (String propertyPath,
   SpecificationType specificationType) {
      this (propertyPath, specificationType, indexCounter++);
   }

   /**
    * Returns the index of this property specification.
    * 
    * @return the index of this property specification
    */
   public int getIndex () {
      return myIndex;
   }

   /**
    * Sets the index of this property specification to {@code index}.
    * 
    * @param index
    * the new index of this property specification
    */
   public void setIndex (int index) {
      myIndex = index;
   }

   /**
    * Returns the property path of this property specification.
    * 
    * @return the property path of this property specification
    */
   public String getPropertyPath () {
      return myPropertyPath;
   }

   /**
    * Sets the property path of this property specification to
    * {@code propertyPath}.
    * 
    * @param propertyPath
    * the new property path of this property specification
    */
   public void setPropertyPath (String propertyPath) {
      myPropertyPath = propertyPath;
   }

   /**
    * Returns the specification type of this property specification.
    * 
    * @return the specification type of this property specification
    */
   public SpecificationType getSpecificationType () {
      if (myRedefStack.isEmpty ()) {
         return mySpecificationType;
      }
      else {
         return myRedefStack.peek ().getSpecificationType ();
      }
   }

   public void add (String value) {
      if (myRedefStack.isEmpty ()) {
         doAdd (value, null);
      }
      else {
         myRedefStack.peek ().add (value);
      }
   }

   public void add (Integer distributionId, DistributionPrinter printer) {
      if (myRedefStack.isEmpty ()) {
         doAdd (distributionId, printer);
      }
      else {
         myRedefStack.peek ().add (distributionId, printer);
      }
   }

   /**
    * Adds either a {@link String} or an {@link Integer} to this property
    * specification's collection, depending on this property specification's
    * specification type.
    * 
    * @param valueOrDistributionId
    * the string or integer to add to this property specification's collection
    * @throws IllegalArgumentException
    * if {@code valueOrDistributionId} is not an instance of the correct type
    * given this property specification's specification type
    */
   protected void doAdd (
      Object valueOrDistributionId, DistributionPrinter printer)
      throws IllegalArgumentException {
      if (valueOrDistributionId instanceof String
      && getSpecificationType () != COMBINATORIAL) {
         throw new IllegalArgumentException (
            "Combinatorial property specifications can only add String values.");
      }
      if (valueOrDistributionId instanceof Integer
      && getSpecificationType () != PROBABILISTIC) {
         throw new IllegalArgumentException (
            "Probabilistic property specifications can only add Integer values.");
      }
      myValueSetOrDistIdList.add (valueOrDistributionId);
      if (printer != null) {
         myDistributionPrinters.add (printer);
      }
   }

   /**
    * Returns the size of this property specification's value set (if
    * <code>{@link #getSpecificationType()} ==
    * {@link SpecificationType#COMBINATORIAL}</code>) or {@link Distribution
    * distribution} identifier list (if <code>getSpecificationType() ==
    * {@link SpecificationType#PROBABILISTIC}</code>).
    * 
    * @return the size of this property specification's collection
    */
   public int size () {
      if (myRedefStack.isEmpty ()) {
         return myValueSetOrDistIdList.size ();
      }
      else {
         return myRedefStack.peek ().size ();
      }
   }

   /**
    * Returns the underlying value set (if
    * <code>{@link #getSpecificationType()} ==
    * {@link SpecificationType#COMBINATORIAL}</code>) or {@link Distribution
    * distribution} identifier list (if <code>getSpecificationType() ==
    * {@link SpecificationType#PROBABILISTIC}</code>) of this
    * {@link PropertySpecification}.
    *
    * @return the underlying value set or distribution identifier list of this
    * {@code PropertySpecification}
    */
   public List<Object> getCollection () {
      if (myRedefStack.isEmpty ()) {
         return myValueSetOrDistIdList;
      }
      else {
         return myRedefStack.peek ().getCollection ();
      }
   }

   protected List<DistributionPrinter> getDistributionPrinters () {
      if (myRedefStack.isEmpty ()) {
         return myDistributionPrinters;
      }
      else {
         return myRedefStack.peek ().getDistributionPrinters ();
      }
   }

   public void addRedef (
      PropertySpecification propSpec, CombinationChecker checker) {
      myRedefs.add (new Redef (checker, propSpec));
   }

   public boolean redefIfNecessary (List<String[]> task)
      throws RuntimeException {
      boolean found = false;
      for (Redef redef : myRedefs) {
         CombinationChecker checker = redef.checker;
         for (String[] propValPair : task) {
            checker.push (propValPair[0], propValPair[1]);
         }
         if (checker.check ()) {
            if (!found) {
               found = true;
               myRedefStack.push (redef.propSpec);
            }
            else {
               throw new RuntimeException (
                  "component \"" + myPropertyPath
                  + "\" has multiple matching redefinitions");
            }
         }
         checker.clear ();
      }
      return found;
   }

   public void undoRedef () throws NoSuchElementException {
      myRedefStack.pop ();
   }

   @Override
   public void print (IndentingPrintWriter writer) {
      print (writer, true);
   }

   public void print (IndentingPrintWriter writer, boolean topLevel) {
      if (topLevel) {
         writer.println ("defined");
         writer.addIndentation (2);
      }
      writer.print ("\"" + myPropertyPath);
      if (getCollection ().get (0) != null) {
         boolean combinatorial = getSpecificationType () == COMBINATORIAL;
         writer.println ((combinatorial ? "\" = {" : "\" ~ ["));
         writer.addIndentation (2);
         print (writer, combinatorial, 0);
         if (size () > 1) {
            print (writer, combinatorial, 1);
         }
         if (size () >= 3) {
            if (size () > 4) {
               writer.println ("... " + (size () - 3) + " others ...");
            }
            else if (size () == 4) {
               print (writer, combinatorial, 2);
            }
            print (writer, combinatorial, size () - 1);
         }
         writer.removeIndentation (2);
         writer.println ((combinatorial ? "}" : "]"));
      }
      else {
         writer.println ("\" = <null>");
      }
      if (topLevel) {
         writer.removeIndentation (2);
      }
      writer.addIndentation (2);
      for (Redef redef : myRedefs) {
         redef.print (writer);
      }
      writer.removeIndentation (2);
   }

   protected void print (
      IndentingPrintWriter writer, boolean combinatorial, int index) {
      if (combinatorial) {
         writer.println (getCollection ().get (index));
      }
      else {
         getDistributionPrinters ().get (index).print (writer);
      }
   }

   @Override
   public String toString () {
      return Utils.printableToString (this);
   }

   /**
    * {@inheritDoc}
    * 
    * Clones, but also increments the index. Does NOT copy any redefs.
    */
   @Override
   public PropertySpecification clone () {
      PropertySpecification other =
         new PropertySpecification (myPropertyPath, mySpecificationType);
      int i = 0;
      for (Object valueOrDistId : myValueSetOrDistIdList) {
         DistributionPrinter printer = null;
         if (mySpecificationType == PROBABILISTIC) {
            printer = myDistributionPrinters.get (i);
         }
         other.doAdd (valueOrDistId, printer);
         i++;
      }
      return other;
   }

}