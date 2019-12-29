package at.ac.tuwien.big.momot.search.algorithm;

import at.ac.tuwien.big.momot.problem.solution.TransformationSolution;
import at.ac.tuwien.big.momot.problem.solution.variable.ITransformationVariable;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.moeaframework.algorithm.AbstractAlgorithm;
import org.moeaframework.core.Initialization;
import org.moeaframework.core.NondominatedPopulation;
import org.moeaframework.core.NondominatedSortingPopulation;
import org.moeaframework.core.PRNG;
import org.moeaframework.core.Problem;
import org.moeaframework.core.Solution;
import org.moeaframework.core.Variation;
import org.moeaframework.core.comparator.DominanceComparator;

import experiment.MySearchContext;

/**
 *
 * My Implementation of multi-objectives paticle swarm optimizer for TransformationSolution.
 *
 */
public class MyMOPSO extends AbstractAlgorithm {

   /**
    * The internal class to store the swap operator.
    */
   protected class Speed {
      private final List<SwapOperator> swapList;

      public Speed() {
         swapList = new ArrayList<>();
      }

      public SwapOperator get(final int index) {
         return this.swapList.get(index);
      }

      public List<SwapOperator> getSwapList() {
         return this.swapList;
      }

      public void remove(final SwapOperator SO) {

      }

      public void ShuffleList() {
         PRNG.shuffle(this.swapList);
      }
   }

   /**
    * The internal class for describing the velocity.
    */
   protected class SwapOperator {
      private final int idNum;
      private final ITransformationVariable ruleApp;
      private final double probability;

      public SwapOperator(final int id, final ITransformationVariable ruleApplication, final double probability) {
         this.idNum = id;
         this.ruleApp = ruleApplication;
         this.probability = probability;
      }

      public int getIdNum() {
         return idNum;
      }

      public double getProbability() {
         return probability;
      }

      public ITransformationVariable getRuleApp() {
         return ruleApp;
      }
   }

   /**
    * The number of particles.
    */
   protected int swarmSize;

   /**
    * The number of leaders.
    */
   protected int leaderSize;

   /**
    * The particles.
    */
   protected TransformationSolution[] particles;

   /**
    * The local best particles.
    */
   protected TransformationSolution[] localBestParticles;

   /**
    * The leaders.
    */
   protected NondominatedSortingPopulation leaders;

   /**
    * The archive of non-dominated solutions; or {@code null} of no external
    * archive is sued.
    */
   protected NondominatedPopulation archive;

   /**
    * The speed / velocity of each particle.
    */
   protected Speed[] velocities;

   /**
    * Comparator for selecting leaders.
    */
   protected DominanceComparator leaderComparator;

   /**
    * Comparator for leaders truncate.
    */
   protected DominanceComparator truncateComparator;

   /**
    * Comparator for updating the local best particles.
    */
   protected DominanceComparator dominanceComparator;

   /**
    * Mutation operator, or {@code null} if no mutation is defined.
    */
   protected Variation mutation;

   /**
    * The probability for the execution of the swap operators from particle and Pbest.
    */
   protected double alpha = 0.5;

   /**
    * The probability for the execution of the swap operators from particle and Gbest.
    */
   protected double beta = 0.1;

   /**
    * The probability for the interia.
    */
   protected double gamma = 0.6;

   /**
    * The initialization operator.
    */
   protected final Initialization initilization;

   // just for debug.
   protected int Numloop;

   /**
    * Constructs a new abstract PSO algorithm.
    *
    * @param problem
    *           the problem
    * @param swarmSize
    *           the number of particles
    * @param leaderSize
    *           the number of leaders
    * @param leaderComparator
    *           comparator for selecting leaders
    * @param dominanceComparator
    *           comparator for updating the local best
    *           particles
    * @param leaders
    *           non-dominated population for storing the leaders
    * @param archive
    *           non-dominated population for storing the external archive;
    *           or {@code null} if no external archive is defined
    * @param mutation
    *           mutation operator, or {@code null} if no mutation is
    *           defined
    */
   public MyMOPSO(final Problem problem, final int swarmSize, final int leaderSize,
         final DominanceComparator leaderComparator, final DominanceComparator dominanceComparator,
         final NondominatedPopulation archive, final Variation mutation, final Initialization initilization) {
      super(problem);
      this.swarmSize = swarmSize;
      this.leaderSize = leaderSize;
      this.leaderComparator = leaderComparator;
      this.dominanceComparator = dominanceComparator;
      this.archive = archive;
      this.mutation = mutation;
      this.initilization = initilization;

      this.particles = new TransformationSolution[swarmSize];
      this.localBestParticles = new TransformationSolution[swarmSize];
      this.leaders = new NondominatedSortingPopulation();
      this.velocities = new Speed[swarmSize];
      this.Numloop = 0;
   }

   @Override
   public NondominatedPopulation getResult() {
      if(archive == null) {
         return new NondominatedPopulation(leaders);
      } else {
         return new NondominatedPopulation(archive);
      }
   }

   @Override
   protected void initialize() {
      super.initialize();

      final Solution[] initialParticles = initilization.initialize();

      evaluateAll(initialParticles);

      for(int i = 0; i < swarmSize; i++) {
         particles[i] = (TransformationSolution) initialParticles[i];
         localBestParticles[i] = (TransformationSolution) initialParticles[i];

         velocities[i] = new Speed();
      }

      leaders.addAll(initialParticles);
      // leaders.truncate(leaderSize, new CrowdingComparator());
      // leaders.truncate(leaderSize, new TarComparator(new ParetoDominanceComparator(), new CrowdingComparator()));
      leaders.truncate(leaderSize);

      if(archive != null) {
         archive.addAll(initialParticles);
      }
      // final NondominatedPopulation particle_archive = new NondominatedPopulation();
      // particle_archive.addAll(particles);
      final String res = MySearchContext.evaluate(archive);
      try {
         MySearchContext.fw.write(res + "\n");
      } catch(final IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   @Override
   protected void iterate() {

      updateVelocities();
      updatePositions();
      // mutate();

      // this.Numloop += 1;
      // if(this.Numloop == 60) {
      // System.out.println("It is debug time!");
      // }

      evaluateAll(particles);

      updateLocalBest();
      leaders.addAll(particles);
      // leaders.truncate(leaderSize, new CrowdingComparator());
      // leaders.truncate(leaderSize, new TarComparator(new ParetoDominanceComparator(), new CrowdingComparator()));
      leaders.truncate(leaderSize);

      if(archive != null) {
         archive.clear();
         archive.addAll(particles);
      }
      // final NondominatedPopulation particle_archive = new NondominatedPopulation();
      // particle_archive.addAll(particles);
      final String res = MySearchContext.evaluate(leaders);
      try {
         MySearchContext.fw.write(res + "\n");
      } catch(final IOException e) {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }

      gamma *= 0.97;
   }

   /**
    * Applies the mutation operator to all particles.
    */
   protected void mutate() {
      for(int i = 0; i < swarmSize; i++) {
         mutate(i);
      }
   }

   /**
    * Applies the mutation operator to an individual particle.
    *
    * @param i
    *           the index of the particle
    */
   protected void mutate(final int i) {
      if(mutation != null) {
         particles[i] = (TransformationSolution) mutation.evolve(new Solution[] { particles[i] })[0];
      }
   }

   /**
    * Randomly select a leader.
    *
    * @return the selected leader
    */
   protected TransformationSolution selectLeader() {
      final TransformationSolution leader1 = (TransformationSolution) leaders.get(PRNG.nextInt(leaders.size()));
      final TransformationSolution leader2 = (TransformationSolution) leaders.get(PRNG.nextInt(leaders.size()));
      final int flag = leaderComparator.compare(leader1, leader2);

      // final TransformationSolution leader1 = (TransformationSolution) leaders.get(0);
      // final TransformationSolution leader2 = (TransformationSolution) leaders.get(1);
      // final int flag = dominanceComparator.compare(leader1, leader2);

      if(flag < 0) {
         return leader1;
      } else if(flag > 0) {
         return leader2;
      } else if(PRNG.nextBoolean()) {
         return leader1;
      } else {
         return leader2;
      }
   }

   /**
    * Updates the local best particles.
    */
   protected void updateLocalBest() {
      for(int i = 0; i < swarmSize; i++) {
         final int flag = dominanceComparator.compare(particles[i], localBestParticles[i]);

         if(flag <= 0) {
            localBestParticles[i] = particles[i];
         }
      }
   }

   /**
    * Update the position of an individual particle.
    *
    * @param i
    *           the index of the particle
    */
   protected void updatePosition(final int i) {
      final TransformationSolution parent = particles[i];
      TransformationSolution child = parent.copy();
      final Speed velocity = velocities[i];
      velocity.ShuffleList();

      if(PRNG.nextDouble() < gamma) {
         final int times = PRNG.nextInt(3) + 1;
         for(int j = 0; j < times; j++) {
            if(mutation != null) {
               child = (TransformationSolution) mutation.evolve(new Solution[] { child })[0];
            }
         }
      } else {
         final int SOLength = velocity.getSwapList().size();
         for(int j = 0; j < SOLength; j++) {
            if(PRNG.nextDouble() < velocity.get(j).getProbability()) {
               child.setVariable(velocity.get(j).getIdNum(), velocity.get(j).getRuleApp());
            }
         }
      }

      particles[i] = child;
   }

   /**
    * Update the positions of all particles.
    */
   protected void updatePositions() {
      for(int i = 0; i < swarmSize; i++) {
         updatePosition(i);
      }
   }

   /**
    * Update the speeds of all particles.
    */
   protected void updateVelocities() {
      for(int i = 0; i < swarmSize; i++) {
         updateVelocity(i);
      }
   }

   /**
    * Update the speed of an individual particle.
    *
    * @param i
    *           the index of the particle
    */
   protected void updateVelocity(final int i) {
      final TransformationSolution particle = particles[i];
      final TransformationSolution localBestParticle = localBestParticles[i];
      final TransformationSolution leader = selectLeader();

      final int variablesLength = particle.getVariables().length;

      // empty the list
      velocities[i].getSwapList().clear();

      // delete some SOs with a probability named gamma
      final int swapListLength = velocities[i].getSwapList().size();
      for(int j = swapListLength - 1; j >= 0; j--) {
         if(PRNG.nextDouble() < gamma) {
            velocities[i].getSwapList().remove(j);
         }
      }

      for(int j = 0; j < variablesLength; j++) {
         if(particle.getVariable(j).compareTo(localBestParticle.getVariable(j)) != 0) {
            final SwapOperator SO = new SwapOperator(j, localBestParticle.getVariable(j).copy(), alpha);
            velocities[i].getSwapList().add(SO);
         }
      }
      for(int j = 0; j < variablesLength; j++) {
         if(particle.getVariable(j).compareTo(leader.getVariable(j)) != 0) {
            final SwapOperator SO = new SwapOperator(j, leader.getVariable(j).copy(), beta);
            velocities[i].getSwapList().add(SO);
         }
      }
   }

}
