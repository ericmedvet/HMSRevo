/*
 * Copyright (C) 2019 Eric Medvet <eric.medvet@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
package it.units.erallab.hmsrevo;

import com.google.common.collect.Lists;
import it.units.erallab.hmsrobots.controllers.CentralizedMLP;
import it.units.erallab.hmsrobots.controllers.Controller;
import it.units.erallab.hmsrobots.controllers.PhaseSin;
import it.units.erallab.hmsrobots.controllers.TimeFunction;
import it.units.erallab.hmsrobots.objects.Voxel;
import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializableFunction;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.core.Sequence;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.MutationOnly;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.ElapsedTime;
import it.units.malelab.jgea.core.evolver.stopcondition.Iterations;
import it.units.malelab.jgea.core.function.Function;
import it.units.malelab.jgea.core.function.NonDeterministicFunction;
import it.units.malelab.jgea.core.genotype.DoubleSequenceFactory;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.Basic;
import it.units.malelab.jgea.core.listener.collector.BestInfo;
import it.units.malelab.jgea.core.listener.collector.FunctionOfBest;
import it.units.malelab.jgea.core.listener.collector.Population;
import it.units.malelab.jgea.core.listener.collector.Static;
import it.units.malelab.jgea.core.operator.GaussianMutation;
import it.units.malelab.jgea.core.operator.GeneticOperator;
import it.units.malelab.jgea.core.operator.SegmentCrossover;
import it.units.malelab.jgea.core.ranker.ParetoRanker;
import it.units.malelab.jgea.core.ranker.selector.Tournament;
import it.units.malelab.jgea.core.ranker.selector.Worst;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.logging.Level;
import java.util.stream.Collectors;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class Main extends Worker {

  public Main(String[] args) throws FileNotFoundException {
    super(args);
  }

  public static void main(String[] args) throws FileNotFoundException {
    new Main(args);
  }

  @Override
  public void run() {
    //prepare shapes
    Map<String, Grid<Boolean>> namedShapes = new LinkedHashMap<>();
    namedShapes.put("worm", createShape(new int[]{11, 4}));
    namedShapes.put("biped", createShape(new int[]{11, 4}, new int[]{2, 2, 9, 4}));
    namedShapes.put("tripod", createShape(new int[]{11, 4}, new int[]{2, 2, 5, 4}, new int[]{7, 2, 9, 4}));
    //read parameters
    int[] runs = ri(a("runs", "0:10"));
    List<String> shapeNames = l(a("shapes", "worm,biped,tripod"));
    List<String> terrainNames = l(a("terrain", "flat,uneven5"));
    List<String> evolverNames = l(a("evolver", "mutationOnly,standard"));
    List<String> typeNames = l(a("type", "phases,phasesDevo,centralizedMLP"));
    double finalT = d(a("finalT", "30"));
    double minDT = d(a("minDT", "0.01"));
    double maxDT = d(a("maxDT", "0.2"));
    double drivingFrequency = d(a("drivingF", "-1"));
    int nPop = i(a("npop", "100"));
    int iterations = i(a("iterations", "200"));
    List<Locomotion.Metric> metrics = Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY);
    //prepare things
    MultiFileListenerFactory statsListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileStats", null));
    MultiFileListenerFactory serializedBestListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileSerialized", null));
    //iterate   
    for (int run : runs) {
      for (String shapeName : shapeNames) {
        for (String terrainName : terrainNames) {
          for (String evolverName : evolverNames) {
            for (String typeName : typeNames) {
              //build problem                
              LocomotionProblem problem = new LocomotionProblem(
                      finalT, minDT, maxDT,
                      createTerrain(terrainName), metrics,
                      LocomotionProblem.ApproximationMethod.FINAL_T
              );
              //prepare robot related things
              Grid<Boolean> shape = namedShapes.get(shapeName);
              //prepare factory and mapper
              Factory<Sequence<Double>> factory = null;
              NonDeterministicFunction<Sequence<Double>, VoxelCompound.Description> mapper = null;
              int voxels = (int) shape.values().stream().filter((b) -> b).count();
              if (typeName.equals("phases")) {
                factory = new DoubleSequenceFactory(-Math.PI, Math.PI, voxels);
                mapper = getPhaseSinMapper(shape, drivingFrequency);
              } else if (typeName.equals("phasesDevo")) {
                factory = new DoubleSequenceFactory(-Math.PI, Math.PI, voxels * 3);
                mapper = getPhaseSinWithDevoMapper(shape, drivingFrequency, finalT);
              } else if (typeName.equals("centralizedMLP")) {
                List<Voxel.Sensor> sensors = Lists.newArrayList(Voxel.Sensor.AREA_RATIO, Voxel.Sensor.Y_ROT_VELOCITY);
                int[] innerNeurons = new int[]{(int)Math.round(sensors.size() * voxels * 0.65d)};
                int params = CentralizedMLP.countParams(
                        shape,
                        Grid.create(shape.getW(), shape.getH(), sensors),
                        innerNeurons
                );
                factory = new DoubleSequenceFactory(-1d, 1d, params);
                mapper = getCentralizedMLPMapper(shape, sensors, drivingFrequency, innerNeurons);
              }
              //prepare evolver
              Evolver<Sequence<Double>, VoxelCompound.Description, List<Double>> evolver = null;
              if (evolverName.equals("mutationOnly")) {
                evolver = new MutationOnly<>(
                        nPop,
                        factory,
                        new ParetoRanker<>(false),
                        mapper,
                        new GaussianMutation(0.25),
                        Lists.newArrayList(new Iterations(iterations)),
                        0,
                        false
                );
              } else if (evolverName.equals("standard")) {
                Map<GeneticOperator<Sequence<Double>>, Double> operators = new LinkedHashMap<>();
                operators.put(new SegmentCrossover(), 0.8d);
                operators.put(new GaussianMutation(0.25d), 0.2d);
                evolver = new StandardEvolver<>(
                        nPop,
                        factory,
                        new ParetoRanker<>(false),
                        mapper,
                        operators,
                        new Tournament<>(3),
                        new Worst(),
                        nPop,
                        true,
                        Lists.newArrayList(new Iterations(iterations)),
                        0,
                        false
                );
              }
              //prepare keys
              Map<String, String> keys = new LinkedHashMap<>();
              keys.put("evolver", evolverName);
              keys.put("type", typeName);
              keys.put("run", Integer.toString(run));
              keys.put("n.pop", Integer.toString(nPop));
              keys.put("driving.frequency", Double.toString(drivingFrequency));
              keys.put("shape", shapeName);
              keys.put("terrain", terrainName);
              keys.put("metrics", metrics.stream().map((m) -> m.toString().toLowerCase().replace("_", ".")).collect(Collectors.joining("/")));
              System.out.println(keys);
              //run evolver
              Random r = new Random(run);
              try {
                evolver.solve(problem, r, executorService, Listener.onExecutor(statsListenerFactory.build(
                        new Static(keys),
                        new Basic(),
                        new Population(),
                        new BestInfo<>(problem.getFitnessFunction(metrics), "%+5.1f"),
                        FunctionOfBest.create(
                                "valid",
                                problem.getFitnessFunction(Lists.newArrayList(Locomotion.Metric.values())),
                                Arrays.stream(Locomotion.Metric.values()).map((m) -> {
                                  return m.toString().toLowerCase().replace('_', '.');
                                }).collect(Collectors.toList()),
                                Collections.nCopies(Locomotion.Metric.values().length, "%+5.1f"),
                                0
                        )
                ).then(serializedBestListenerFactory.build(
                        new Static(keys),
                        new Basic(),
                        new FunctionOfBest<>("serialized", (VoxelCompound.Description vcd, Listener l) -> {
                          return Util.lazilySerialize(vcd);
                        }, 0, "%s")
                )), executorService));
              } catch (InterruptedException | ExecutionException ex) {
                L.log(Level.SEVERE, String.format("Cannot solve problem: %s", ex), ex);
                
                ex.printStackTrace();
                
              }
            }
          }
        }
      }
    }
  }

  private Function<Sequence<Double>, VoxelCompound.Description> getPhaseSinMapper(final Grid<Boolean> shape, final double frequency) {
    return (Sequence<Double> values, Listener listener) -> {
      int c = 0;
      Grid<Double> phases = Grid.create(shape);
      for (Grid.Entry<Boolean> entry : shape) {
        if (entry.getValue()) {
          phases.set(entry.getX(), entry.getY(), values.get(c));
          c = c + 1;
        }
      }
      Controller controller = new PhaseSin(frequency, 1d, phases);
      Voxel.Builder builder = Voxel.Builder.create()
              .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
              .ropeJointsFlag(false);
      return new VoxelCompound.Description(shape, controller, Grid.create(shape.getW(), shape.getH(), builder));
    };
  }

  private Function<Sequence<Double>, VoxelCompound.Description> getCentralizedMLPMapper(final Grid<Boolean> shape, final List<Voxel.Sensor> sensors, final double frequency, final int[] innerNeurons) {
    return (Sequence<Double> values, Listener listener) -> {
      double[] weights = new double[values.size()];
      for (int i = 0; i < values.size(); i++) {
        weights[i] = values.get(i);
      }
      Controller controller = new CentralizedMLP(
              shape,
              Grid.create(shape.getW(), shape.getH(), sensors),
              innerNeurons,
              null,
              t -> Math.sin(2d * Math.PI * frequency * t)
      );
      Voxel.Builder builder = Voxel.Builder.create()
              .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
              .ropeJointsFlag(false);
      return new VoxelCompound.Description(shape, controller, Grid.create(shape.getW(), shape.getH(), builder));
    };
  }

  private Function<Sequence<Double>, VoxelCompound.Description> getPhaseSinWithDevoMapper(final Grid<Boolean> shape, final double frequency, final double devoInterval) {
    return (Sequence<Double> values, Listener listener) -> {
      int c = 0;
      Grid<SerializableFunction<Double, Double>> functions = Grid.create(shape);
      for (Grid.Entry<Boolean> entry : shape) {
        if (entry.getValue()) {
          double phase = values.get(c);
          double initD = values.get(c + 1);
          double finalD = values.get(c + 2);
          functions.set(entry.getX(), entry.getY(),
                  t -> Math.sin(2d * Math.PI * frequency * t + phase) + initD + (t / devoInterval) * (finalD - initD)
          );
          c = c + 3;
        }
      }
      Controller controller = new TimeFunction(functions);
      Voxel.Builder builder = Voxel.Builder.create()
              .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
              .ropeJointsFlag(false);
      return new VoxelCompound.Description(shape, controller, Grid.create(shape.getW(), shape.getH(), builder));
    };
  }

  private Grid<Boolean> createShape(int[] enclosing, int[]  
    ... holes) {
    Grid<Boolean> shape = Grid.create(enclosing[0], enclosing[1], true);
    for (int[] hole : holes) {
      for (int x = hole[0]; x < hole[2]; x++) {
        for (int y = hole[1]; y < hole[3]; y++) {
          shape.set(x, y, false);
        }
      }
    }
    return shape;
  }

  private static double[][] randomTerrain(int n, double length, double peak, double borderHeight, Random random) {
    double[] xs = new double[n + 2];
    double[] ys = new double[n + 2];
    xs[0] = 0d;
    xs[n + 1] = length;
    ys[0] = borderHeight;
    ys[n + 1] = borderHeight;
    for (int i = 1; i < n + 2; i++) {
      xs[i] = 1 + (double) (i - 1) * (length - 2d) / (double) n;
      ys[i] = random.nextDouble() * peak;
    }
    return new double[][]{xs, ys};
  }

  public static double[][] createTerrain(String name) {
    Random random = new Random(1);
    if (name.equals("flat")) {
      return new double[][]{new double[]{0, 1, 1999, 2000}, new double[]{30, 0, 0, 30}};
    } else if (name.startsWith("uneven")) {
      int h = Integer.parseInt(name.replace("uneven", ""));
      return randomTerrain(20, 2000, h, 30, random);
    }
    return null;
  }

}
