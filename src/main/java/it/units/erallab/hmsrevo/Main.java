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
import it.units.erallab.hmsrobots.util.Util;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.core.Sequence;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.MutationOnly;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
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
import it.units.malelab.jgea.core.util.Pair;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.logging.Level;
import java.util.stream.Collectors;

import static it.units.malelab.jgea.core.util.Args.*;

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
    namedShapes.put("biped", createShape(new int[]{11, 4}, new int[]{2, 0, 9, 2}));
    namedShapes.put("tripod", createShape(new int[]{11, 4}, new int[]{2, 0, 5, 2}, new int[]{7, 0, 9, 2}));
    //read parameters
    int[] runs = ri(a("run", "0:10"));
    List<String> shapeNames = l(a("shape", "worm,biped,tripod,box10x10"));
    List<String> terrainNames = l(a("terrain", "flat,uneven5"));
    List<String> evolverNames = l(a("evolver", "mutationOnly,standard"));
    List<String> typeNames = l(a("type", "phases,phasesDevo,centralizedMLP"));
    double finalT = d(a("finalT", "30"));
    double minDT = d(a("minDT", "0.01"));
    double maxDT = d(a("maxDT", "0.2"));
    double drivingFrequency = d(a("drivingF", "-1"));
    List<Double> mutationSigmas = d(l(a("mutationSigma", "0.25")));
    List<Integer> controlStepIntervals = i(l(a("controlStepInterval", "1")));
    int nPop = i(a("npop", "100"));
    int iterations = i(a("iterations", "200"));
    List<Locomotion.Metric> metrics = Lists.newArrayList(Locomotion.Metric.TRAVEL_X_RELATIVE_VELOCITY);
    //prepare things
    MultiFileListenerFactory statsListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileStats", null));
    MultiFileListenerFactory serializedBestListenerFactory = new MultiFileListenerFactory(a("dir", "."), a("fileSerialized", null));
    //iterate   
    for (int run : runs) {
      for (String shapeName : shapeNames) {
        for (String terrainName : terrainNames) {
          for (String evolverName : evolverNames) {
            for (String typeName : typeNames) {
              for (int controlStepInterval : controlStepIntervals) {
                for (double mutationSigma : mutationSigmas) {
                  //build problem                
                  LocomotionProblem problem = new LocomotionProblem(
                          finalT, minDT, maxDT,
                          createTerrain(terrainName),
                          controlStepInterval,
                          metrics,
                          LocomotionProblem.ApproximationMethod.FINAL_T
                  );
                  //prepare robot related things
                  Grid<Boolean> shape = namedShapes.get(shapeName);
                  //prepare factory and mapper
                  Factory<Sequence<Double>> factory = null;
                  NonDeterministicFunction<Sequence<Double>, VoxelCompound.Description> mapper = null;
                  if (typeName.equals("phases") && (shape != null)) {
                    int voxels = (int) shape.values().stream().filter((b) -> b).count();
                    factory = new DoubleSequenceFactory(-Math.PI, Math.PI, voxels);
                    mapper = getPhaseSinMapper(shape, drivingFrequency);
                  } else if (typeName.equals("phasesDevo") && (shape != null)) {
                    int voxels = (int) shape.values().stream().filter((b) -> b).count();
                    factory = new DoubleSequenceFactory(-Math.PI, Math.PI, voxels * 3);
                    mapper = getPhaseSinWithDevoMapper(shape, drivingFrequency, finalT);
                  } else if (typeName.equals("centralizedMLP") && (shape != null)) {
                    int voxels = (int) shape.values().stream().filter((b) -> b).count();
                    List<Voxel.Sensor> sensors = Lists.newArrayList(Voxel.Sensor.AREA_RATIO, Voxel.Sensor.Y_ROT_VELOCITY);
                    int[] innerNeurons = new int[]{(int) Math.round(sensors.size() * voxels * 0.65d)};
                    int params = CentralizedMLP.countParams(
                            shape,
                            Grid.create(shape.getW(), shape.getH(), sensors),
                            innerNeurons
                    );
                    factory = new DoubleSequenceFactory(-1d, 1d, params);
                    mapper = getCentralizedMLPMapper(shape, sensors, drivingFrequency, innerNeurons);
                  } else if (typeName.equals("shapeMaterials") && shapeName.startsWith("box")) {
                    int nGaussians = 5;
                    int nMaterials = 4;
                    int params = nGaussians * nMaterials * 4;
                    factory = new DoubleSequenceFactory(0d, 1d, params);
                    mapper = getGaussianMixtureShapeMapper(
                            Integer.parseInt(shapeName.replace("box", "").split("x")[0]),
                            Integer.parseInt(shapeName.replace("box", "").split("x")[1]),
                            drivingFrequency, nGaussians, 1d
                    );
                  }
                  //prepare evolver
                  Evolver<Sequence<Double>, VoxelCompound.Description, List<Double>> evolver = null;
                  if (evolverName.equals("mutationOnly")) {
                    evolver = new MutationOnly<>(
                            nPop,
                            factory,
                            new ParetoRanker<>(false),
                            mapper,
                            new GaussianMutation(mutationSigma),
                            Lists.newArrayList(new Iterations(iterations)),
                            0,
                            false
                    );
                  } else if (evolverName.equals("standard")) {
                    Map<GeneticOperator<Sequence<Double>>, Double> operators = new LinkedHashMap<>();
                    operators.put(new SegmentCrossover(), 0.8d);
                    operators.put(new GaussianMutation(mutationSigma), 0.2d);
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
                  keys.put("control.step.interval", Integer.toString(controlStepInterval));
                  keys.put("type", typeName);
                  keys.put("run", Integer.toString(run));
                  keys.put("n.pop", Integer.toString(nPop));
                  keys.put("driving.frequency", Double.toString(drivingFrequency));
                  keys.put("mutation.sigma", Double.toString(mutationSigma));
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
                  }
                }
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
      Voxel.Builder builder = Voxel.Builder.create();
      return new VoxelCompound.Description(Grid.create(shape, b -> b ? builder : null), controller);
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
              weights,
              t -> Math.sin(2d * Math.PI * frequency * t)
      );
      Voxel.Builder builder = Voxel.Builder.create();
      return new VoxelCompound.Description(Grid.create(shape, b -> b ? builder : null), controller);
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
      Voxel.Builder builder = Voxel.Builder.create();
      return new VoxelCompound.Description(Grid.create(shape, b -> b ? builder : null), controller);
    };
  }

  private Function<Sequence<Double>, VoxelCompound.Description> getGaussianMixtureShapeMapper(final int w, final int h, final double frequency, final int nGaussian, final double threshold) {
    final Voxel.Builder hardBuilder = Voxel.Builder.create()
            .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
            .springF(40d)
            .ropeJointsFlag(false);
    final Voxel.Builder softBuilder = Voxel.Builder.create()
            .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
            .springF(10d)
            .ropeJointsFlag(false);
    final Voxel.Builder actuatedBuilder = Voxel.Builder.create()
            .springScaffoldings(EnumSet.of(Voxel.SpringScaffolding.CENTRAL_CROSS, Voxel.SpringScaffolding.SIDE_EXTERNAL))
            .ropeJointsFlag(false);
    final List<Pair<Voxel.Builder, SerializableFunction<Double, Double>>> materials = Lists.newArrayList(
            Pair.build(hardBuilder, t -> 0d),
            Pair.build(softBuilder, t -> 0d),
            Pair.build(actuatedBuilder, t -> Math.sin(2d * Math.PI * frequency * t)),
            Pair.build(actuatedBuilder, t -> Math.sin(-2d * Math.PI * frequency * t))
    );
    return (Sequence<Double> values, Listener listener) -> {
      //build grid of sum of gaussians for each material
      List<Grid<Double>> gaussianGrids = new ArrayList<>(materials.size());
      int c = 0;
      for (int i = 0; i < materials.size(); i++) {
        Grid<Double> gaussianGrid = Grid.create(w, h, 0d);
        gaussianGrids.add(gaussianGrid);
        for (int j = 0; j < nGaussian; j++) {
          //extract parameter of the j-th gaussian for the i-th material
          double muX = values.get(c + 0);
          double muY = values.get(c + 1);
          double sigma = Math.max(0d, values.get(c + 2));
          double weight = values.get(c + 3);
          c = c + 4;
          //compute over grid
          for (int x = 0; x < w; x++) {
            for (int y = 0; y < h; y++) {
              double d = d((double) x / (double) w, (double) y / (double) h, muX, muY);
              double g = gaussian(d, sigma) * weight;
              gaussianGrid.set(x, y, gaussianGrid.get(x, y) + g);
            }
          }
        }
      }
      //build grid with material index
      Grid<Integer> materialIndexGrid = Grid.create(w, h);
      for (int x = 0; x < w; x++) {
        for (int y = 0; y < h; y++) {
          Integer index = null;
          double max = Double.NEGATIVE_INFINITY;
          for (int i = 0; i < gaussianGrids.size(); i++) {
            double value = gaussianGrids.get(i).get(x, y);
            if ((value > threshold) && (value > max)) {
              index = i;
              max = value;
            }
          }
          materialIndexGrid.set(x, y, index);
        }
      }
      //find largest connected and crop
      materialIndexGrid = Util.gridLargestConnected(materialIndexGrid, i -> i != null);
      materialIndexGrid = Util.cropGrid(materialIndexGrid, i -> i != null);
      //build grids for description
      Grid<Voxel.Builder> builderGrid = Grid.create(materialIndexGrid);
      Grid<SerializableFunction<Double, Double>> functions = Grid.create(materialIndexGrid);
      for (Grid.Entry<Integer> entry : materialIndexGrid) {
        int x = entry.getX();
        int y = entry.getY();
        if (entry.getValue() != null) {
          builderGrid.set(x, y, materials.get(entry.getValue()).first());
          functions.set(x, y, materials.get(entry.getValue()).second());
        }
      }
      Controller controller = new TimeFunction(functions);
      return new VoxelCompound.Description(builderGrid, controller);
    };
  }

  private static double d(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow(x1 - x2, 2d) + Math.pow(y1 - y2, 2d));
  }

  private static double gaussian(double x, double sigma) {
    return 1d / sigma / Math.sqrt(2d * Math.PI) * Math.exp(-x * x / 2d / sigma / sigma);
  }

  private Grid<Boolean> createShape(int[] enclosing, int[]... holes) {
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
