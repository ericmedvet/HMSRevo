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
import it.units.erallab.hmsrobots.controllers.Controller;
import it.units.erallab.hmsrobots.controllers.PhaseSin;
import it.units.erallab.hmsrobots.objects.Voxel;
import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.malelab.jgea.SurrogatePrecisionControl;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.PrecisionController;
import it.units.malelab.jgea.core.Sequence;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.MutationOnly;
import it.units.malelab.jgea.core.evolver.stopcondition.ElapsedTime;
import it.units.malelab.jgea.core.function.Function;
import it.units.malelab.jgea.core.genotype.DoubleSequenceFactory;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.MultiFileListenerFactory;
import it.units.malelab.jgea.core.listener.collector.Basic;
import it.units.malelab.jgea.core.listener.collector.BestInfo;
import it.units.malelab.jgea.core.listener.collector.FunctionOfBest;
import it.units.malelab.jgea.core.listener.collector.FunctionOfEvent;
import it.units.malelab.jgea.core.listener.collector.Population;
import it.units.malelab.jgea.core.listener.collector.Static;
import it.units.malelab.jgea.core.operator.GaussianMutation;
import it.units.malelab.jgea.core.ranker.ParetoRanker;
import it.units.malelab.jgea.core.util.Pair;
import it.units.malelab.jgea.problem.surrogate.ControlledPrecisionProblem;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Collections;
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
    Random random = new Random(1);
    //prepare shapes
    Map<String, Grid<Boolean>> namedShapes = new LinkedHashMap<>();
    namedShapes.put("worm", createShape(new int[]{11, 4}));
    namedShapes.put("biped", createShape(new int[]{11, 4}, new int[]{2, 2, 9, 4}));
    namedShapes.put("tripod", createShape(new int[]{11, 4}, new int[]{2, 2, 5, 4}, new int[]{7, 2, 9, 4}));
    //prepare episodes
    Map<String, double[][]> namedTerrainProfiles = new LinkedHashMap<>();
    namedTerrainProfiles.put("flat", new double[][]{new double[]{0, 1, 1999, 2000}, new double[]{30, 0, 0, 30}});
    namedTerrainProfiles.put("uneven10", randomTerrain(20, 2000, 10, 30, random));
    //read parameters
    int[] runs = ri(a("runs", "0:10"));
    List<String> shapeNames = l(a("shapes", "worm,biped,tripod"));
    List<String> terrainNames = l(a("terrains", "flat"));
    List<String> pControllerNames = l(a("pController", "static-0,static-0.5,linear-0.5-0-1.0"));
    double finalT = d(a("finalT", "30"));
    double minDT = d(a("minDT", "0.01"));
    double maxDT = d(a("maxDT", "0.2"));
    double drivingFrequency = d(a("drivingF", "-1"));
    int nPop = i(a("npop", "100"));
    double maxElapsed = d(a("maxElapsed", "60"));
    Locomotion.Metric[] metrics = new Locomotion.Metric[]{Locomotion.Metric.TRAVEL_X_VELOCITY};
    //prepare things
    MultiFileListenerFactory listenerFactory = new MultiFileListenerFactory(a("dir", "."), a("file", null));
    //iterate   
    for (int run : runs) {
      for (String shapeName : shapeNames) {
        for (String terrainName : terrainNames) {
          for (String pControllerName : pControllerNames) {
            //build problem
            LocomotionProblem problem = new LocomotionProblem(
                    finalT, minDT, maxDT,
                    namedTerrainProfiles.get(terrainName), metrics,
                    LocomotionProblem.ApproximationMethod.FINAL_T
            );
            PrecisionController<VoxelCompound.Description> pController = null;
            if (pControllerName.startsWith("static")) {
              pController = new SurrogatePrecisionControl.StaticController<>(
                      d(p(pControllerName, 1))
              );
            } else if (pControllerName.startsWith("linear")) {
              pController = new SurrogatePrecisionControl.LinearController<>(
                      d(p(pControllerName, 1)),
                      d(p(pControllerName, 2)),
                      (int) Math.round((double) 2000 / d(p(pControllerName, 3)))
              );
            }
            ControlledPrecisionProblem<VoxelCompound.Description, List<Double>> cpProblem = new ControlledPrecisionProblem<>(
                    problem,
                    pController
            );
            //prepare robot related things
            Grid<Boolean> shape = namedShapes.get(shapeName);
            int voxels = (int) shape.values().stream().filter((b) -> b).count();
            //prepare evolver
            Evolver<Sequence<Double>, VoxelCompound.Description, List<Double>> evolver = new MutationOnly<>(
                    nPop,
                    new DoubleSequenceFactory(-Math.PI, Math.PI, voxels),
                    new ParetoRanker<>(false),
                    getPhaseSinMapper(shape, drivingFrequency),
                    new GaussianMutation(0.25),
                    Lists.newArrayList(new ElapsedTime(maxElapsed, TimeUnit.MINUTES)),
                    0,
                    false
            );
            //prepare keys
            Map<String, String> keys = new LinkedHashMap<>();
            keys.put("run", Integer.toString(run));
            keys.put("n.pop", Integer.toString(nPop));
            keys.put("driving.frequency", Double.toString(drivingFrequency));
            keys.put("shape", shapeName);
            keys.put("terrain", terrainName);
            keys.put("precision.controller", pControllerName);
            keys.put("metrics", Arrays.stream(metrics).map((m) -> m.toString().toLowerCase().replace("_", ".")).collect(Collectors.joining("/")));
            System.out.println(keys);
            //run evolver
            Random r = new Random(run);
            try {
              evolver.solve(cpProblem, r, executorService, Listener.onExecutor(listenerFactory.build(
                      new Static(keys),
                      new Basic(),
                      new Population(),
                      new BestInfo<>(problem.getFitnessFunction(metrics), "%+5.1f"),
                      new FunctionOfEvent<>("sum.precisions", (e, l) -> cpProblem.getController().getSumOfPrecisions(), "%8.4f"),
                      new FunctionOfEvent<>("calls", (e, l) -> cpProblem.getController().getCalls(), "%7d"),
                      new FunctionOfEvent<>("history.avg.precision", (e, l) -> cpProblem.getController().getHistory().stream().mapToDouble((o) -> ((Double) ((Pair) o).second()).doubleValue()).average().orElse(Double.NaN), "%5.3f"),
                      FunctionOfBest.create(
                              "valid",
                              problem.getFitnessFunction(Locomotion.Metric.values()),
                              Arrays.stream(Locomotion.Metric.values()).map((m) -> {
                                return m.toString().toLowerCase().replace('_', '.');
                              }).collect(Collectors.toList()),
                              Collections.nCopies(Locomotion.Metric.values().length, "%+5.1f"),
                              0
                      ),
                      new FunctionOfBest<>("serialized", (VoxelCompound.Description vcd, Listener l) -> {
                        return Util.lazilySerialize(vcd);
                      }, 0, "%s")
              ), executorService));
            } catch (InterruptedException | ExecutionException ex) {
              L.log(Level.SEVERE, String.format("Cannot solve problem: %s", ex), ex);
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
      return new VoxelCompound.Description(shape, controller, Voxel.Builder.create());
    };
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

  private double[][] randomTerrain(int n, double length, double peak, double borderHeight, Random random) {
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

}
