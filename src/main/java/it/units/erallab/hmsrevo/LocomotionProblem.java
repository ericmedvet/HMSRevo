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

import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.malelab.jgea.core.function.Function;
import it.units.malelab.jgea.core.function.NonDeterministicBiFunction;
import it.units.malelab.jgea.core.function.NonDeterministicFunction;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.problem.surrogate.TunablePrecisionProblem;
import java.util.List;
import java.util.Random;
import org.dyn4j.dynamics.Settings;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class LocomotionProblem implements TunablePrecisionProblem<VoxelCompound.Description, List<Double>> {

  public static enum ApproximationMethod {
    FINAL_T, DT
  };

  private final double maxFinalT;
  private final double minDT;
  private final double maxDT;
  private final double[][] groundProfile;
  private final int controlStepInterval;
  private final List<Locomotion.Metric> metrics;
  private final ApproximationMethod approximationMethod;

  public LocomotionProblem(double maxFinalT, double minDT, double maxDT, double[][] groundProfile, int controlStepInterval, List<Locomotion.Metric> metrics, ApproximationMethod approximationMethod) {
    this.maxFinalT = maxFinalT;
    this.minDT = minDT;
    this.maxDT = maxDT;
    this.groundProfile = groundProfile;
    this.controlStepInterval = controlStepInterval;
    this.metrics = metrics;
    this.approximationMethod = approximationMethod;
  }

  @Override
  public NonDeterministicBiFunction<VoxelCompound.Description, Double, List<Double>> getTunablePrecisionFitnessFunction() {
    return getTunablePrecisionFitnessFunction(metrics);
  }

  @Override
  public NonDeterministicFunction<VoxelCompound.Description, List<Double>> getFitnessFunction() {        
    return getFitnessFunction(metrics);
  }

  private NonDeterministicBiFunction<VoxelCompound.Description, Double, List<Double>> getTunablePrecisionFitnessFunction(List<Locomotion.Metric> localMetrics) {
    return (VoxelCompound.Description vcd, Double p, Random random, Listener listener) -> {
      double dT = minDT;
      double finalT = maxFinalT;
      if (approximationMethod.equals(ApproximationMethod.FINAL_T)) {
        finalT = maxFinalT * (1d - p);
      } else {
        dT = minDT + p * (maxDT - minDT);
      }
      Settings settings = new Settings();
      settings.setStepFrequency(dT);
      Locomotion locomotion = new Locomotion(finalT, groundProfile, localMetrics, controlStepInterval, settings);
      List<Double> metricValues = locomotion.apply(vcd);
      for (int i = 0; i<metricValues.size(); i++) {
        metricValues.set(i, metricValues.get(i)*(localMetrics.get(i).isToMinimize()?1d:(-1d)));
      }
      return metricValues;
    };
  }
  
  public Function<VoxelCompound.Description, List<Double>> getFitnessFunction(List<Locomotion.Metric> localMetrics) {
    NonDeterministicBiFunction<VoxelCompound.Description, Double, List<Double>> f = getTunablePrecisionFitnessFunction(localMetrics);
    Random random = new Random(1);
    return (VoxelCompound.Description vcd, Listener listener) -> {
      return f.apply(vcd, 0d, random, listener);
    };    
  }

}
