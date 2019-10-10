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

import it.units.erallab.hmsrobots.Starter;
import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.viewers.VideoFileWriter;
import it.units.malelab.jgea.Worker;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class VideoBuilder extends Worker {

  public VideoBuilder(String[] args) throws FileNotFoundException {
    super(args);
  }

  public static void main(String[] args) throws FileNotFoundException {
    new VideoBuilder(args);
  }

  @Override
  public void run() {
    //get params
    double finalT = d(a("finalT", "3"));
    double minDT = d(a("minDT", "0.05"));
    //obtain controllers
    final Map<String, String> namedSerializedVcd = new LinkedHashMap<>();
    namedSerializedVcd.put("one", "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");
    namedSerializedVcd.put("two", "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");
    //namedSerializedVcd.put("three", "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");
    //prepare grid
    int gridW = i(a("gridW", "3"));
    int gridH = (int) Math.ceil((float) namedSerializedVcd.size() / (float) gridW);
    Grid<String> namesGrid = Grid.create(gridW, gridH);
    int c = 0;
    for (String name : namedSerializedVcd.keySet()) {
      namesGrid.set(c % gridW, Math.floorDiv(c, gridW), name);
      c = c + 1;
    }
    //prepare video file writer
    File file = new File(a("dir", "."), a("file", "video.mp4"));
    VideoFileWriter videoFileWriter = null;
    try {
      videoFileWriter = new VideoFileWriter(file, namesGrid, executorService);
    } catch (IOException ex) {
      L.severe(String.format("Cannot create file: %s", ex));
      System.exit(0);
    }
    final VideoFileWriter writer = videoFileWriter;
    //iterate
    List<Future<String>> futures = new ArrayList<>();
    for (final Grid.Entry<String> entry : namesGrid) {
      if (entry.getValue() != null) {
        final LocomotionProblem locomotionProblem = new LocomotionProblem(
                finalT, minDT, minDT,
                Main.createTerrain("uneven10"),
                Locomotion.Metric.values(), LocomotionProblem.ApproximationMethod.FINAL_T
        );
        futures.add(executorService.submit(() -> {
          VoxelCompound.Description vcd = Util.deserialize(namedSerializedVcd.get(entry.getValue()), true);
          locomotionProblem.getFitnessFunction().apply(
                  vcd, null, new BridgeListener(writer.listener(entry.getX(), entry.getY()))
          );
          return String.format("%s at (%d,%d)", entry.getValue(), entry.getX(), entry.getY());
        }));
      }
    }
    //wait for end
    for (Future<String> future : futures) {
      try {
        System.out.printf("Done %s%n", future.get());
      } catch (InterruptedException | ExecutionException ex) {
        L.severe(String.format("Exception: %s", ex));
      }
    }
    try {
      writer.flush();
    } catch (IOException ex) {
      L.severe(String.format("Cannot flush: %s", ex));
    }
    executorService.shutdown();
  }

}
