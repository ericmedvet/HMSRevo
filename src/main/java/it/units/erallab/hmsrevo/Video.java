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
import it.units.erallab.hmsrobots.objects.VoxelCompound;
import it.units.erallab.hmsrobots.problems.Episode;
import it.units.erallab.hmsrobots.problems.Locomotion;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Util;
import it.units.erallab.hmsrobots.viewers.GraphicsDrawer;
import it.units.erallab.hmsrobots.viewers.VideoGridWriter;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Function;
import java.util.logging.Logger;
import org.apache.commons.csv.CSVFormat;
import org.apache.commons.csv.CSVRecord;
import org.apache.commons.lang3.tuple.Pair;
import org.dyn4j.dynamics.Settings;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class Video {

  private static final Logger L = Logger.getLogger(Video.class.getName());
  
  public static void main(String[] args) throws IOException, ClassNotFoundException {
    String serializedColumnName = "serialized";
    Grid<Map<String, String>> filterGrid = Grid.create(3, 3);
    filterGrid.set(0, 0, filter("iterations=1;evolver=standard;type=phases;mutation.sigma=0.15"));
    filterGrid.set(0, 1, filter("iterations=50;evolver=standard;type=phases;mutation.sigma=0.15"));
    filterGrid.set(0, 2, filter("iterations=100;evolver=standard;type=phases;mutation.sigma=0.15"));
    filterGrid.set(1, 0, filter("iterations=1;evolver=standard;type=phases;mutation.sigma=0.25"));
    filterGrid.set(1, 1, filter("iterations=50;evolver=standard;type=phases;mutation.sigma=0.25"));
    filterGrid.set(1, 2, filter("iterations=100;evolver=standard;type=phases;mutation.sigma=0.25"));
    filterGrid.set(2, 0, filter("iterations=1;evolver=standard;type=phases;mutation.sigma=0.35"));
    filterGrid.set(2, 1, filter("iterations=50;evolver=standard;type=phases;mutation.sigma=0.35"));
    filterGrid.set(2, 2, filter("iterations=100;evolver=standard;type=phases;mutation.sigma=0.35"));
    Locomotion locomotion = new Locomotion(
            30,
            Main.createTerrain("flat"),
            Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
            1,
            new Settings()
    );
    fromCSV(
            "/home/eric/experiments/2dhmsr/hpc/serialized.4.2876077.aa034532.txt",
            "/home/eric/experiments/2dhmsr/biped.sigmas.7.mp4",
            600, 600, 20,
            serializedColumnName,
            filterGrid,
            s -> Util.<VoxelCompound.Description>lazilyDeserialize(s),
            locomotion
    );
  }
  
    private static <S> void fromCSV(String inputFile, String outputFile, int w, int h, int frameRate, String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer, Episode<S, ?> episode) throws IOException {
    //read data
    Grid<Pair<String, S>> namedSolutionGrid = Grid.create(filterGrid);
    Reader in = new FileReader(inputFile);
    for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
      String serialized = record.get(serializedColumnName);
      if (serialized != null) {
        for (Grid.Entry<Map<String, String>> gridEntry : filterGrid) {
          //check filter
          boolean met = true;
          for (Map.Entry<String, String> condition : gridEntry.getValue().entrySet()) {
            met = met && condition.getValue().equals(record.get(condition.getKey()));
          }
          //put or replace in grid
          if (met) {
            L.info(String.format("Found record meeting %s", gridEntry.getValue()));
            namedSolutionGrid.set(
                    gridEntry.getX(),
                    gridEntry.getY(),
                    Pair.of(
                            gridEntry.getValue().toString(),
                            deserializer.apply(serialized)
                    ));
          }
        }
      }
    }
    ExecutorService executorService = Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    VideoGridWriter<S> writer = new VideoGridWriter<>(
            namedSolutionGrid, episode,
            w, h, frameRate,
            new File(outputFile),
            executorService,
            GraphicsDrawer.RenderingDirectives.create()
    );
    writer.run();
    executorService.shutdown();
  }

  private static Map<String, String> filter(String string) {
    Map<String, String> map = new HashMap<>();
    for (String pair : string.split(";")) {
      if (!pair.trim().isEmpty()) {
        String[] pieces = pair.trim().split("=");
        if (pieces.length > 1) {
          map.put(pieces[0].trim(), pieces[1].trim());
        }
      }
    }
    return map;
  }

}
