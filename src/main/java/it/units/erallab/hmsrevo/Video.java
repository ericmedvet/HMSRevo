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
import it.units.erallab.hmsrobots.viewers.GridEpisodeRunner;
import it.units.erallab.hmsrobots.viewers.GridFileWriter;
import it.units.erallab.hmsrobots.viewers.GridOnlineViewer;
import it.units.erallab.hmsrobots.viewers.GridSnapshotListener;
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

import static it.units.malelab.jgea.core.util.Args.*;
import java.util.concurrent.ScheduledExecutorService;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class Video {

  private static final Logger L = Logger.getLogger(Video.class.getName());

  public static void main(String[] args) throws IOException, ClassNotFoundException {
    //read main params
    String serializedColumnName = a(args, "serializedColumnName", "serialized");
    String inputFile = a(args, "inputFile", "input.txt");
    String outputFile = a(args, "outputFile", "output.mp4");
    boolean online = b(a(args, "online", "false"));
    String terrain = a(args, "terrain", "flat");
    int w = i(a(args, "w", "800"));
    int h = i(a(args, "h", "600"));
    int frameRate = i(a(args, "frameRate", "25"));
    int controlStepInterval = i(a(args, "controlStepInterval", "1"));
    double finalT = d(a(args, "finalT", "30"));
    //read grid params
    //TODO: fix, doesn't work without filters
    String commonFilter = a(args, "commonFilter", "");
    String xFilters = a(args, "xFilter", "");
    String yFilters = a(args, "yFilter", "");
    String[] xFilterValues = xFilters.split(":")[1].split(",");
    String[] yFilterValues = yFilters.split(":")[1].split(",");
    String xFilterKey = xFilters.split(":")[0];
    String yFilterKey = yFilters.split(":")[0];
    Grid<Map<String, String>> filterGrid = Grid.create(xFilterValues.length, yFilterValues.length);
    for (int x = 0; x < xFilterValues.length; x++) {
      for (int y = 0; y < yFilterValues.length; y++) {
        String filter = commonFilter.replace("#", ";");
        filter = filter + ";" + xFilterKey + ":" + xFilterValues[x];
        filter = filter + ";" + yFilterKey + ":" + yFilterValues[y];
        filterGrid.set(x, y, filter(filter));
      }
    }
    //prepare problem
    Locomotion locomotion = new Locomotion(
            finalT,
            Locomotion.createTerrain(terrain),
            Lists.newArrayList(Locomotion.Metric.TRAVEL_X_VELOCITY),
            controlStepInterval,
            new Settings()
    );
    fromCSV(
            inputFile,
            online,
            outputFile,
            w, h, frameRate,
            serializedColumnName,
            filterGrid,
            s -> Util.<VoxelCompound.Description>lazilyDeserialize(s),
            locomotion
    );
  }

  private static <S> void fromCSV(String inputFile, boolean online, String outputFile, int w, int h, int frameRate, String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer, Episode<S, ?> episode) throws IOException {
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
    for (Grid.Entry<Pair<String, S>> entry : namedSolutionGrid) {
      if (entry.getValue() == null) {
        throw new IllegalArgumentException(String.format(
                "Cell in position (%d,%d) is null because of filter %s",
                entry.getX(), entry.getY(),
                filterGrid.get(entry.getX(), entry.getY())
        ));
      }
    }
    ScheduledExecutorService uiExecutor = Executors.newScheduledThreadPool(4);
    ExecutorService executor = online ? Executors.newCachedThreadPool() : Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors());
    GridSnapshotListener gridSnapshotListener = null;
    if (online) {
      gridSnapshotListener = new GridOnlineViewer(Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor, GraphicsDrawer.RenderingDirectives.create());
      ((GridOnlineViewer)gridSnapshotListener).start();
    } else {
      gridSnapshotListener = new GridFileWriter(w, h, frameRate, new File(outputFile), Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor, GraphicsDrawer.RenderingDirectives.create());
    }
    GridEpisodeRunner<S> runner = new GridEpisodeRunner<>(
            namedSolutionGrid, episode,
            gridSnapshotListener,
            executor
    );
    runner.run();
    if (!online) {
      executor.shutdown();
      executor.shutdownNow();
    }
  }

  private static Map<String, String> filter(String string) {
    Map<String, String> map = new HashMap<>();
    for (String pair : string.split(";")) {
      if (!pair.trim().isEmpty()) {
        String[] pieces = pair.trim().split(":");
        if (pieces.length > 1) {
          map.put(pieces[0].trim(), pieces[1].trim());
        }
      }
    }
    return map;
  }

}
