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
import it.units.erallab.hmsrobots.tasks.Locomotion;
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
import it.units.erallab.hmsrobots.tasks.Task;
import it.units.erallab.hmsrobots.viewers.FramesFileWriter;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class FramesImage {

  private static final Logger L = Logger.getLogger(FramesImage.class.getName());

  public static void main(String[] args) throws IOException, ClassNotFoundException {
    //read main params
    String serializedColumnName = a(args, "serializedColumnName", "r0.serialized.description");
    String inputFile = a(args, "inputFile", "input.txt");
    String outputFile = a(args, "outputFile", "image.png");
    String terrain = a(args, "terrain", "flat");
    int w = i(a(args, "w", "200"));
    int h = i(a(args, "h", "200"));
    int controlStepInterval = i(a(args, "controlStepInterval", "1"));
    double initialT = d(a(args, "initialT", "5"));
    double finalT = d(a(args, "finalT", "5.5"));
    double frameDT = d(a(args, "frameDT", "0.1"));
    double dT = d(a(args, "dT", "0.0333"));
    String filterString = a(args, "filter", "");
    Settings settings = new Settings();
    settings.setStepFrequency(dT);
    //prepare problem
    Locomotion locomotion = new Locomotion(
            finalT,
            Locomotion.createTerrain(terrain),
            Lists.newArrayList(Locomotion.Metric.values()),
            controlStepInterval,
            new Settings()
    );
    //set filter
    Map<String, String> filter = Video.filter(filterString.replace("#", ";"));
    //read data
    Reader in = new FileReader(inputFile);
    for (CSVRecord record : CSVFormat.DEFAULT.withDelimiter(';').withFirstRecordAsHeader().parse(new FileReader(inputFile))) {
      String serialized = record.get(serializedColumnName);
      if (serialized != null) {
        //check filter
        boolean met = true;
        for (Map.Entry<String, String> condition : filter.entrySet()) {
          met = met && condition.getValue().equals(record.get(condition.getKey()));
        }
        //put or replace in grid
        if (met) {
          //prepare listener
          FramesFileWriter framesFileWriter = new FramesFileWriter(
                  initialT, finalT, frameDT, w, h, FramesFileWriter.Direction.HORIZONTAL,
                  new File(outputFile),
                  Executors.newFixedThreadPool(Runtime.getRuntime().availableProcessors()),
                  GraphicsDrawer.RenderingDirectives.create()
          );
          L.info(String.format("Found record meeting %s", filter));
          locomotion.apply(Util.<VoxelCompound.Description>lazilyDeserialize(serialized), framesFileWriter);
          framesFileWriter.flush();
          break;
        }
      }
    }
  }

  private static <S> void fromCSV(String inputFile, boolean online, String outputFile, int w, int h, int frameRate, String serializedColumnName, Grid<Map<String, String>> filterGrid, Function<String, S> deserializer, Task<S, ?> task) throws IOException {
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
      ((GridOnlineViewer) gridSnapshotListener).start(5);
    } else {
      gridSnapshotListener = new GridFileWriter(w, h, frameRate, new File(outputFile), Grid.create(namedSolutionGrid, Pair::getLeft), uiExecutor, GraphicsDrawer.RenderingDirectives.create());
    }
    GridEpisodeRunner<S> runner = new GridEpisodeRunner<>(
            namedSolutionGrid, task,
            gridSnapshotListener,
            executor
    );
    runner.run();
    if (!online) {
      executor.shutdownNow();
      uiExecutor.shutdownNow();
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
