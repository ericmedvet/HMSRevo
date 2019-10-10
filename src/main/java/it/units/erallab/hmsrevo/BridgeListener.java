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

import it.units.erallab.hmsrobots.Snapshot;
import it.units.erallab.hmsrobots.viewers.SnapshotListener;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.event.Event;

/**
 *
 * @author Eric Medvet <eric.medvet@gmail.com>
 */
public class BridgeListener implements Listener {

  public static class SnapshotEvent implements Event {

    private final Snapshot snapshot;

    public SnapshotEvent(Snapshot snapshot) {
      this.snapshot = snapshot;
    }

    public Snapshot getSnapshot() {
      return snapshot;
    }

  }
  
  private final SnapshotListener snapshotListener;

  public BridgeListener(SnapshotListener snapshotListener) {
    this.snapshotListener = snapshotListener;
  }

  public SnapshotListener getSnapshotListener() {
    return snapshotListener;
  }

  @Override
  public void listen(Event event) {
    if (event instanceof SnapshotEvent) {
      snapshotListener.listen(((SnapshotEvent)event).getSnapshot());
    }
  }

}
