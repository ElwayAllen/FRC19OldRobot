/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import java.util.List;
import java.util.ArrayList;
import frc.util.Vec2d;

/**
 * Simple route manager for test driving
 * Just create a fixed list of vectors for driving, to test the "DriveRoute"
 * command.
 */
public class TestRouteMgr extends RouteMgr {

  public TestRouteMgr() {
  }

  public void initRoute() {
    Vec2d vec1 = Vec2d.makePolar(48.0d, Math.PI / 2.0d);
    Vec2d vec2 = Vec2d.makePolar(48.0d, -Math.PI / 2.0d);
    List<Vec2d> vecs = new ArrayList<>();
    vecs.add(vec1);
    vecs.add(vec2);
    setRoute(vecs);
  }
}
