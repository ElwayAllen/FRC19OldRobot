/*
 * The MIT License
 *
 * Copyright 2019 lwa.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.subsystems.VectorDriver;
import frc.subsystems.LimelightRouteMgr;

/**
 * Command to determine a route to the target currently in
 * view by the Limelight camera, then drive the robot to
 * that target (if any).  Two steps:
 *  - Use the Limelight's vision pipeline to find the target
 *    and calculate a route to it (list of vectors)
 *  - Drive the list of vectors (at a specified maximum velocity)
 *    to get to the target
 * Can time out if no target is visible.  Can abort if (e.g.) the
 * robot collides with another robot or other obstacle during the
 * trip.
 */

public class DriveRouteToTarget extends CommandGroup {

 /**
   * Constructor given the drive train and nav unit.
   * @param rteMgr Manager for finding the route to the target
   * @param driver Vector driver for driving route to the target
   * @param driveVel Velocity to drive the route at
   */
  public DriveRouteToTarget(LimelightRouteMgr rteMgr, VectorDriver driver, double driveVel) {

    addSequential(new GetRouteToTarget(rteMgr));
    addSequential(new DriveRoute(rteMgr, driver, driveVel));
  }

}
