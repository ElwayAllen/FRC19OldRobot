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

import edu.wpi.first.wpilibj.command.Command;
import frc.subsystems.VectorDriver;
import frc.subsystems.RouteMgr;
import frc.util.Vec2d;

/**
 * Command to drive the robot according to the specified route.
 * Each time we're initialized, call the route manager to
 * obtain the route to drive (as a list of vectors), and the
 * vector driver to drive the list of vectors at a pre-determined
 * velocity (units per second).
 */
public class DriveRoute extends Command {

  /**
   * The route manager that will give us the route to be
   * driven (list of vectors) each time we're initialized
   */
  private RouteMgr m_rteMgr;

  /**
   * The driver subsystem that will handle driving to the
   * list of routes specified by the route manager
   */
  private VectorDriver m_driver;

  /**
   * Max velocity to drive at
   */
  private final double m_maxVel;

  /**
   * Constructor given the needed subsystems.
   * @param rteMgr The route manager
   * @param driver The vector driver for the route
   * @param maxVec Maximum velocity to drive at
   */
  public DriveRoute(RouteMgr rteMgr, VectorDriver driver, double maxVel) {
    super(rteMgr);
    requires(driver);
    m_rteMgr = rteMgr;
    m_driver = driver;
    m_maxVel = maxVel;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // Start driving the first vector in the route!
    if (!m_rteMgr.isActiveRoute()) {
      System.err.println("DriveRoute initialize called with no active route");
    } else {
      Vec2d curVec = m_rteMgr.getCurVector();
      m_driver.startDrive(curVec, m_maxVel);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
 
      // Nothing special to do while the driver is driving; it handles
      // its own motion
      if (m_driver.isDone()) {

          // Driver finished a vector!  Move on to next vector in route
          m_rteMgr.advance();
          if (m_rteMgr.isActiveRoute()) {
            // More to go; keep going!
            Vec2d curVec = m_rteMgr.getCurVector();
            m_driver.startDrive(curVec, m_maxVel);
          }
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

      // Done when no more routes and driver is done
      return ((!m_rteMgr.isActiveRoute()) && m_driver.isDone());
   }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  //
  @Override
  protected void interrupted() {
    m_driver.reset();
    m_rteMgr.reset();
  }
}
