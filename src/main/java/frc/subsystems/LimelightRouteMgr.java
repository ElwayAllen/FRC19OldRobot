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

package frc.subsystems;

import java.util.List;

import frc.util.TargetCalculator;
import frc.util.TargetVecMapper;
import frc.util.Vec2d;

/**
 * Find and manage driving to a route to a target found by the Limelight camera
 * Calculate the distance to the target using trig (as suggested by Limelight docs),
 * then calculate the vectors to drive to get to the target, arriving via a vector
 * normal to the target of a desired length.
 */
public class LimelightRouteMgr extends RouteMgr {

  /**
   * Our camera
   */
  private Limelight m_cam;

  /**
   * Our nav subsystem
   */
  private Nav m_nav;

  /**
   * Target calculator for finding route to target
   */
  private TargetCalculator m_calc;

  /**
   * Current target vector, for debugging only
   */
  private Vec2d m_targetVec;

  /**
   * Constructor given the camera we're using
   * @param cam The Limelight camera we're using to find our target
   * @param nav The navigation subsystem we're using to find our orientation
   */
  public LimelightRouteMgr(Limelight cam, Nav nav) {
    m_cam = cam;
    m_nav = nav;
    m_calc = new TargetCalculator(Limelight.HEIGHT, Limelight.ANGLE_FROM_HORIZONTAL);
    m_targetVec = null;
  }

  /**
   * Initialize the target-finding function.
   * Tell the Limelight to go into target-finding mode.
   */
  public void tryToFindTarget() {
    m_cam.visionMode();
  }

  /**
   * Return true iff there is currently a target in view
   * @return True iff there is currently a target in view
   */
  public boolean isTarget() {
    return m_cam.isTarget();
  }

  /**
   * Set up the route to the target currently in view
   * Should only be called when isTarget() is true.
   * Set up the route that driving commands can follow
   * to get to the target.
	 * @param targHeight Height of target (in units) above the floor
   * @param normDist Desired length of final (normal) vector to target
   */
  public void setupRouteToStdTarget(double targHeight, double normDist) {

    Vec2d robotVec = m_nav.getRobotVec();
    m_targetVec = m_cam.getTargetVector(robotVec, targHeight);
    Vec2d camVec = m_cam.getCameraVector(robotVec);
    Vec2d targNorm = TargetVecMapper.getStdTargNorm(Nav.fieldAngleToYaw(robotVec.getTheta()));
    List<Vec2d> route = m_calc.getRouteToTarget(m_targetVec, camVec, targNorm, normDist);
    setRoute(route);
  }

  @Override
  public void advance() {
    super.advance();
    if (!isActiveRoute()) {
      m_targetVec = null;
    }
  }

  /**
   * For debugging purposes, get the current target vector if any
   * @return The current target vector, or null if none
   */
  public Vec2d getTargetVector() {
    return m_targetVec;
  }

  /**
   * End the attempt to find a target; turn off the camera
   */
  public void endFindTarget() {
    m_cam.driverMode();
  }
}
