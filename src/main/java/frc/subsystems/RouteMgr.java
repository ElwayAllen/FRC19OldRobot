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

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.util.Vec2d;


/**
 * Manage a "route" (list of vectors to be driven)
 * This includes the notion of the "current" portion of the route being
 * driven.  A route is driven by first turning the robot to the desired
 * angle as specified by the vector, then driving for the specified distance.
 * The route manager tracks which vector we're driving, and whether we're
 * in the "turning" phase or the "driving" phase for that vector.
 * 
 * This class is intended to be extended by sensor-based (e.g. vision-based)
 * classes that can determine the route to be driven.
 * 
 * NOTE: Should we just change this to be an Iterable and use an Iterator<Vec2d>?
 */
public class RouteMgr extends Subsystem {

  /**
   * This is the list of vectors we're to drive
   */
  private List<Vec2d> m_route;

  /**
   * Index of current element in the route to drive to
   */
  private int m_curElem;

  /**
   * Empty constructor
   */
  public RouteMgr() {
    m_route = null;
    m_curElem = 0;
  }

  /**
   * Protected method to set up the list of drive vectors,
   * for use in derived classes.  Also initializes the current
   * element and current phase to the start of the list.
   * @param route The new route (list of vectors) to drive
   */
  protected void setRoute(List<Vec2d> route) {
    m_route = route;
    m_curElem = 0;
  }

  /**
   * Return True iff there is current vector being driven
   * (non-null route and current element < size of route)
   * @return True iff there is still active route to be driven
   */
  public boolean isActiveRoute() {
    return ((m_route != null) && (m_curElem < m_route.size()));
  }

  /**
   * Get the current drive vector (if any) from the current
   * route.  Return null if there is no current drive vector
   * (no route has been set or the entire route has been driven).
   * @return Current drive vector; null if none
   */
  public Vec2d getCurVector() {
    if ((m_route == null) || (m_curElem >= m_route.size())) {
      return null;
    }
    return m_route.get(m_curElem);
  }

  /**
   * Get the angle (in radians) of the current drive vector
   * from the current route.
   * @return Angle in radians of the current drive vector
   * @throws IllegalStateException if there is no current route
   * or the entire route has been driven
   */
  public double getCurAngle() {
    Vec2d curVec = getCurVector();
    if (curVec == null) {
      throw new IllegalStateException("No current drive vector");
    }
    return curVec.getTheta();
  }

  /**
   * Get the distance of the current drive vector
   * from the current route.
   * @return Distance of the current drive vector
   * @throws IllegalStateException if there is no current route
   * or the entire route has been driven
   */
  public double getCurDistance() {
    Vec2d curVec = getCurVector();
    if (curVec == null) {
      throw new IllegalStateException("No current drive vector");
    }
    return curVec.getR();
  }

  /**
   * Advance to the next phase and/or vector in the current route.
   * The path advances as "turn, drive" for each vector in the
   * route, until the route is completed.  This method is a no-op
   * if the route is already complete.  The user should call this
   * method after finishing each phase of the route.
   */
  public void advance() {
    if (m_curElem < m_route.size()) {
      m_curElem++;
    }
  }

/**
 * Reset the route manager
 */
public void reset() {
  m_route = null;
  m_curElem = 0;
}

  @Override
  public void initDefaultCommand() {
  }
}
