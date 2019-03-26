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

package frc.util;

import java.util.List;
import java.util.ArrayList;

/**
 * Calculator to determine route to target found by the vision system.
 * A TargetCalculator is initialized with camera-specific parameters (aim
 * angle, height above ground), and can calculate a route the robot should
 * drive to arrive at the target.
 * The calculated route is designed so that the robot arrives normal (i.e.
 * perpendicular) to the target and at a fixed, known distance from the target.
 * @author lwa
 */
public class TargetCalculator {

    /**
     * The camera's lens height above the floor, in convenient units
     * (must be the same units as used in routeToTarget computations)
     */
    private final double m_cameraHeight;

    /**
     * The camera's vertical aiming angle relative to the horizon, in
     * radians (e.g. a camera pointing level with the floor will have an
     * aiming angle of 0)
     */
    private final double m_vAimAngle;

    /**
     * Minimum rotation we'll deal with, in radians
     */
    public static final double MIN_ROTATE = 0.02;

    /**
     * Constructor given the basic camera parameters
     * @param camHeight Camera height above floor
     * @param vAim Vertical aiming angle relative to horizon
     * (in degrees)
     */
    public TargetCalculator(double camHeight, double vAim) {

        m_cameraHeight = camHeight;
        m_vAimAngle = Math.toRadians(vAim);
    }

    /**
     * Determine the "target vector" between the camera lens and the target identified
     * on the camera's screen with the specified central x and y angles, given the
     * necessary information about the camera and the target.  The target vector is a
     * 2-dimensional vector, in the plane of the field, whose length is the floor distance
     * between the camera lens and the (floor projection of the) target, and whose 
     * direction is direction is the direction from the lens to the target.
     * The angle of the target vector is the robot's current aiming angle minus the central
     * x angle of the target (minus, because field angles are measured CCW and the aiming
     * angle is measured CW).  We determine the distance by computing the angle
     * of the target above the horizontal (from the target's central y angle plus the mounting
     * angle of the camera), and then using the arctan of that angle together with the 
     * target height above the camera mount to calculate the distance.
     * @param tx Central x angle of the target in degrees (for Limelight, -27 to +27 l to r)
     * @param ty Central y angle of the target in degrees (for Limelight, -20.5 to 20.5 b to t)
     * @param robotVec Unit vector (field-relative) in the robot's current direction
     * @param targHeight Height of target (in units) above the floor
     * @return Field-relative vector from camera lens to target
     */
    public Vec2d getTargetVector(double tx, double ty, Vec2d robotVec, double targHeight) {

        System.out.println("getTargVec:centralXAngle:" + tx);
        System.out.println("getTargVec:centralYAngle:" + ty);

        /* Next, we can compute the distance to the target from the camera, based on the camera's height,
         * its aiming angle, and the central Y angle obtained above:
         *   distance = (targ ht - camera ht) / tan(camera angle + central y angle)
         * This formula is inaccurate if the target height and camera height are "too close" or the
         * sum of the camera angle and central y angle are too close to PI/2 radians (90 deg) -- so
         * robot designers should avoid those situations!
         */
        double targetDistance = ((targHeight - m_cameraHeight) / Math.tan(Math.toRadians(ty) + m_vAimAngle));
        System.out.println("getTargVec:targetDistance:" + targetDistance);

        /* (robot angle - X angle) and distance give us the target vector */
        Vec2d targetVec = Vec2d.makePolar(targetDistance, robotVec.getTheta() - Math.toRadians(tx));
        return targetVec;
    }

    /**
     * Calculate the desired route to the target identified on the camera's screen
     * with the specified central x and y angles, given the necessary information about
     * the robot and the target.  We need to know:
     *  - the "target vector", as returned by the "getTargetVector" method of this class
     *    (above) -- a 2d vector between the camera lens and the target as projected onto
     *    the floor of the field
     *  - the camera's offset from the robot centerline (specified as a vector)
     *  - the target alignment, specified as a unit vector normal (perpendicular) to the
     *    target and pointing away from the target -- i.e. in the reflected-light direction
      *  - the desired "normal distance" -- i.e. the minimum distance we want the robot to be
     *    away from the target when it makes its final turn to drive perpendicular to the
     *    target to drop off its payload.
     * @param camVec Camera vector: vector from camera lens to robot's center; perpendicular to
     * robotVec, for correcting offset of camera from the robot's centerline
     * @param targNorm Unit vector (field-relative) pointing perpendicularly away from target
     * @param normDist Minimum distance from target (in units) for robot to drive normal to
     * target at end of its route
     * @return Route to target: List of vectors the robot must drive to get to the target from its
     * current position
     */
    public List<Vec2d> getRouteToTarget(Vec2d targetVec, Vec2d camVec,
                                        Vec2d targNorm, double normDist) {

        Vec2d normVec = targNorm.mulScalar(-normDist);
        Vec2d interceptVec = targetVec.sub(normVec);
        interceptVec = interceptVec.sub(camVec);

        ArrayList<Vec2d> ret = new ArrayList<Vec2d>(2);
        ret.add(interceptVec);
        ret.add(normVec);
        return ret;
    }

}
