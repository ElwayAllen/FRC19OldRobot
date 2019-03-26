/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.util;

import frc.util.Vec2d;

/**
 * Utility class for mapping a robot vector to the target normal vector
 * for the target the robot 'must' be pointing at -- assuming it's pointing
 * at a target.  In the 2019 game there are different mappings for the
 * 'orthogonal' targets -- those on the cargo ship and the loading station --
 * and the 'rocket' targets, which are offset at about 61 deg.
 */
public class TargetVecMapper {

  // Well-known height of all targets above the carpet
  public static double TARGET_HEIGHT = 19d + 25d/2d + 5.25/2d;

  // These are the orthogonal standard target normal vectors
  static final Vec2d[] stdTargets = {
    Vec2d.makeCart(1.0d, 0.0d),
    Vec2d.makeCart(0.0d, 1.0d),
    Vec2d.makeCart(-1.0d, 0.0d),
    Vec2d.makeCart(0.0d, -1.0d) 
  };

  // This is the mapping from robot vector angle (in degrees) to
  // standard robot target vector
  static final double[][] stdTargetMapping = {
    {-45.0, 45.0, 3.0},
    {45.0, 135.0, 2.0},
    {135.0, 180.0, 1.0},
    {-180.0, -135.0, 1.0},
    {-135.0, -45.0, 0.0}
  };

  /** These are the normal vectors to the 'rocket' targets:
   * those on the hatch ports of the rockets (located at +-61 deg from the field
   * center)
   */
  static final Vec2d[] rocketTargets = {
    Vec2d.makePolar(1.0, Math.toRadians(119.0d)),
    Vec2d.makePolar(1.0, Math.toRadians(-119.0d)),
    Vec2d.makePolar(1.0, Math.toRadians(-61.0d)),
    Vec2d.makePolar(1.0, Math.toRadians(61.0d))
  };

  /** These are the robot angle ranges corresponding to the
   * associated rocket target normal vectors.  If the robot's field-relative heading
   * is in the associated range, and a target is in view, its rocket target normal has
   * the specified index in the rocketTargets table.Headings are in degrees from -180
   * to +180.
   */
  static final double[][] rocketTargetMapping = {
    {0.0, 90.0, 1.0},
    {90.0, 180.0, 0.0},
    {-90.0, 0.0, 2.0},
    {-180.0, -90.0, 3.0}
  };



    /**
   * Find the target normal for the standard target that's in view from the given yaw
   * angle.  Only one such target should be in view.
   */
  public static Vec2d getStdTargNorm(double yaw){
 
    for (double[] mpg: stdTargetMapping) {
      if ((yaw >= mpg[0]) && (yaw <= mpg[1])) {
        return stdTargets[(int)mpg[2]];
      }
    }
    throw new RuntimeException("Could not find standard target for yaw " + yaw);
  }

    /**
   * Find the target normal for the rocket target that's in view from the given yaw
   * angle.  Only one such target should be in view.
   */
  public static Vec2d getRocketTargNorm(double yaw){
 
    for (double[] mpg: rocketTargetMapping) {
      if ((yaw >= mpg[0]) && (yaw <= mpg[1])) {
        return rocketTargets[(int)mpg[2]];
      }
    }
    throw new RuntimeException("Could not find rocket target for yaw " + yaw);
  }


}
