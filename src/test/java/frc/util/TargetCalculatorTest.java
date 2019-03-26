/*
 * The MIT License
 *
 * Copyright 2018 lwa.
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

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

/**
 *
 * @author lwa
 */
public class TargetCalculatorTest {

    static final double vRes = 41;      // vertical resolution in degrees
    static final double hRes = 54;      // horizontal resolution in degrees
    static final double EPS = 0.01d;


    public TargetCalculatorTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }

    /**
     * Simplest target calculator test
     */
    @Test
    public void test1() {

        /* So, target is dead ahead; vector should be at 0 radians.  Target
         * is at the top of the camera's FOV, camera at height 0 above floor.
         * Angle from camera to target = 1/2 vFov = 20.5 deg; so if target
         * is 10.0 units above floor (= 10.0 units above camera), 
         *   distance = 10.0 / tan(20.5 deg) ~=  26.746 units
         * This gives us the distance vector.
         * Since the target is dead ahead, the intercept vector should be
         * parallel to the normal vector.
         */
        Vec2d robotVec = Vec2d.makeCart(0.0d, 1.0d);
        Vec2d targNorm = Vec2d.makeCart(0.0d, -1.0d);
        Vec2d camVec = Vec2d.ZERO;
        double targHeight = 10.0;
        double normDist = 1.0;      // drive for 1.0 units normal at end
        TargetCalculator cUtil = new TargetCalculator(0.0d, 0.0d);
        Vec2d targetVec = cUtil.getTargetVector(0.0d, vRes/2.0d, robotVec, targHeight);
        List<Vec2d> route = cUtil.getRouteToTarget(targetVec, camVec, targNorm, normDist);
        
        Vec2d direct = targetVec;
        double expectedDist = 26.746d;
        double expectedAng = robotVec.getTheta();
        assertEquals(expectedDist, direct.getR(), EPS);
        assertEquals(expectedAng, direct.getTheta(), EPS);

        Vec2d intercept = route.get(0);
        expectedDist = 26.746d - 1.0d;
        expectedAng = robotVec.getTheta();
        assertEquals(expectedDist, intercept.getR(), EPS);
        assertEquals(expectedAng, intercept.getTheta(), EPS);

        Vec2d norm = route.get(1);
        expectedDist = 1.0d;
        expectedAng = robotVec.getTheta();
        assertEquals(expectedDist, norm.getR(), EPS);
        assertEquals(expectedAng, norm.getTheta(), EPS);
  }

    /**
     * Robot pointed down field as in test1, target pointed opposite,
     * target at top of robot FOV as in test1, but this time target
     * at left edge of robot FOV (pixel location (0,0)).  Camera ht
     * 0 above floor, target 10 above floor; dist ~= 26.746 units.
     * But now, targ vec angle is -27 deg () = .471 rad + robot angle.
     * Same distance and target height as case 1 but robot pointed at +-
     * 1/2 hFov (so target would be at edge of fov if robot was pointed
     * straight ahead).
     * Intercept vector should be distance to target (26.746 units) in
     * robot's current direction; normal vector should be 1.0, dir 0.
     */
//    @Test
    public void test2() {
        Vec2d robotVec = Vec2d.makeCart(0.0d, 1.0d);
        Vec2d targNorm = Vec2d.makeCart(0.0d, -1.0d);
        Vec2d camVec = Vec2d.ZERO;
        double targHeight = 10.0;
        double normDist = 1.0;      // drive for 1.0 units normal at end
        TargetCalculator cUtil = new TargetCalculator(0.0d, 0.0d);

        Vec2d targetVec = cUtil.getTargetVector((-hRes/2.0d), vRes/2.0, robotVec, targHeight);
        List<Vec2d> route = cUtil.getRouteToTarget(targetVec, camVec, targNorm, normDist);

        Vec2d direct = targetVec;
        double expectedDist = 26.746d;
        double expectedAng = Math.toRadians((-hRes/2)) + robotVec.getTheta();
        assertEquals(expectedDist, direct.getR(), EPS);
        assertEquals(expectedAng, direct.getTheta(), EPS);
    
        Vec2d norm = route.get(0);
        expectedDist = 1.0d;
        expectedAng = robotVec.getTheta();
        assertEquals(expectedDist, norm.getR(), EPS);
        assertEquals(expectedAng, norm.getTheta(), EPS);
        
        Vec2d intercept = route.get(1);
        expectedDist = 25.97d;
        expectedAng = 1.08d;
        assertEquals(expectedDist, intercept.getR(), EPS);
        assertEquals(expectedAng, intercept.getTheta(), EPS);
    }

    /**
     * In this test, robot is 2 units from the target and pointed 3 deg left 
     * of normal line.  Normal drive distance will be sqrt(3) units, so
     * intercept line will be 1 unit (a 30-60-90 triangle), so central X angle
     * is 27 degrees (left edge).  Camera is mounted at +15 deg and we arrange
     * it so target is at +15 deg central Y-angle, so target height is (2 tan 30 deg)
     * i.e. about 1.155 units.
     * From Limelight: vph = 2*tan(20.5 deg) = 0.748
     *                 ny = y / (vph/2) = tan(ay)/(vph/2) = 0.698
     *                 py = 119.5 - (120)ny = 33.6
     * px = 0 since target is at left edge of view, but in general:
     *                 vpw = tan(27 deg) = 1.02
     *                 nx = x / vpw = tan(ax) / vpw = -1.0
     *                 px = 159.5 - (160)nx = 0.5
     */
//    @Test
    public void test3() {
        Vec2d robotVec = Vec2d.makePolar(1.0d, Math.toRadians(93.0d));
        Vec2d targNorm = Vec2d.makePolar(1.0d, -(Math.PI / 2.0));
        Vec2d camVec = Vec2d.ZERO;
        double targHeight = 2.255d;
        double camHeight = 1.0d;
        double camAngle = 15.0d;    // angle above horizon in degrees
        double normDist = Math.sqrt(3.0d);      // drive for root(3) units normal at end
        TargetCalculator cUtil = new TargetCalculator(camHeight,camAngle);

        double tx = 27.0d;
        double ty = 15.0;
        Vec2d targetVec = cUtil.getTargetVector(tx, ty, robotVec, targHeight);
        List<Vec2d> route = cUtil.getRouteToTarget(targetVec, camVec, targNorm, normDist);

        Vec2d direct = targetVec;
        double expectedDist = 2.0d;
        double expectedAng = Math.toRadians((-hRes/2)) + robotVec.getTheta();
        assertEquals(expectedDist, direct.getR(), EPS);
        assertEquals(expectedAng, direct.getTheta(), EPS);
    
        Vec2d norm = route.get(1);
        expectedDist = normDist;
        expectedAng = -targNorm.getTheta();
        assertEquals(expectedDist, norm.getR(), EPS);
        assertEquals(expectedAng, norm.getTheta(), EPS);
        
        Vec2d intercept = route.get(0);
        expectedDist = 1.0d;
        expectedAng = Math.PI;
        assertEquals(expectedDist, intercept.getR(), EPS);
        assertEquals(expectedAng, intercept.getTheta(), EPS);
     }
}
