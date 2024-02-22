package frc.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class fieldShoot {
    Transform3d robot2Apriltag;
    static double desired = 0.32;
    static double settle = 0.001;//in meters haven't tested

    public static boolean OKshoot(Transform3d toApriltag){
        if(Math.abs(Math.abs(toApriltag.getY()) - desired) <= settle){
            return true;
        }else{
            return false;
        }
    }
}
