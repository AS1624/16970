package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class camera {
    public static double x = 0;
    public static double y = 0;
    public static double z = 0;
    //TODO: add camera position relative to robot
    public static double pich = 0;
    public static double yaw = 0;
    //TODO: add camera ROTATION
}
/*
all diagrams are top-down

                 FRONT

______<O>__________________________________
|      |                                  |
|      |                                  |
|      | + y                              |
|      |                                  |
|      |                                  |
|      |                                  |
|      |          center                  |
|      |_____________                     |   RIGHT
|          + x                            |
|                                         |
|                                         |
|                                         |
|                                         |
|                                         |
|_________________________________________|


      FRONT
         ____
        |    \___  + yaw
        |        \__
        |           \
        |           (camera)
        |              \
        *_______________|    RIGHT
 */