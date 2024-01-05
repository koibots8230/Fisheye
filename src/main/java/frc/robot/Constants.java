package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public class Constants {
    public static class Vision {
        public static final Pose2d[] CAMERA_DISTANCES_TO_CENTER_METERS = {
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(90))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(180))),
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(270))),
        }; //x is left, y is forward, counterclockwise on rotation

        public static final String[][] TOPIC_NAMES = {
            {
                "Cam1Tvec",
                "Cam1Rvec",
                "Cam1Ids"
            }, {
                "Cam2Tvec",
                "Cam2Rvec",
                "Cam2Ids"
            }, {
                "Cam3Tvec",
                "Cam3Rvec",
                "Cam3Ids"
            }, {
                "Cam4Tvec",
                "Cam4Rvec",
                "Cam4Ids"
            }
        };
        public static final double[] VECTOR_DEFAULT_VALUE = {0, 0, 0};
        public static final int ID_DEFAULT_VALUE = 0;

        public static final double FIELD_WIDTH_METERS = 8.02;
        public static final double FIELD_LENGTH_METERS = 16.54;

        public static final double MAX_MEASUREMENT_DIFFERENCE_METERS = 1;

        public static final Pose2d[] TAG_POSES_METERS = {
            new Pose2d(1.5, .5, new Rotation2d()),
            new Pose2d(2, .5, new Rotation2d())
        };
    }
}
