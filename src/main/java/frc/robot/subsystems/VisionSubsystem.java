package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

    private DoubleArraySubscriber[][] vecSubscribers;
    private IntegerSubscriber[] idSubscribers;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private LinkedList<Pose2d> poses;
    private LinkedList<Integer> timestamps;

    public VisionSubsystem() {
        vecSubscribers = new DoubleArraySubscriber[4][2];
        idSubscribers = new IntegerSubscriber[4];

        for(int a = 0; a < 4; a++) {
            idSubscribers[a] = inst.getTable("fisheye").getIntegerTopic(
                Constants.Vision.TOPIC_NAMES[a][3]).subscribe(Constants.Vision.ID_DEFAULT_VALUE);
            for(int b = 0; b < 2; b++) {
                vecSubscribers[a][b] = inst.getTable("fisheye").getDoubleArrayTopic(
                        Constants.Vision.TOPIC_NAMES[a][b]).subscribe(Constants.Vision.VECTOR_DEFAULT_VALUE);
            }
        }
    }

    private Pose2d translateToFieldPose(double[] translation, double[] rotation, int tagId, int camera) {
        Vector<N3> rvec = new Vector<>(Nat.N3());
        rvec.set(0, 0, rotation[0]);
        rvec.set(1, 0, rotation[1]);
        rvec.set(2, 0, rotation[2]);
        Pose2d camToTagPose = new Pose2d(-translation[0], translation[2], new Rotation3d(rvec).toRotation2d());

        Rotation2d botToTagRotation = new Rotation2d(
            camToTagPose.getRotation().getRadians() - Constants.Vision.CAMERA_DISTANCES_TO_CENTER_METERS[camera].getRotation().getRadians()); 
        // If it turns out to be clockwise, make cam rotation negative

        double botToTagX = (botToTagRotation.getDegrees() > 180) ? 
            -camToTagPose.getX() + Constants.Vision.CAMERA_DISTANCES_TO_CENTER_METERS[camera].getX() : 
            camToTagPose.getX() + Constants.Vision.CAMERA_DISTANCES_TO_CENTER_METERS[camera].getX();
        double botToTagY = (botToTagRotation.getDegrees() > 90 && botToTagRotation.getDegrees() < 270) ? 
            -camToTagPose.getY() + Constants.Vision.CAMERA_DISTANCES_TO_CENTER_METERS[camera].getY() : 
            camToTagPose.getY() + Constants.Vision.CAMERA_DISTANCES_TO_CENTER_METERS[camera].getY();
        Pose2d botToTagPose = new Pose2d(botToTagX, botToTagY, botToTagRotation);
            
        
        return null;
    }

    @Override
    public void periodic() {
        for(int a = 0; a < 4; a++) {
            TimestampedDoubleArray[] tvec = vecSubscribers[a][0].readQueue();
            TimestampedDoubleArray[] rvec = vecSubscribers[a][1].readQueue();
            TimestampedInteger[] ids = idSubscribers[a].readQueue();
            for(int b = 0; b < ids.length; b++) {
                if(rvec[b].timestamp == tvec[b].timestamp && tvec[b].timestamp == ids[b].timestamp) {
                    translateToFieldPose(tvec[b].value, rvec[b].value, (int) ids[a].value, a);
                }
            }
        }
    }

    public LinkedList<Pose2d> getPoseQueue() {
        return poses;
    }
}