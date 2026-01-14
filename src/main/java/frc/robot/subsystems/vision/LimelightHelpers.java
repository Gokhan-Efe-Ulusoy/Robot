package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightHelpers {
    public static boolean hasTargets(String limelightName) {
        return getDouble(limelightName, "tv") > 0.5;
    }
    
    public static double getTX(String limelightName) {
        return getDouble(limelightName, "tx");
    }
    
    public static double getTY(String limelightName) {
        return getDouble(limelightName, "ty");
    }
    
    public static double getTA(String limelightName) {
        return getDouble(limelightName, "ta");
    }
    
    public static int getAprilTagID(String limelightName) {
        return (int) getDouble(limelightName, "tid");
    }
    
    public static double getLatency(String limelightName) {
        return getDouble(limelightName, "tl");
    }
    
    public static double getTotalLatency(String limelightName) {
        return getLatency(limelightName) + 
               getDouble(limelightName, "cl");
    }
    
    public static Pose2d getBotPose2d_wpiBlue(String limelightName) {
        double[] poseArray = getDoubleArray(limelightName, "botpose_wpiblue");
        if (poseArray.length < 6) {
            return new Pose2d();
        }
        return toPose2d(poseArray);
    }
    
    public static Pose2d getBotPose2d_wpiRed(String limelightName) {
        double[] poseArray = getDoubleArray(limelightName, "botpose_wpired");
        if (poseArray.length < 6) {
            return new Pose2d();
        }
        return toPose2d(poseArray);
    }
    
    public static Pose3d getCameraPose3d_TargetSpace(String limelightName) {
        double[] poseArray = getDoubleArray(limelightName, "camerapose_targetspace");
        if (poseArray.length < 6) {
            return new Pose3d();
        }
        return toPose3d(poseArray);
    }
    
    public static Pose3d getTargetPose3d_CameraSpace(String limelightName) {
        double[] poseArray = getDoubleArray(limelightName, "targetpose_cameraspace");
        if (poseArray.length < 6) {
            return new Pose3d();
        }
        return toPose3d(poseArray);
    }
    
    public static void setLEDMode(String limelightName, int mode) {
        setNumber(limelightName, "ledMode", mode);
    }
    
    public static void setPipeline(String limelightName, int pipeline) {
        setNumber(limelightName, "pipeline", pipeline);
    }
    
    public static void setCameraMode(String limelightName, int mode) {
        setNumber(limelightName, "camMode", mode);
    }
    
    public static void takeSnapshot(String limelightName) {
        setNumber(limelightName, "snapshot", 1);
    }
    
    
    private static NetworkTable getTable(String limelightName) {
        return NetworkTableInstance.getDefault().getTable(limelightName);
    }
    
    private static double getDouble(String limelightName, String key) {
        return getTable(limelightName).getEntry(key).getDouble(0.0);
    }
    
    private static double[] getDoubleArray(String limelightName, String key) {
        return getTable(limelightName).getEntry(key).getDoubleArray(new double[0]);
    }
    
    private static void setNumber(String limelightName, String key, Number value) {
        getTable(limelightName).getEntry(key).setNumber(value);
    }
    
    private static Pose2d toPose2d(double[] poseArray) {
        return new Pose2d(
            poseArray[0],
            poseArray[1],
            Rotation2d.fromDegrees(poseArray[5])
        );
    }
    
    private static Pose3d toPose3d(double[] poseArray) {
        return new Pose3d(
            new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
            new Rotation3d(
                Math.toRadians(poseArray[3]),
                Math.toRadians(poseArray[4]),
                Math.toRadians(poseArray[5])
            )
        );
    }
    
    public static class LimelightResults {
        public boolean valid;
        public double tx;
        public double ty;
        public double ta;
        public int aprilTagID;
        public Pose2d botPose;
        public double latencyMs;
        public int pipelineIndex;
        
        public LimelightResults(String limelightName) {
            this.valid = hasTargets(limelightName);
            if (valid) {
                this.tx = getTX(limelightName);
                this.ty = getTY(limelightName);
                this.ta = getTA(limelightName);
                this.aprilTagID = getAprilTagID(limelightName);
                this.botPose = getBotPose2d_wpiBlue(limelightName);
                this.latencyMs = getTotalLatency(limelightName);
                this.pipelineIndex = (int) getDouble(limelightName, "getpipe");
            }
        }
    }
}