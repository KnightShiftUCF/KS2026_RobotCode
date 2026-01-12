// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.jni.AprilTagJNI;
import edu.wpi.first.apriltag.*;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;

//navx dependencies
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //declare navx
  private AHRS navx;

  Thread m_visionThread;

  private RobotContainer m_robotContainer;


  private void startImageProcThread() {
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);
              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("AprilTag Detection", 640, 480);
              
              // Mats are very memory expensive. Reuse these Mats.
              Mat mat = new Mat();
              Mat grayMat = new Mat();
              
              // Create AprilTag detector
              AprilTagDetector detector = new AprilTagDetector();
              
              // Add tag family (tag36h11 is standard for FRC)
              detector.addFamily("tag36h11");
              
              // Configure detector for reasonable performance on RoboRIO 2
              AprilTagDetector.Config config = new AprilTagDetector.Config();
              config.numThreads = 4;
              config.quadDecimate = 2.0f;
              config.quadSigma = 0.0f;
              config.refineEdges = true;
              config.decodeSharpening = 0.25;
              detector.setConfig(config);
              
              // Create pose estimator
              // You'll need to adjust these camera parameters for your specific camera
              // tagSize is in meters (16 inches = 0.1524 meters for FRC tags)
              // fx, fy are focal lengths in pixels, cx, cy are principal point coordinates
              AprilTagPoseEstimator.Config poseConfig = 
                  new AprilTagPoseEstimator.Config(
                      0.1524,  // tag size in meters (6 inches)
                      640.0,   // fx (approximate, needs calibration)
                      640.0,   // fy (approximate, needs calibration)
                      320.0,   // cx (image width / 2)
                      240.0);  // cy (image height / 2)
              
              AprilTagPoseEstimator poseEstimator = new AprilTagPoseEstimator(poseConfig);
              
              // Colors for drawing
              Scalar greenColor = new Scalar(0, 255, 0);
              Scalar redColor = new Scalar(0, 0, 255);
              Scalar blueColor = new Scalar(255, 0, 0);
              Scalar cyanColor = new Scalar(255, 255, 0);
              Scalar magentaColor = new Scalar(255, 0, 255);
              
              while (!Thread.interrupted()) {
                // Grab frame from camera
                if (cvSink.grabFrame(mat) == 0) {
                  outputStream.notifyError(cvSink.getError());
                  continue;
                }
                
                // Convert to grayscale for AprilTag detection
                Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY);
                
                // Detect AprilTags
                AprilTagDetection[] detections = detector.detect(grayMat);
                
                // Draw detections on the color image
                for (AprilTagDetection detection : detections) {
                  // Draw the four corners of the tag
                  for (int i = 0; i < 4; i++) {
                    int j = (i + 1) % 4;
                    Point pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
                    Point pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
                    Imgproc.line(mat, pt1, pt2, greenColor, 2);
                  }
                  
                  // Draw center point
                  Point center = new Point(detection.getCenterX(), detection.getCenterY());
                  Imgproc.circle(mat, center, 5, redColor, -1);
                  
                  // Draw tag ID
                  String idText = "ID: " + detection.getId();
                  Imgproc.putText(mat, idText, 
                                  new Point(center.x + 10, center.y - 10),
                                  Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, blueColor, 2);
                  
                  // Draw decision margin (detection confidence)
                  String marginText = String.format("Margin: %.1f", detection.getDecisionMargin());
                  Imgproc.putText(mat, marginText,
                                  new Point(center.x + 10, center.y + 10),
                                  Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, blueColor, 1);
                  
                  // Estimate pose and draw 3D axes
                  Transform3d pose = poseEstimator.estimate(detection);
                  
                  // Project 3D points to 2D for visualization
                  // Draw coordinate axes: X=red, Y=green, Z=blue
                  double axisLength = 0.05; // 5cm axes in meters
                  
                  // Camera intrinsic matrix parameters
                  double fx = poseConfig.fx;
                  double fy = poseConfig.fy;
                  double cx = poseConfig.cx;
                  double cy = poseConfig.cy;
                  
                  // Get pose translation and rotation
                  Translation3d translation = pose.getTranslation();
                  Rotation3d rotation = pose.getRotation();
                  
                  // Origin point (tag center in 3D)
                  Point origin = project3DPoint(translation.getX(), translation.getY(), 
                                                translation.getZ(), fx, fy, cx, cy);
                  
                  // X-axis point (red)
                  Vector3d xAxis = rotatePoint(axisLength, 0, 0, rotation);
                  Point xPoint = project3DPoint(translation.getX() + xAxis.getX(), 
                                                translation.getY() + xAxis.getY(),
                                                translation.getZ() + xAxis.getZ(), 
                                                fx, fy, cx, cy);
                  Imgproc.line(mat, origin, xPoint, redColor, 3);
                  
                  // Y-axis point (green)
                  Vector3d yAxis = rotatePoint(0, axisLength, 0, rotation);
                  Point yPoint = project3DPoint(translation.getX() + yAxis.getX(),
                                                translation.getY() + yAxis.getY(),
                                                translation.getZ() + yAxis.getZ(),
                                                fx, fy, cx, cy);
                  Imgproc.line(mat, origin, yPoint, greenColor, 3);
                  
                  // Z-axis point (blue)
                  Vector3d zAxis = rotatePoint(0, 0, axisLength, rotation);
                  Point zPoint = project3DPoint(translation.getX() + zAxis.getX(),
                                                translation.getY() + zAxis.getY(),
                                                translation.getZ() + zAxis.getZ(),
                                                fx, fy, cx, cy);
                  Imgproc.line(mat, origin, zPoint, blueColor, 3);
                }
                
                // Draw detection count
                String countText = "Tags: " + detections.length;
                Imgproc.putText(mat, countText, new Point(10, 30),
                                Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, greenColor, 2);
                
                // Send processed frame to dashboard
                outputStream.putFrame(mat);
              }
              
              // Clean up detector when thread ends
              detector.close();
              grayMat.release();
              mat.release();
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }
  
  // Helper method to project 3D point to 2D image coordinates
  private Point project3DPoint(double x, double y, double z, double fx, double fy, double cx, double cy) {
    // Simple pinhole camera projection
    double u = fx * (x / z) + cx;
    double v = fy * (y / z) + cy;
    return new Point(u, v);
  }
  
  // Helper method to rotate a point by a rotation
  private Vector3d rotatePoint(double x, double y, double z, Rotation3d rotation) {
    // Convert to quaternion and rotate
    Quaternion q = rotation.getQuaternion();
    
    // Quaternion rotation: v' = q * v * q^-1
    // For simplicity, using rotation matrix
    double[][] rotMatrix = new double[3][3];
    
    // Convert quaternion to rotation matrix
    double w = q.getW();
    double qx = q.getX();
    double qy = q.getY();
    double qz = q.getZ();
    
    rotMatrix[0][0] = 1 - 2*qy*qy - 2*qz*qz;
    rotMatrix[0][1] = 2*qx*qy - 2*qz*w;
    rotMatrix[0][2] = 2*qx*qz + 2*qy*w;
    rotMatrix[1][0] = 2*qx*qy + 2*qz*w;
    rotMatrix[1][1] = 1 - 2*qx*qx - 2*qz*qz;
    rotMatrix[1][2] = 2*qy*qz - 2*qx*w;
    rotMatrix[2][0] = 2*qx*qz - 2*qy*w;
    rotMatrix[2][1] = 2*qy*qz + 2*qx*w;
    rotMatrix[2][2] = 1 - 2*qx*qx - 2*qy*qy;
    
    // Apply rotation
    double rx = rotMatrix[0][0]*x + rotMatrix[0][1]*y + rotMatrix[0][2]*z;
    double ry = rotMatrix[1][0]*x + rotMatrix[1][1]*y + rotMatrix[1][2]*z;
    double rz = rotMatrix[2][0]*x + rotMatrix[2][1]*y + rotMatrix[2][2]*z;
    
    return new Vector3d(rx, ry, rz);
  }
  
  // Simple 3D vector class for helper calculations
  private static class Vector3d {
    private final double x, y, z;
    
    public Vector3d(double x, double y, double z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
    
    public double getX() { return x; }
    public double getY() { return y; }
    public double getZ() { return z; }
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    startImageProcThread();

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);

    //start navX
    navx = new AHRS(SPI.Port.kMXP);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //print out navx values for testing purposes
    SmartDashboard.putBoolean("NavX Connected", navx.isConnected());
    SmartDashboard.putBoolean("NavX Calibrating", navx.isCalibrating());
    SmartDashboard.putNumber("NavX Yaw", navx.getYaw());
    SmartDashboard.putNumber("NavX Pitch", navx.getPitch());
    SmartDashboard.putNumber("NavX Roll", navx.getRoll());

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

 