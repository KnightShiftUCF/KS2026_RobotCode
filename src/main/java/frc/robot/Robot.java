// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
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
              detector.addFamily("tag41h11");
              
              // Configure detector for reasonable performance on RoboRIO 2
              AprilTagDetector.Config config = new AprilTagDetector.Config();
              config.numThreads = 4;
              config.quadDecimate = 2.0f;
              config.quadSigma = 0.0f;
              config.refineEdges = true;
              config.decodeSharpening = 0.25;
              detector.setConfig(config);
              
              // Colors for drawing
              Scalar greenColor = new Scalar(0, 255, 0);
              Scalar redColor = new Scalar(0, 0, 255);
              Scalar blueColor = new Scalar(255, 0, 0);
              
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

 