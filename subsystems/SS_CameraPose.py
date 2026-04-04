import commands2
import wpilib
from wpimath.geometry import Pose3d, Pose2d, Transform3d, Translation3d, Rotation3d
from wpilib import SmartDashboard, Timer
from photonlibpy import PhotonCamera, PhotonPoseEstimator
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

class SS_CameraPose(commands2.Subsystem):


    def __init__(self, swerve_drive):
        super().__init__()
        self.swerve_drive = swerve_drive
        if not wpilib.RobotBase.isReal():
            print("PhotonVision disabled in simulation")
            self.rightcam = None
            self.leftcam = None
            self.estimator = None
            return
        # load load field 
        self.field_layout = AprilTagFieldLayout.loadField(AprilTagField.kDefaultField)

        # set camera
        self.rightcam = PhotonCamera("RightCamera")
        #self.leftcam = PhotonCamera("LeftCamera") # unused for now, but we can add it later if we want

        # we need to measure this 
        #self.robot_to_leftcam = Transform3d(
            #Translation3d(0.3048, -0.3302, 0.4826), # meters forward, right, up from robot center
            #Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        #)
        self.robot_to_rightcam = Transform3d(
            Translation3d(0.3048, 0.3302, 0.4826), # meters forward, left, up from robot center
            Rotation3d(0.0, 0.0, 0.0) # radians roll, pitch, yaw from robot forward
        )

        # pos estimation
        self.estimator = PhotonPoseEstimator(
            self.field_layout,
            self.robot_to_rightcam,
            #self.robot_to_leftcam
        )

    def periodic(self):
        # grab cams 
        if self.rightcam is None:
            return
        results = self.rightcam.getAllUnreadResults()
        if not results:
            return

        result = results[-1]
        if not result.hasTargets():
            return

        # calculate pos based on cams (use one of these)
        #est = self.estimator.estimatePnpDistanceTrigSolvePose(result)
        #est = self.estimator.estimateCoprocMultiTagPose(result)
        est = self.estimator.estimateLowestAmbiguityPose(result)

        if est is None:
            return

        # send to swerve
        self.swerve_drive.drivetrain.add_vision_measurement(
            est.estimatedPose.toPose2d(),
            est.timestampSeconds,
            vision_measurement_std_devs = [.1, .1, 5.0] # distrust of x, y in meters, heading in degrees
        )

        # Dashboard
        pose = est.estimatedPose
        SmartDashboard.putNumber("Vision/X", pose.X())
        SmartDashboard.putNumber("Vision/Y", pose.Y())
        SmartDashboard.putNumber("Vision/Heading",
                                 pose.rotation().toRotation2d().degrees())