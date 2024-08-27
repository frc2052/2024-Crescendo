# 2024-Crescendo
Code for Team 2052's 2024 robot, Scarab.

Previous code:
- Code from [2023](https://github.com/frc2052/2023-ChargedUp) and [2022](https://github.com/frc2052/2022-RapidReact).

![Robot Image](/images/Scarab.JPG)

## Highlights
- Drivetrain

  Scarab's SDS MK4i swerve drivetrain utilizes our [custom swerve library](/src/main/java/com/team2052/swervemodule/) to smoothly drive our Kraken drive and Neo steer setup.

- RobotState

    [RobotState.java](/src/main/java/frc/robot/RobotState.java) serves as a central hub for robot-wide variables, such as the [current robot pose](/src/main/java/frc/robot/RobotState.java#L275).

- AprilTags

  Using a dual Orange Pi 5 loaded with PhotonVision and four Arducam OV9281, Scarab continuously polls each camera [in parallel](/src/main/java/frc/robot/subsystems/AprilTagSubsystem.java#L80). This data is fed into [RobotState](/src/main/java/frc/robot/RobotState.java#L97), where the vision updates are filtered and thrown away if outside the field border or the pose ambiguity is too high. Once filtered, this data is processed by the [RobotStateEstimator](/src/main/java/frc/robot/util/RobotStateEstimator.java).

- RobotStateEstimator

  [RobotStateEstimator.java](/src/main/java/frc/robot/util/RobotStateEstimator.java) utilizes WPILib's [SwerveDrivePoseEstimator](https://first.wpi.edu/wpilib/allwpilib/docs/release/java/edu/wpi/first/math/estimator/SwerveDrivePoseEstimator.html) to combine the odometry and vision updates. While [processing the individual vision updates](src/main/java/frc/robot/util/RobotStateEstimator.java#L67) from each camera, we dynamically adjust the vision measurement standard deviations based on how much we trust the tag. Only when both speaker tags are seen do we trust the rotational value of the vision pose.

- PathPlanner & AutoFactory

  This year we decided to try [PathPlanner](https://pathplanner.dev/home.html) for our autos. Once the paths and autos have been created in the PathPlanner application, we add the autonomous modes to [AutoFactory.java](/src/main/java/frc/robot/auto/AutoFactory.java), which then is used to select and [precompile](/src/main/java/frc/robot/auto/AutoFactory.java#L45) a chosen auto before each match.
