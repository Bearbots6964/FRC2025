package frc.robot.automation.drivebase.requests

class DriveBackUpRequest(val speed: Speed = Speed.FAST) : DriveRequest

enum class Speed {
    NORMAL,
    FAST,
    FORWARD
}