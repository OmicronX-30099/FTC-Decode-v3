package org.firstinspires.ftc.teamcode.Enums

import com.pedropathing.geometry.Pose
import dev.nextftc.units.Measure
import dev.nextftc.units.unittypes.DistanceUnit
import dev.nextftc.units.unittypes.Inches

enum class Alliance {
    RED {
        override val goalPose: Pose = Pose(this.fieldWidth.magnitude, this.fieldLength.magnitude)
        override val turretTargetPose: Pose = Pose(this.goalPose.x - this.turretAimOffsetX.magnitude, this.goalPose.y - this.turretAimOffsetY.magnitude)

        override val firstResetPose: Pose = Pose(0.0,0.0)
        override val secondResetPose: Pose = Pose(0.0,0.0)
        override val thirdResetPoes: Pose = Pose(0.0,0.0)
        override val fourthResetPose: Pose = Pose(0.0,0.0)
        override val fifthResetPose: Pose = Pose(0.0,0.0)
    },
    BLUE {
        override val goalPose: Pose = Pose(0.0, this.fieldLength.magnitude)
        override val turretTargetPose: Pose = Pose(this.goalPose.x + this.turretAimOffsetX.magnitude, this.goalPose.y - this.turretAimOffsetY.magnitude)

        override val firstResetPose: Pose = Pose(0.0,0.0)
        override val secondResetPose: Pose = Pose(0.0,0.0)
        override val thirdResetPoes: Pose = Pose(0.0,0.0)
        override val fourthResetPose: Pose = Pose(0.0,0.0)
        override val fifthResetPose: Pose = Pose(0.0,0.0)
    };

    abstract val goalPose: Pose
    abstract val turretTargetPose: Pose

    abstract val firstResetPose: Pose
    abstract val secondResetPose: Pose
    abstract val thirdResetPoes: Pose
    abstract val fourthResetPose: Pose
    abstract val fifthResetPose: Pose

    open val fieldWidth: Measure<DistanceUnit> = Inches.of(141.5)
    open val fieldLength: Measure<DistanceUnit> = Inches.of(141.5)
    open val turretAimOffsetX: Measure<DistanceUnit> = Inches.of(1.0)
    open val turretAimOffsetY: Measure<DistanceUnit> = Inches.of(0.0)
}