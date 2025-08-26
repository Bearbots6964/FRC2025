private fun runOneFullCoralCycle(): Command {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return sequence(
            goToCoralStation(),
            // wait until the driver signals a coral on the intake (we have no way of detecting this)
            lockWheelsAndWaitForInput(),
            runOnce({ state.push(task = AutoTask.TO_REEF) }),
            // pathfind to reef; requires drive, elevator, arm, intake, climber
            parallel(
                // this command sequence concerns the drivebase + pathfinding
                sequence(
                    // pathfinding speed; doesn't require anything
                    runOnce({
                        drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                        inPosition = false
                    }),


                    // actually go to the reef
                    pathfindToReef().deadlineFor(
                        // wait until the claw has the coral secured;
                        // if we move too fast,
                        // we risk throwing it off the intake,
                        // but we still want to be able to move quickly
                        waitUntil { state.coralStatus == CoralStatus.IN_CLAW }.andThen({
                            // set pathfinding speed to the normal speed
                            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                        })
                    ),

                    // at this point we're in the reef protected zone
                    // which should provide reprieve
                    // if the opposing alliance is playing defense
                    // and knocks us out of alignment.
                    // more importantly, we're stopped and waiting

                    // wait until the superstructure is in the position
                    // we want it to be in for scoring
                    waitUntil { inPosition }
                        // while we wait, lock the wheels
                        // this might actually help
                        // mitigate some of the "getting knocked out of alignment"
                        // issues we could face in a real match
                        .deadlineFor(run({ drive.stopWithX() }, drive)).andThen(
                            // close that last bit of distance
                            finalReefLineup()
                        ).withName("Pathfind to Reef (final)")
                ), // end drivebase sequence

                // wait until the claw has the coral secured and set that state
                waitUntil { clawIntake.grabbed }.andThen({
                    state.coralStatus = CoralStatus.IN_CLAW
                }),

                // superstructure stuff
                sequence(
                    // pick up the coral from the intake
                    autoHandler.accept(
                        SuperstructureRequest(
                            armPosition = Constants.SuperstructureConstants.SuperstructureState.CORAL_PICKUP.armPosition,
                            elevatorPosition = Constants.SuperstructureConstants.SuperstructureState.CORAL_PICKUP.elevatorPosition,
                            climberPosition = Constants.SuperstructureConstants.CLIMBER_POSITION_EXTENDED,
                        )
                    )
                        // i couldn't imagine a situation
                        // in which the coral wouldn't be on the intake,
                        // because this command can already only start
                        // once the driver confirms it,
                        // but this check could be useful at some point or another
                        .onlyIf { state.coralStatus == CoralStatus.ON_INTAKE },

                    // might fix an issue we were having
                    // where the arm tries to go to some random position
                    runOnce({ arm.setGoalToCurrent() }),

                    // wait until we're near where we need to be
                    waitUntil(drive::nearGoal).andThen(
                        // defer this command construction until it's called.
                        // that way,
                        // we're checking what position
                        // the drivers want the superstructure
                        // to be in at the last possible moment
                        // in case they change something.
                        // the field is dynamic, after all
                        runOnce({ updateHmi() })
                    ).andThen(
                        defer({
                            // go to the position
                            autoHandler.accept(
                                SuperstructureRequest(
                                    armPosition = nextPosition.armPosition,
                                    elevatorPosition = nextPosition.elevatorPosition,
                                    climberPosition = nextPosition.climberPosition
                                )
                            )
                        }, setOf(elevator, arm, climber))
                    ),
                    // once we're in position,
                    // set the state so the drivebase can continue its final lineup
                    runOnce({ inPosition = true })
                )
            ), // end pathfinding to reef and lining up and all that jazz


            // deferred command, for the same reasons as the other one
            defer({
                // score the coral
                autoHandler.accept(
                    SuperstructureRequest(
                        armPosition = nextPosition.armPosition,
                        elevatorPosition = nextPosition.elevatorPosition,
                        climberPosition = nextPosition.climberPosition,
                        intakeSpeed = IntakeState.OUTTAKE
                    )
                )
            }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                // set state of the coral
                Runnable { state.coralStatus = CoralStatus.NONE })
                // only do this deferred command + compositions
                // if we actually have the coral
                .onlyIf { state.coralStatus == CoralStatus.IN_CLAW }).andThen(
            select(
                mapOf(
                    Pair(
                        true, runOnce({ runOneFullCoralCycle().schedule() })
                    ), Pair(
                        false, none()
                    )
                ), ::keepGoing
            )
        ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Full Coral Cycle")

    }

    private fun justScore(): Command {
        var inPosition = false
        return parallel(
            runOnce({ state.push(task = AutoTask.TO_REEF) }),
            // this command sequence concerns the drivebase + pathfinding
            sequence(
                // pathfinding speed; doesn't require anything
                runOnce({
                    drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                    inPosition = false
                }),


                // actually go to the reef
                pathfindToReef(),

                // at this point we're in the reef protected zone
                // which should provide reprieve
                // if the opposing alliance is playing defense
                // and knocks us out of alignment.
                // more importantly, we're stopped and waiting

                // wait until the superstructure is in the position
                // we want it to be in for scoring
                waitUntil { inPosition }
                    // while we wait, lock the wheels
                    // this might actually help
                    // mitigate some of the "getting knocked out of alignment"
                    // issues we could face in a real match
                    .deadlineFor(run({ drive.stopWithX() }, drive)).andThen(
                        // close that last bit of distance
                        finalReefLineup()
                    ).withName("Pathfind to Reef (final)")
            ), // end drivebase sequence

            // superstructure stuff
            sequence(
                // might fix an issue we were having
                // where the arm tries to go to some random position
                runOnce({ arm.setGoalToCurrent() }),

                // wait until we're near where we need to be
                waitUntil(drive::nearGoal).andThen(
                    // defer this command construction until it's called.
                    // that way,
                    // we're checking what position
                    // the drivers want the superstructure
                    // to be in at the last possible moment
                    // in case they change something.
                    // the field is dynamic, after all
                    defer({
                        // go to the position
                        autoHandler.accept(
                            SuperstructureRequest(
                                armPosition = nextPosition.armPosition,
                                elevatorPosition = nextPosition.elevatorPosition,
                                climberPosition = nextPosition.climberPosition
                            )
                        )
                    }, setOf(elevator, arm, climber))
                ),
                // once we're in position,
                // set the state so the drivebase can continue its final lineup
                runOnce({ inPosition = true })
            )
        ) // end pathfinding to reef and lining up and all that jazz

            .andThen(
                // deferred command, for the same reasons as the other one
                defer({
                    // score the coral
                    autoHandler.accept(
                        SuperstructureRequest(
                            armPosition = nextPosition.armPosition,
                            elevatorPosition = nextPosition.elevatorPosition,
                            climberPosition = nextPosition.climberPosition,
                            intakeSpeed = IntakeState.OUTTAKE
                        )
                    )
                }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                    // set state of the coral
                    Runnable { state.coralStatus = CoralStatus.NONE })
            ).finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Score Coral")

    }

    private fun runOneFullCoralCycleButWaitTime(): Command {
        var inPosition = false
        // go to coral station; requires drive, arm, elevator, and climber
        return sequence(
            goToCoralStation(),

            // wait for some arbitrary amount of time defined elsewhere
            lockWheelsAndWaitTime(), runOnce({ state.push(task = AutoTask.TO_REEF) }),

            // pathfind to reef; requires drive, elevator, arm, intake, climber
            parallel(

                // this command sequence concerns the drivebase + pathfinding
                sequence(
                    // pathfinding speed; doesn't require anything
                    runOnce({
                        drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.coralIntakeSpeed)
                        inPosition = false
                    }),


                    // actually go to the reef
                    finalReefLineup(), run({ drive.stopWithX() }, drive).withTimeout(0.1)
                ), // end drivebase sequence

                // wait until the claw has the coral secured and set that state
                waitUntil { clawIntake.grabbed }.withTimeout(3.0).andThen({
                    state.coralStatus = CoralStatus.IN_CLAW
                    drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
                }),

                // superstructure stuff
                sequence(
                    // pick up the coral from the intake
                    autoHandler.accept(
                        SuperstructureRequest(
                            armPosition = Constants.SuperstructureConstants.SuperstructureState.CORAL_PICKUP.armPosition,
                            elevatorPosition = Constants.SuperstructureConstants.SuperstructureState.CORAL_PICKUP.elevatorPosition,
                            climberPosition = Constants.SuperstructureConstants.CLIMBER_POSITION_EXTENDED,
                        )
                    )
                        // i couldn't imagine a situation
                        // in which the coral wouldn't be on the intake,
                        // because this command can already only start
                        // once the driver confirms it,
                        // but this check could be useful at some point or another
                        .onlyIf { state.coralStatus == CoralStatus.ON_INTAKE }.withTimeout(3.0),

                    // might fix an issue we were having
                    // where the arm tries to go to some random position
                    runOnce({ arm.setGoalToCurrent() }),

                    // wait until we're near where we need to be
                    // defer this command construction until it's called.
                    // that way,
                    // we're checking what position
                    // the drivers want the superstructure
                    // to be in at the last possible moment
                    // in case they change something.
                    // the field is dynamic, after all
                    defer({
                        // go to the position
                        autoHandler.accept(
                            SuperstructureRequest(
                                armPosition = nextPosition.armPosition,
                                elevatorPosition = nextPosition.elevatorPosition,
                                climberPosition = nextPosition.climberPosition
                            )
                        )
                    }, setOf(elevator, arm, climber)),
                    // once we're in position,
                    // set the state so the drivebase can continue its final lineup
                    runOnce({ inPosition = true })
                )
            ), // end pathfinding to reef and lining up and all that jazz

            // deferred command, for the same reasons as the other one
            defer({
                // score the coral
                autoHandler.accept(
                    SuperstructureRequest(
                        armPosition = nextPosition.armPosition,
                        elevatorPosition = nextPosition.elevatorPosition,
                        climberPosition = nextPosition.climberPosition,
                        intakeSpeed = IntakeState.OUTTAKE
                    )
                )
            }, setOf(elevator, arm, clawIntake, drive)).finallyDo(
                // set state of the coral
                Runnable { state.coralStatus = CoralStatus.NONE })
                // only do this deferred command + compositions
                // if we actually have the coral
                .onlyIf { state.coralStatus == CoralStatus.IN_CLAW }).finallyDo(Runnable {
            state.push(
                task = AutoTask.IDLE
            )
        })
            .withName("[AUTO] Full Coral Cycle")
    }

    private fun algaeCycle(): Command {

        return runOnce({
            // check if we have anything selected for barge positions.
            // if not, default to none
            bargePosition = bargeChooser.get() ?: BargePosition.NONE
            state.push(task = AutoTask.TO_ALGAE)
        }).andThen(
            sequence(
                sequence(
                    sequence( // prepare to grab algae
                        defer( // pathfind to reef
                            {
                                // pathfind to where we need to be,
                                // just 20 inches back so we have room to swing the arm around
                                PathfindingFactories.pathfindToReefButBackALittleMore(
                                    drive, { nextAlgaePosition }, driveTranslationalControlSupplier
                                )
                            }, setOf(drive)
                        ),
                        // wait until we're near the target reef,
                        // then put the superstructure in whatever position we need to be in to grab algae

                        defer( // move superstructure to algae grab position
                            {
                                superstructureCommands.goToPosition(
                                    elevator, arm, climber, when (nextAlgaePosition) {
                                        PathfindingFactories.Reef.AB_ALGAE, PathfindingFactories.Reef.EF_ALGAE, PathfindingFactories.Reef.IJ_ALGAE -> Constants.SuperstructureConstants.SuperstructureState.UPPER_REEF_ALGAE
                                        else -> Constants.SuperstructureConstants.SuperstructureState.LOWER_REEF_ALGAE
                                    }
                                )
                            }, setOf(arm, elevator, climber)
                        ).deadlineFor(
                            run({ drive.stopWithX() }, drive)
                        )
                    ),

                    // set the pathfinding speed
                    // to be slower so we're not slamming into the reef at top speed
                    runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.algaeGrabSpeed) }),

                    // pathfind forward so we can actually pick the algae up
                    sequence(
                        PathfindingFactories.pathfindToReefButBackALittleLess(
                            drive, { nextAlgaePosition }, driveTranslationalControlSupplier
                        ),
                        run(
                            { drive.stopWithX() }, drive
                        )
                    )
                        // lock the wheels
                        .withDeadline(
                            // spin the intake
                            clawIntake.intakeWithoutStoppingForAlgae().withDeadline(
                                // wait until the motor stalls
                                // (surefire way to tell if we have an algae) and then wait an extra half-second
                                // FIXME: check if this value can go lower
                                // update 8/5/25: turned down by half
                                waitUntil(clawIntake::grabbed).andThen(waitSeconds(0.25))
                                // set algae state
                            ).andThen({ algaeStatus = AlgaeStatus.IN_CLAW })
                        ),
                    // back up
                    drive.backUp()
                ).onlyIf { algaeStatus == AlgaeStatus.NONE }, // only do this if we don't have algae
                // decide what to do based on the barge position selection
                select(
                    mapOf(
                        BargePosition.NONE to spitOutAlgae(),
                        BargePosition.LEFT to putAlgaeInBarge(),
                        BargePosition.MIDDLE to putAlgaeInBarge(),
                        BargePosition.RIGHT to putAlgaeInBarge()
                    )
                ) { bargePosition }.onlyIf { algaeStatus == AlgaeStatus.IN_CLAW } // use the barge position as the key
            )).finallyDo(Runnable { state.push(task = AutoTask.IDLE) })
    }

    // </editor-fold>
    // <editor-fold desc="Larger utility functions">

    private fun putAlgaeInBarge(): Command {
        return sequence(
            runOnce({ state.push(task = AutoTask.TO_BARGE) }),
            // set pathfinding speed to whatever we have set for going to the barge
            runOnce({ drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toBargeSpeed) }),

            // pathfind to the barge
            defer({
                PathfindingFactories.pathfindToPosition(
                    drive,
                    Constants.PathfindingConstants.getPosition(bargePosition),
                    driveTranslationalControlSupplier
                )
            }, setOf(drive)).alongWith(
                // wait until we're near the barge
                waitSeconds(0.5).andThen(waitUntil(drive::lessNearGoal)).andThen(
                    // and then put the superstructure in the right position
                    autoHandler.accept(
                        SuperstructureRequest(
                            armPosition = Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH.armPosition,
                            elevatorPosition = Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH.elevatorPosition,
                            climberPosition = Constants.SuperstructureConstants.SuperstructureState.BARGE_LAUNCH.climberPosition
                        )
                    )
                )
                // while this is happening, spin the intake so we can keep the algae in
            ).deadlineFor(clawIntake.intakeWithoutStoppingForAlgae()),

            // drive forward while spitting the algae out.
            // the drive command ends after half a second
            drive.goForward().deadlineFor(
                clawIntake.outtakeMaxSpeed()
            ).andThen(
                // stop the intake and set algae status to none
                runOnce({
                    algaeStatus = AlgaeStatus.NONE
                }).alongWith(clawIntake.stopOnce())
            ),

            // back up a little bit
            drive.backUp(),

            // stop the drive
            // while we put the superstructure back into the position it needs to be in
            superstructureCommands.preCoralPickup(elevator, arm, climber).deadlineFor(
                run({ drive.stopWithX() }, drive)
            ).withTimeout(3.0)
        ).finallyDo(Runnable {
            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed); state.push(
            task = AutoTask.IDLE
        )
        }).withName("Put Algae in Barge")
    }

    private fun spitOutAlgae(): Command {
        return sequence(
            runOnce({ state.push(task = AutoTask.SPIT_OUT_ALGAE) }),
            // rotate 180°, but stop after half a second
            DriveCommands.joystickDriveAtAngle(
                drive,
                { 0.0 },
                { 0.0 },
                { drive.pose.rotation.rotateBy(Rotation2d(Math.PI)) })
                .withDeadline(waitSeconds(0.5)),
            // spin the intake to spit out algae
            // no way to determine when this is done so just do it for 3/4 of a second
            run({ drive.stopWithX() }, drive).alongWith(
                clawIntake.outtakeFaster()
            ).withDeadline(
                waitSeconds(0.75)
            ),
            // set algae status to none, stop intake
            runOnce({
                algaeStatus = AlgaeStatus.NONE
            }).alongWith(clawIntake.stopOnce())
        ).finallyDo(Runnable {
            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed); state.push(
            task = AutoTask.IDLE
        )
        })
    }
    // </editor-fold>
    // <editor-fold desc="Low-level functions">

    private fun lockWheelsAndWaitForInput(): Command = DriveCommands.joystickDrive(
        drive,
        { -driveController.leftY },
        { -driveController.leftX },
        { -driveController.rightX }).withDeadline(
        waitUntil { state.coralStatus == CoralStatus.ON_INTAKE })
        .alongWith(runOnce({ state.push(task = AutoTask.WAITING) }).finallyDo(Runnable {
            state.push(
                task = AutoTask.IDLE
            )
        }))
        .withName("Lock Wheels")


    private fun lockWheelsAndWaitTime(): Command = run(
        { drive.stopWithX() }, drive
    ).alongWith(climber.moveClimberToIntakePosition()).withDeadline(
        waitSeconds(Constants.PathfindingConstants.benCompensation).andThen({
            state.coralStatus = CoralStatus.ON_INTAKE
        }).alongWith(runOnce({ state.push(task = AutoTask.WAITING) }))
            .finallyDo(Runnable { state.push(task = AutoTask.IDLE) }).withName("Lock Wheels")
    )

    private fun finalReefLineup(): Command = defer({
        PathfindingFactories.pathfindToReef(
            drive, { state.nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive)).withName("Pathfind to Reef")

    private fun pathfindToReef(): Command = defer({
        PathfindingFactories.pathfindToReefButBackALittle(
            drive, { state.nextReef }, driveTranslationalControlSupplier
        )
    }, setOf(drive))

    private fun goToCoralStation(): Command =
        runOnce({
            drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toCoralStationSpeed); state.push(
            task = AutoTask.TO_CORAL_STATION
        )
        }).andThen(
            defer(
                {
                    PathfindingFactories.pathfindToCoralStation(
                        drive, { state.nextStation }, driveTranslationalControlSupplier
                    )
                }, setOf(drive)
            ).deadlineFor(
                superstructureCommands.preCoralPickupWithoutSafety(elevator, arm)
                    .alongWith(climber.moveClimberToIntakePosition())
            )
        ).finallyDo(Runnable {
            arm.setGoalToCurrent(); drive.setPathfindingSpeedPercent(Constants.PathfindingConstants.toReefSpeed)
            state.push(task = AutoTask.IDLE)
        }).onlyIf { state.coralStatus == CoralStatus.NONE }.withName("Pathfind to Coral Station")
