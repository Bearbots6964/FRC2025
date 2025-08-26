package frc.robot.util.annotation

/**
 * Annotation to mark a class for AdvantageKit logging.
 * Classes annotated with this will have a generated subclass that implements
 * logging functionality, allowing their properties to be logged and restored.
 */
@Target(AnnotationTarget.CLASS) annotation class Logged
