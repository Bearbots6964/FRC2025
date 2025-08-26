package frc.robot.util.annotation

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.KSAnnotated
import com.google.devtools.ksp.symbol.KSClassDeclaration
import com.google.devtools.ksp.symbol.Modifier
import com.google.devtools.ksp.validate
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.KModifier
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.ksp.toClassName
import com.squareup.kotlinpoet.ksp.writeTo
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

/**
 * Symbol processor that generates a subclass for each class annotated with @Logged.
 * The generated subclass implements [LoggableInputs] and provides automatic logging
 * and deserialization of all mutable properties using LogTable.
 *
 * @property codeGenerator The code generator used to write generated files.
 */
class LoggedProcessor(private val codeGenerator: CodeGenerator) : SymbolProcessor {
  private val logTableType = LogTable::class
  private val loggableInputsType = LoggableInputs::class

  /**
   * Processes all classes annotated with @Logged.
   * For each valid class, generates a subclass that implements logging functionality.
   *
   * @param resolver The resolver to find annotated symbols.
   * @return List of symbols that could not be validated.
   */
  override fun process(resolver: Resolver): List<KSAnnotated> {
    val annotatedClasses =
      resolver.getSymbolsWithAnnotation("frc.robot.util.annotation.Logged")
        .filterIsInstance<KSClassDeclaration>()
    annotatedClasses.forEach { process(it) }
    return annotatedClasses.filterNot { it.validate() }.toList()
  }

  /**
   * Generates a subclass for the given class declaration that implements [LoggableInputs].
   * The generated class overrides toLog and fromLog to serialize/deserialize all mutable properties.
   *
   * @param classDeclaration The class declaration to process.
   * @throws IllegalStateException if the class is not open or has non-mutable properties.
   */
  private fun process(classDeclaration: KSClassDeclaration) {
    if (!classDeclaration.modifiers.contains(Modifier.OPEN))
      throw IllegalStateException(
        """[Logged] Please ensure the class you are annotating (${classDeclaration.simpleName.asString()}) has the open modifier!"""
      )

    val packageName = classDeclaration.packageName.asString()
    val className = classDeclaration.simpleName.asString()

    val newClassName = "Logged${className}"

    val toLogBuilder =
      FunSpec.builder("toLog").addModifiers(KModifier.OVERRIDE).addParameter("table", logTableType)
    val fromLogBuilder =
      FunSpec.builder("fromLog").addModifiers(KModifier.OVERRIDE)
        .addParameter("table", logTableType)

    classDeclaration.getAllProperties().forEach { property ->
      val simpleName = property.simpleName.asString()
      val logName = simpleName.substring(0, 1).uppercase() + simpleName.substring(1)

      if (!property.isMutable)
        throw IllegalStateException(
          """[Logged] Please ensure the class you are annotating (${classDeclaration.simpleName.asString()}) has only mutable properties!"""
        )

      toLogBuilder.addCode(
        """ |table.kPut("$logName", $simpleName)
                    |
                """
          .trimMargin()
      )

      fromLogBuilder.addCode(
        """ |$simpleName = table.kGet("$logName", $simpleName)
                    |
                """
          .trimMargin()
      )
    }

    val type =
      TypeSpec.classBuilder(newClassName)
        .addSuperinterface(loggableInputsType)
        .superclass(classDeclaration.toClassName())
        .addFunction(toLogBuilder.build())
        .addFunction(fromLogBuilder.build())

    val file = FileSpec.builder(packageName, newClassName)
    file.addType(type.build())
    file.indent("    ")
    file.addImport(LogTableUtils::class, "kGet", "kPut")
    file.build().writeTo(codeGenerator, Dependencies(true, classDeclaration.containingFile!!))
  }
}

/**
 * Provider for the [LoggedProcessor] symbol processor.
 * Registers the processor with the KSP environment.
 */
class Provider : SymbolProcessorProvider {
  /**
   * Creates a new [LoggedProcessor] using the provided environment.
   *
   * @param environment The symbol processor environment.
   * @return A new instance of [LoggedProcessor].
   */
  override fun create(environment: SymbolProcessorEnvironment) =
    LoggedProcessor(codeGenerator = environment.codeGenerator)
}