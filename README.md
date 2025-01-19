# **FRC Team #8016: 2025 FIRST REEFSCAPE Robot Code**

## **Setup:**

To properly set up your development environment for the 2025 season, you must install VSCode and WPILib extensions (found [here](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)) and git (found [here](https://git-scm.com/)).

## **Repo Overview**

The main branch contains the development robot source code. After each event, a branch ``competition/<event_name>`` is created from main to preserve the software run at the respective event.

### **Contributing**

You must be a member of FRC team 8016 to contribute. To be added to the 8016 GitHub organization, please contact Caroline or Gio. To contribute, fork the repository and make your changes locally. Changes should be submitted as a Pull Request against alpha. Use clear, concise commit messages as outlined in [this](https://cbea.ms/git-commit/#seven-rules) guide.

## **Code formatting:**

This project uses modified Google style guides for all source code, which are enforced by the Spotless autoformatter.

By default, ``./gradlew spotlesscheck`` runs while building the project; generally, any convention or formatting errors reported by Spotless can be resolved by running ``./gradlew spotlessApply`` from the root directory.

Please note the following conventions:

- Classes should be in UpperCamelCase
- Methods should be in lowerCamelCase
- Variables should be concise but descriptive, and written in lowerCamelCase
- Constants should be kept solely in the Constants class, and should be written in UPPERCASE_SEPERATED_BY_UNDERSCORES
- When you create a method, write a comment telling us what the method does, and if necessary, how it does it.
- When you create a constant, include units either in the name or in a comment to the right of the constant declaration.
- Bracketing: follow the conventions below:
```
public void ExampleMethod() {
    if(x == 1) {
        //Do something
    } else {
        //Do something else
    }
}
```
