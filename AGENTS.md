# AGENTS.md

## Purpose
- This repo is STM32F407 firmware for the `Infantry_01` target.
- It uses C11, CMake, Ninja, and the STM32 `arm-none-eabi-gcc` toolchain.
- Handwritten logic mostly lives in `Application/`, `ALL_Task/`, `Bsp/`, `Components/`, and `Algorithm/`.
- Generated STM32CubeMX/HAL/USB/FreeRTOS code mostly lives in `Core/`, `USB_DEVICE/`, `Drivers/`, `Middlewares/`, and `cmake/stm32cubemx/`.

## Rules Audit
- Checked `.cursor/rules/`: not present.
- Checked `.cursorrules`: not present.
- Checked `.github/copilot-instructions.md`: not present.
- No prior `AGENTS.md` existed in this repository root.

## Build System
- Root build file: `CMakeLists.txt`.
- Presets file: `CMakePresets.json`.
- Generator: `Ninja`.
- Toolchain file: `cmake/gcc-arm-none-eabi.cmake`.
- Main target: `Infantry_01`.
- Main artifact: `build/<preset>/Infantry_01.elf`.
- `compile_commands.json` is generated in `build/<preset>/`.
- `.clangd` is configured to use `build/Debug` as the compilation database.

## Canonical Commands

### Configure
```bash
cmake --preset Debug
cmake --preset Release
```

### Build
```bash
cmake --build --preset Debug
cmake --build --preset Release
```

### Reconfigure + Build
```bash
cmake --preset Debug && cmake --build --preset Debug
```

### Clean
```bash
cmake --build build/Debug --target clean
cmake --build build/Release --target clean
```

### Discover Presets / Targets
```bash
cmake --list-presets
ninja -C build/Debug -t targets
```

## Test / Lint Reality
- No unit-test framework is configured in this repo.
- `CTest` is not enabled; no `enable_testing()` or `add_test()` calls were found.
- No dedicated lint config was found for `clang-format`, `clang-tidy`, `cppcheck`, or `uncrustify`.
- The practical baseline validation step is a normal Debug build.

## Single Test Guidance
- There is no true "single test" command because no automated tests exist.
- If asked to run one test, say so explicitly and use targeted compilation as the closest substitute.
- The most useful focused check is rebuilding one object file for the changed source.

### Example: Rebuild One Source File
```bash
cmake --build build/Debug --target "CMakeFiles/Infantry_01.dir/Bsp/uart/bsp_uart.c.obj"
```

### How To Find An Object Target
- Look in `build/Debug/build.ninja`.
- Or inspect `build/Debug/compile_commands.json` and convert the output path into a target name.
- Use targeted rebuilds for quick compile checks, not as a replacement for full firmware validation.

## Current Build Status
- `cmake --build --preset Debug` currently fails in `Bsp/uart/bsp_uart.c`.
- The main hard error is passing `&pData->rxdatas` where HAL expects `uint8_t *`.
- The current branch also emits several warnings for signedness and unused parameters/variables.
- Do not assume the repo is build-clean before you start editing.

## Architecture Landmarks
- `Core/Src/main.c` initializes HAL/peripherals, then starts FreeRTOS.
- `Core/Src/freertos.c` creates task threads.
- Task entry points are implemented in `ALL_Task/*_task.c`.
- Shared system state is centralized in `Application/robot_global.*`.
- Motor abstractions live in `Components/motor/`.
- Remote-control parsing lives in `Components/remote/`.
- Auto-aim parsing/control lives in `Application/auto_ctrl.*`.

## Generated-Code Boundaries
- Treat `Core/`, `USB_DEVICE/`, and `cmake/stm32cubemx/` as generated or generator-adjacent.
- Preserve every `/* USER CODE BEGIN ... */` and `/* USER CODE END ... */` region.
- Prefer adding custom logic inside user blocks instead of reshaping generated control flow.
- Avoid style-only rewrites in generated files.
- Do not rename generated entry points such as `MX_*`, IRQ handlers, or `Error_Handler()`.

## Code Style

### Language And Types
- Use C11-compatible code.
- Prefer fixed-width integer types like `uint8_t`, `uint16_t`, `int32_t`.
- Use project math aliases like `fp32` when touching existing control/math code that already uses them.
- Keep enum typedefs suffixed with `_e`.
- Keep struct typedefs suffixed with `_t`.
- Module-private structs may remain plain `struct name` when that matches local style.

### Naming
- Functions use lower snake case: `motor_get_device`, `parse_target_data`.
- Task entry points use `*_task_func`.
- Macros use upper snake case: `RC_DEADZONE`, `AUTO_SCAN_SPEED_RAD_S`.
- Encode units in constant names when useful: `_MS`, `_TICKS`, `_RAD`, `_RPM`, `_LIMIT`.
- Global/static device instances often use a `g_` prefix.
- Keep names descriptive; avoid cryptic abbreviations outside math/control hot paths.

### Includes
- Include the module's own header first when it has one.
- Prefer `<...>` for standard library headers.
- Prefer `"..."` for project, HAL, CMSIS, and FreeRTOS headers.
- Existing code uses many relative includes; stay consistent with the surrounding directory.
- Do not do broad include-path cleanup unless the task requires it.
- Match filename casing exactly; Windows hides case mistakes that other environments may not.

### Formatting
- Follow the surrounding file instead of imposing a new formatter style.
- Most handwritten files use 4-space indentation.
- Keep opening braces on the same line in handwritten code.
- Group related macros near the top of the file.
- Use blank lines to separate logical sections.
- Avoid formatting-only churn.

### Comments
- The repo already contains many Chinese comments and section banners.
- Preserve existing comments unless they become wrong.
- Add comments only for non-obvious hardware, control, or safety logic.
- Do not add comments that merely restate the code.

### Error Handling
- Follow the existing embedded pattern of early returns for bad pointers and invalid input.
- Check `NULL` aggressively in drivers, parsers, and reusable helpers.
- For HAL init/config failures in startup code, use `Error_Handler()`.
- For runtime control paths, prefer fail-safe state changes over optimistic assumptions.
- When comms drop, follow the existing pattern: clear enables, relax actuators, and reset transient state.
- Avoid infinite blocking in reusable helpers unless the function is explicitly fatal.

### RTOS And Concurrency
- Respect task cadence; several loops run at 2 ms or similarly tight periods.
- Avoid long blocking calls in high-priority tasks.
- Use `osDelay()` and tick-delta timing in the same style as neighboring code.
- Understand ownership of fields in `robot_ctrl` before changing shared state.
- Preserve startup sequencing such as waiting for sensors before enabling motion.

### Safety / Control Logic
- Mechanical limits, dead zones, timeout guards, and offline checks are intentional and safety-critical.
- Keep unit semantics straight; some comments mention degrees while many control constants are radians.
- Preserve clamping and watchdog behavior unless the task explicitly changes system behavior.
- Prefer small, local edits in control logic and verify the surrounding state machine.

## Agent Workflow
- Read the target file and at least one neighboring module before changing control code.
- Reuse existing abstractions such as `motor_device`, `robot_ctrl`, and UART/USB wrappers.
- Keep cross-module dependencies narrow; update headers only when an interface actually changes.
- If you edit generated files, keep changes inside user blocks whenever possible.
- After edits, run a full Debug build or at least a targeted object rebuild.
- If validation fails, distinguish pre-existing failures from failures introduced by your change.
- Be explicit that the repo has no tests instead of inventing unsupported test commands.

## Default Validation
```bash
cmake --preset Debug
cmake --build --preset Debug
```

## Fast Focused Validation
```bash
cmake --build build/Debug --target "CMakeFiles/Infantry_01.dir/path/to/file.c.obj"
```

## When Unsure
- Prefer repo consistency over textbook purity.
- Prefer fail-safe behavior over cleverness.
- Prefer small diffs over broad cleanup.
- Prefer honest reporting over pretending tests or lint steps exist.
