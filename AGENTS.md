# AGENTS.md

## Purpose
- This repository builds STM32F407 firmware for the `Infantry_01` target using CMake, Ninja, STM32CubeMX-generated code, HAL, FreeRTOS, and custom application modules.
- Use this file as the working agreement for autonomous coding agents modifying this repo.
- Prefer small, localized edits that match the existing embedded-C style and avoid unnecessary framework or architecture churn.

## Repository Shape
- `Application/`: top-level app state and cross-module integration.
- `ALL_Task/`: FreeRTOS task entry points and periodic control/UI loops.
- `Bsp/`: board support code for CAN, UART, USB CDC, LED, and similar hardware wrappers.
- `Components/`: reusable device/protocol modules such as remote, referee, motor, BMI088, and super capacitor logic.
- `Core/`, `USB_DEVICE/`, `cmake/stm32cubemx/`: mostly STM32CubeMX-generated startup, HAL, RTOS, and USB glue.
- `Drivers/`, `Middlewares/`: vendor code; do not restyle or refactor unless the task explicitly requires it.
- `Infantry_01.ioc`: STM32CubeMX project source; hardware config changes should stay consistent with generated code.

## External Rule Files
- No `.cursor/rules/` directory was found.
- No `.cursorrules` file was found.
- No `.github/copilot-instructions.md` file was found.
- If any of those files are added later, merge their instructions into this document and treat them as repo-specific policy.

## Toolchain And Build System
- Required toolchain: `arm-none-eabi-gcc` family must be on `PATH`.
- The CMake toolchain file is `cmake/gcc-arm-none-eabi.cmake`.
- C standard is fixed to C11 in `CMakeLists.txt`.
- Default generator is Ninja via `CMakePresets.json`.
- Supported configure/build presets: `Debug`, `RelWithDebInfo`, `Release`, `MinSizeRel`.
- The main firmware artifact is an ELF executable named after the project, currently `sentry_chassis.elf`.

## Build Commands
- Configure a debug build:
```sh
cmake --preset Debug
```
- Build the debug firmware:
```sh
cmake --build --preset Debug
```
- Configure and build release firmware:
```sh
cmake --preset Release
cmake --build --preset Release
```
- Build a specific target from an existing build directory if needed:
```sh
cmake --build build/Debug --target sentry_chassis
```
- Clean by deleting `build/<Preset>/` or using the generator clean target:
```sh
cmake --build build/Debug --target clean
```

## Lint / Static Analysis
- No dedicated lint target, formatter config, or static-analysis script is checked into this repo.
- The closest built-in guardrail is compiling with warnings enabled in `cmake/gcc-arm-none-eabi.cmake`:
  - `-Wall`
  - `-Wextra`
  - `-Wpedantic`
- Treat a clean build as the minimum validation step.
- If a task asks for linting, use a build first; do not invent a new lint stack unless requested.

## Test Status
- No automated unit-test framework or `CTest` setup was found.
- No `add_test(...)`, `enable_testing()`, Unity, Catch2, GoogleTest, or pytest config was found.
- There is currently no supported repository command for `test` or `single test` execution.
- Be explicit in reports: say that automated tests are not configured rather than pretending they exist.

## Single-Test Guidance
- Current status: not available in-repo because no test harness is defined.
- If tests are added later through `CTest`, the likely pattern will be:
```sh
ctest --test-dir build/Debug -R <test_name> --output-on-failure
```
- Do not add this as a claimed supported command unless the repo actually gains `CTest` targets.

## Generated Code Boundaries
- `Core/`, `USB_DEVICE/`, and `cmake/stm32cubemx/` contain generated STM32CubeMX code and build glue.
- When editing generated STM32 files, prefer `/* USER CODE BEGIN ... */` blocks.
- Do not manually reformat entire generated files.
- Do not move generated function signatures or reorder generated initialization unless the task requires regeneration-aware edits.
- Keep `Infantry_01.ioc` in sync with any hardware/peripheral changes that depend on CubeMX configuration.

## What To Edit First
- Put new business logic in `Application/`, `Bsp/`, `Components/`, or `ALL_Task/` before touching generated code.
- Use `Core/Src/main.c` mainly for startup sequencing and calls into higher-level modules.
- Add peripheral wrappers in `Bsp/` instead of scattering HAL calls across unrelated modules.
- Keep protocol parsing and device-specific state in `Components/`.

## Imports And Include Style
- Match the local convention: include the module's own header first in `.c` files.
- Then include nearby project headers, then HAL/RTOS headers, then standard library headers if the file already follows that pattern.
- Existing files use both quotes and angle brackets for C library headers; preserve the surrounding file's style instead of mass-normalizing.
- Relative includes are common, for example `../...` and `../../...`; do not rewrite include paths repo-wide unless required.
- Avoid adding unused includes; this codebase is warning-sensitive and runs in a constrained embedded environment.

## Formatting Conventions
- Use 4-space indentation in user-authored code.
- Keep braces and whitespace consistent with the file you are editing; generated files and hand-written files differ slightly.
- Keep lines readable rather than aggressively wrapped; long macro tables and protocol structs are already accepted where necessary.
- Prefer one statement per line.
- Preserve existing block comment banners if editing files that already use them.
- Do not restyle vendor or generated code just to make it prettier.

## Types
- Use the repo's established fixed-width aliases where existing modules already rely on them:
  - `uint8_t`, `uint16_t`, `uint32_t`, `int16_t`, etc.
  - `fp32` and `fp64` from `Application/struct_typedef.h` where that file is already part of the module boundary.
- Prefer explicit-width integer types for protocol packets, CAN frames, DMA buffers, and hardware-facing structs.
- Use `const` for read-only pointers and handles when the API allows it.
- Preserve packed structs and bitfields exactly when they map to wire formats.
- Be careful with implicit integer promotions and signed/unsigned mixing in register, DMA, and protocol code.

## Naming Conventions
- Macro names are uppercase with underscores, often module-prefixed, for example `RC_FRAME_LENGTH_VT13`.
- Public functions usually use PascalCase or upper-module prefixes, for example `Robot_Global_Init`, `Referee_Init`, `RC_Init`.
- Internal helper functions are often `static` and may use snake_case, for example `clamp_u16`.
- Struct typedef names commonly end with `_t`; keep existing public names stable.
- Hardware/device globals follow STM32/HAL naming such as `huart3`, `hcan1`, and should not be renamed casually.
- New names should be descriptive, module-scoped, and consistent with nearby code rather than globally standardized.

## Error Handling
- Prefer cheap guard clauses for invalid pointers, malformed frames, wrong CAN IDs, and failed preconditions.
- Existing code commonly uses early `return` for invalid state; follow that pattern in leaf functions.
- For HAL init/config failures in startup code, defer to `Error_Handler()` unless the surrounding module already has a richer recovery path.
- In runtime modules, fail safely and preserve previous valid state when parsing or receiving bad data.
- Do not add exceptions, heap-heavy recovery logic, or desktop-style logging frameworks.

## RTOS And Concurrency
- This project uses FreeRTOS through CMSIS-OS.
- Task loops commonly run forever with short `osDelay(...)` periods; preserve task cadence unless behavior changes are intentional.
- Be mindful of shared globals such as `global_info` and device state updated from interrupts or periodic tasks.
- ISR-related code should stay fast, deterministic, and allocation-free.
- Avoid blocking calls in interrupt context or timing-sensitive control paths.

## Embedded-Specific Practices
- Avoid dynamic allocation unless the existing module clearly depends on it.
- Prefer static storage, fixed-size buffers, and explicit bounds.
- Keep floating-point use intentional; the toolchain enables `_printf_float`, but firmware size still matters.
- Treat communication structs, DMA buffers, and CAN payload packing as compatibility-sensitive.
- Do not change register-level or HAL sequencing casually; many embedded bugs are ordering bugs.

## Comments And Documentation
- Add comments when hardware behavior, packet layout, or control logic is non-obvious.
- Do not add redundant comments that merely restate the code.
- Several user-authored files contain Chinese comments. Match the surrounding language style in edited files.
- Keep public header comments useful for callers: units, ranges, ownership, and side effects matter more than prose.

## Validation Expectations For Agents
- Minimum check after code changes: build with `cmake --build --preset Debug` if the toolchain is available.
- If you touch startup, interrupts, CAN, UART, USB, or RTOS code, review call ordering carefully even if you cannot run on hardware.
- If no hardware validation is possible, clearly say so and list the code paths most likely to need on-device verification.
- Mention when a change affects CubeMX-managed files so humans know regeneration risk exists.

## Practical Agent Rules
- Prefer surgical diffs over broad cleanup.
- Do not introduce new third-party dependencies without an explicit request.
- Do not claim test coverage that does not exist.
- Keep changes compatible with bare-metal/RTOS embedded constraints.
- When uncertain, preserve current module boundaries and existing patterns.
