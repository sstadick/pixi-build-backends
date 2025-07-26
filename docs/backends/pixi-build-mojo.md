# pixi-build-mojo

The `pixi-build-mojo` backend is designed for building Mojo projects. It provides seamless integration with Pixi's package magement workflow.

!!! warning
    `pixi-build` is a preview feature, and will change untill it is stabilized.
    This is why we require users to opt in to that feature by adding "pixi-build" to `workspace.preview`.

    ```toml
    [workspace]
    preview = ['pixi-build']
    ```

## Overview

This backend automatically generates conda packages from Mojo projects, and can install those into local environments.

## Basic Usage

To use the Mojo backend in your `pixi.toml`, add it to your package's build configuration:


```txt
# Example project layout for combined binary/library.
.
├── greetings
│   ├── __init__.mojo
│   └── lib.mojo
├── main.mojo
├── pixi.lock
├── pixi.toml
├── README.md
├── src
└── structure.txt
```

```toml
[workspace]
authors = ["Seth Stadick <sstadick@gmail.com>"]
platforms = ["linux-64"]
preview = ["pixi-build"]
channels = [
    "conda-forge",
    "https://conda.modular.com/max-nightly",
    "https://prefix.dev/pixi-build-backends",
    "https://repo.prefix.dev/modular-community"
]

[package]
name = "greetings"
version = "0.1.0"

[package.build]
backend = { name = "pixi-build-mojo", version = "0.1.*" }

[tasks]

[package.build.configuration]
# This is entirey optional. pixi install is recommended
# dist-dir = "./target"

[[package.build.configuration.bins]]
name = "greet"
path = "./main.mojo"
#extra-args = ["-I", "special-thing"]
#extra-input_globs = ["**/.c"]

[package.build.configuration.pkg]
name = "greetings"
path = "greetings"
#extra-args = ["-I", "special-thing"]
#extra-input_globs = ["**/.c"]

[package.host-dependencies]
max = "=25.4.0"

[package.build-dependencies]
max = "=25.4.0"
small_time = ">=25.4.1,<26"
extramojo = ">=0.16.0,<0.17"

[package.run-dependencies]
max = "=25.4.0"

[dependencies]
# For running `mojo test` while developing you'll probably want
# everything under "build-dependencies" as well
greetings = { path = "." }
```

### Required Dependencies

- `max` package for both the compiler and linked runtime

## Configuration Options

- **env** key-value pairs of environment variables to pass in
    - **note** not yet implemented
- **dist-dir** a directory to create relative to the manifest pixi.toml to copy build artifacts into. Prefer `pixi install` to using this for better caching.
- **debug-dir** directory to place internal pixi debug information into.
- **extra-input-globs** additional globs to pass to pixi for including more than just mojo files in the build
- **bins** list of binary configurations to build, see [bins](#bins).
- **pkg** pkg configuration for creating a `.mojopkg`, see [pkg](#pkg).

### `bins`

- **name** the binary name
- **path** the path to the file that contains a `main` function
- **extra-args** list of extra arguments to pass to the compiler
- **env** env vars to set
    - **note** not yet implemented

The created binary will be placed in the `$PREFIX/bin` dir and will be in the path after running `pixi install` assuming the package is listed as a dependency as in the example above. `pixi build` will create a conda package that includes the binary.

### `pkg`

- **name** the binary name
- **path** the path to the file that contains a `main` function
- **extra-args** list of extra arguments to pass to the compiler
- **env** env vars to set
    - **note** not yet implemented

The created mojopkg will be placed in the `$PREFIX/lib/mojo` dir, which will make it discoverable to anything that depends on the package.
