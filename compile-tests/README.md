# Tests to check if "released" crates work

This folder is special in a couple of ways.

It's meant to check that released crates work well together. Especially it's meant to "simulate" a release
by using a private registry (i.e. a local registry).

For that reason the projects under this folder don't use path-dependencies.

This folder (and it's sub-folders) are also not known to the general `xtask` commands. (except the `rel-check` command)

The projects don't contain real code but just define dependencies to make sure they compile.

## Walkthrough how to use it with the `rel-check` command

`cargo xrel-check init`

This will initialize the local registry with the dependencies used by the projects found in the `compile-tests` folder.

`cargo xrel-check check`

This just tries to build every project in the `compile-tests` folder.
Please note that we remove `Cargo.lock` files before the build if they are present.
If the local registry is initialized this will use it.

To test how a version bump affects the test projects let's manually bump `esp-backtrace` to e.g. `0.999.0`. (You should bump the version of `esp-backtrace` in its `Cargo.toml` and the version used in the `compile-test` project)

`cargo xrel-check update esp-backtrace`

This will package and add the esp-backtrace crate.

Let's see if it still compiles. (the test projects still use the current version - so this is expected to work)

`cargo xrel-check check`

Now get the test projects updated to use the bumped version of `esp-backtrace`:

`cargo xrel-check yolo-bump`

Notice the change in the `Cargo.toml`s - executing `cargo xrel-check check` should still work using the freshly "released" version of `esp-backtrace`.

Now lets see how a bad release looks like!

Bump the patch version of `esp-rom-sys` and change the name of the `init_syscall_table` function.

`cargo xrel-check update esp-rom-sys` will "release" this breaking update.

`cargo xrel-check check` should fail now because the test project will pickup the problematic patch release of `esp-rom-sys`.
Even without changing anything anything in the test project.

When done: `cargo xrel-check deinit` and revert local changes
