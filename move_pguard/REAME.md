# Package move_pguard

## Build instructions

### Make configuration file executable

This package uses `dynamic_reconfigure`. Therefore all the files located at
`./cfg` **have to be executable**. Assuming the current directory is the root of
the package, you can use the following command:

```bash
chmod a+x cfg/*
```

This step is only required when there is a new file in the `cfg` folder.

### Build the workspace

Then as usual, you can compile with Catkin, from your workspace, by typing:

```bash
catkin_make
```
