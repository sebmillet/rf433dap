Tests
=====

The test_linux folder manages a test plan for the rf433decode.ino sketch.

As the name suggests, it'll work under Linux only...

The test is made of two-digit folders where we find:

- A code file, that has a name starting with code-

- The expected output, with always the name expect.txt

In order for the test plan to work, the SIMULATE macro must be defined inside
rf433decode.ino.

The test consists in launching the script

    tt.sh

This script must be updated depending on the port the board is connected to.

The typical sequence of instructions to do the test is:

1. The working directory being main rf433dap folder, execute:

```shell
./am rf433decode.ino -u --stty
cd test_linux
./tt.sh
```

