# Unit test for UMD/KMD

The basic unit test cases for quickly verifying UMD and KMD.

- mkdir ./benchmark ./simulator.

- open file build.sh:
   add you cross compile tool chain to variable PATH.

- copy X1 benchmark files aipu.bin and input0.bin to ./benchmark folder.

- copy X1 simulator binary to ./benchmark folder if run on silulator.

- run ./build.sh PLATFORM command to build and run ./runtime_unit_test to test. if you run on board, please copy benchmark folder to board.

## config example
```bash
# cd unit_test
# edit build.sh to add tool-chain
# ./buld.sh x86

# cd test_demo
# mdkir ./benchmark; mkdir ./simulator
# cp runtime_unit_test ./
# cp aipu.bin/input0.bin ./benchmark
# cp aipu_simulator_x1 ./simulator
# ./runtime_unit_test
```