# Unit test for UMD/KMD

The basic unit test cases for quickly verifying UMD and KMD.

- mkdir ./benchmark.

- open file build.sh:
   add path where store cross compile tool chain to variable PATH.
   add path where store aipu v1/v2/v3 simulators and libraries to variable COMPASS_DRV_RTENVAR_SIM_LPATH.

- copy benchmark files aipu.bin and input0.bin to ./benchmark folder.

- if compile with arch X1 and run on silulator, mkdir ./simulator, copy X1 simulator binary to ./simulator folder, X2 don't need.

- run ./build.sh PLATFORM ARCH command to build and run ./runtime_unit_test to test. if you run on board, please copy benchmark folder to board.

## config example
```bash
# cd unit_test
# edit build.sh to add tool-chain and simulator lib path
# ./buld.sh x86 X1

# cd test_demo
# mdkir ./benchmark; mkdir ./simulator
# cp runtime_unit_test ./
# cp aipu.bin/input0.bin ./benchmark
# cp aipu_simulator_x1 ./simulator
# ./runtime_unit_test
```