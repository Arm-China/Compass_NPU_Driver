// Copyright (C) 2022 Arm Technology (China) Co. Ltd. All rights reserved.

1. mkdir ./benchmark ./simulator.

2. open file build.sh:
   add you gcc location to variable PATH;
   if you run unit test on simulator, edit variable X1_SIMULATOR_PATH as your simulator location.

3. copy X1 benchmark files aipu.bin and input0.bin to ./benchmark folder.

4. run ./build.sh PLATFORM command to build and run ./runtime_unit_test to test. if you run on board, please copy benchmark folder to board.
