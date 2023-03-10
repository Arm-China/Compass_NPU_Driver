AIPU LINUX DRIVER RELEASE OUT-Of-BOX README
===========================================
This README is for Zhouyi driver release out-of-box package.

In this package, we provide a simple user application demo which
allow you to run an inference task on x86-Linux simulator as target platform.

The demo application source code is also an example of the programming model.
You may update it based on the specification in AIPU Software Programming Guide
if you need to support more features.

Before running this demo, you should ensure that AIPU simulator and corresponding .so
are in related path, and configure the related directory in run.sh script:

    SIMULATOR
    SIMULATOR_SO_DIR

To build & run this demo application, please follow these steps:

	1. Compile and execute the applicateion with provided benchmark resnet_50 by:
		$./out-of-box-test.sh

	2. A log "[TEST INFO] Test Result Check PASS!" will be printed
        which means that the execution is successful.

    note:
		you can only execute the application with provided benchmark resnet_50 if run step #1 beforehand, by:
        - for aipu v1/v2
        $./run.sh -c resnet_50

        - for aipu v3
        $./run.sh -a [X2_1204|X2_1204MP3] -c resnet_50
