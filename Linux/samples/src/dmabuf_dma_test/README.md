This test case is for the verification of accessing dma_buf in kernel mode via vmap mechanism.

1. compile importer.ko module
2. compile the user application aipu_dmabuf_dma_test
3. copy the above binaries and aipu.ko & libaipudrv.so to the target board
4. on the target board, execute:
	# insmod aipu.ko
	# insmod importer.ko
	# ./aipu_dmabuf_dma_test

5. if the above commands run successfully, the following result is expected to be printed on screen:

	read from dma_buf: This is string wroten by user!
	read from dma_buf: This is string filled by kernel module!
