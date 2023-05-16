This test case is for verify accessing dma_buf in kernel mode via vmap mechanism.

1. compile impoter.ko module
2. compile user application aipu_dmabuf_vmap_test
3. copy the above binary and libaipudrv.so to target board
4. on target board, run the below commands
	# insmod aipu.ko
	# insmod importer.ko
	# ./aipu_dmabuf_vmap_test

5. if the above command run successfully, the expected result as follows:

	read from dma_buf: This is string wroten by user!
	read from dma_buf: This is string filled by kernel module!
