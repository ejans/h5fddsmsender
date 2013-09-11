ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
CC=clang++
INCLUDE_DIR=$(ROOT_DIR)/src/

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

H5FDdsmSender.so: H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so
	${CC} $(CFLAGS_SHARED) -o H5FDdsmSender.so H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so /home/evert/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsmTools.so /home/evert/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsm.so /home/evert/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsmTesting.a /home/evert/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsm_sys.so /home/evert/local/lib/libhdf5.so
H5FDdsmSender.o: H5FDdsmSender.cxx $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -I/home/evert/local/include/ -I/home/evert/h5fddsm/h5fddsm-0.9.9/src -I/home/evert/h5fddsm/h5fddsm-0.9.9/build/src/ -I/home/evert/h5fddsm/h5fddsm-0.9.9/Testing/ -c $(CFLAGS) H5FDdsmSender.cxx

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
