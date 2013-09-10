ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
CC=clang++
INCLUDE_DIR=$(ROOT_DIR)/src/

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

H5FDdsmSender.so: H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so
	${CC} $(CFLAGS_SHARED) -o H5FDdsmSender.so H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so /home/evert/Documents/KULeuven/NEW_2013_07_15/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsmTools.so /home/evert/Documents/KULeuven/NEW_2013_07_15/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsm.so /home/evert/Documents/KULeuven/NEW_2013_07_15/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsmTesting.a /home/evert/Documents/KULeuven/NEW_2013_07_15/h5fddsm/h5fddsm-0.9.9/build/bin/libH5FDdsm_sys.so /usr/local/lib/libhdf5.so
H5FDdsmSender.o: H5FDdsmSender.cxx $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -c $(CFLAGS) H5FDdsmSender.cxx

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
