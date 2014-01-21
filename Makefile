ROOT_DIR=$(CURDIR)/../..
include $(ROOT_DIR)/make.conf
INCLUDE_DIR=$(ROOT_DIR)/src/
#CC=clang++
# Set this to the directory where you installed h5fddsm
H5FDDSM_DIR=$(HOME)/h5fddsm/h5fddsm-0.9.9
#H5FDDSM_DIR=$(KUL)/h5fddsm/h5fddsm-0.9.9/
# Set this to the directory where you installed the dependencies of h5fddsm
#LOCAL_DIR=/usr/local/
LOCAL_DIR=$(HOME)/local
KDL_DIR=$(ROOT_DIR)/std_types/kdl/types

TYPES:=$(wildcard types/*.h)
HEXARRS:=$(TYPES:%=%.hexarr)

H5FDdsmSender.so: H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so
	${CC} $(CFLAGS_SHARED) -o H5FDdsmSender.so H5FDdsmSender.o $(INCLUDE_DIR)/libubx.so $(H5FDDSM_DIR)/build/bin/libH5FDdsmTools.so $(H5FDDSM_DIR)/build/bin/libH5FDdsm.so $(H5FDDSM_DIR)/build/bin/libH5FDdsmTesting.a $(H5FDDSM_DIR)/build/bin/libH5FDdsm_sys.so $(LOCAL_DIR)/lib/libhdf5.so

H5FDdsmSender.lua.hexarr: H5FDdsmSender.lua
	../../tools/file2carr.lua H5FDdsmSender.lua

H5FDdsmSender.o: H5FDdsmSender.cpp $(INCLUDE_DIR)/ubx.h $(INCLUDE_DIR)/ubx_types.h $(INCLUDE_DIR)/ubx.c $(HEXARRS)
	${CC} -fPIC -I$(INCLUDE_DIR) -I$(LOCAL_DIR)/include/ -I$(H5FDDSM_DIR)/src -I$(H5FDDSM_DIR)/build/src/ -I$(H5FDDSM_DIR)/Testing/ -I$(KDL_DIR) -c $(CFLAGS) H5FDdsmSender.cpp -std=gnu++0x

clean:
	rm -f *.o *.so *~ core $(HEXARRS)
