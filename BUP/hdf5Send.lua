#!/usr/bin/luajit

local ffi = require("ffi")
local ubx = require "ubx"
local ubx_utils = require("ubx_utils")
local ts = tostring

-- prog starts here.
ni=ubx.node_create("testnode")

ubx.load_module(ni, "std_types/stdtypes/stdtypes.so")
ubx.load_module(ni, "std_types/kdl/kdl_types.so")
ubx.load_module(ni, "std_blocks/webif/webif.so")
ubx.load_module(ni, "std_blocks/H5FDdsmSender/H5FDdsmSender.so")
ubx.load_module(ni, "std_triggers/ptrig/ptrig.so")

print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })

print("creating instance of 'std_triggers/ptrig'")
ptrig1=ubx.block_create(ni, "std_triggers/ptrig", "ptrig1")

print("creating instance of 'std_blocks/H5FDdsmSender'")
hdf5=ubx.block_create(ni, "H5FDdsmSender/H5FDdseSender", "hdf5")

print("running webif init", ubx.block_init(webif1))
print("running webif start", ubx.block_start(webif1))

io.read()

node_cleanup(ni)



