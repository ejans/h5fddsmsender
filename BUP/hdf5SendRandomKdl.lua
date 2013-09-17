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
ubx.load_module(ni, "std_blocks/h5fddsmsender/H5FDdsmSender.so")
ubx.load_module(ni, "std_blocks/lfds_buffers/lfds_cyclic.so")
--ubx.load_module(ni, "std_triggers/ptrig/ptrig.so")
ubx.load_module(ni, "std_blocks/ptrig/ptrig.so")
ubx.load_module(ni, "std_blocks/random_kdl/random_kdl.so")

--TODO Test
ubx.ffi_load_types(ni)

print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })

--print("creating instance of 'std_blocks/ptrig'")
--ptrig1=ubx.block_create(ni, "std_blocks/ptrig", "ptrig1")

print("creating instance of 'random_kdl/random_kdl'")
random_kdl1=ubx.block_create(ni, "random_kdl/random_kdl", "random_kdl1", {min_max_config={min=32, max=127}})

print("creating instance of 'std_blocks/h5fddsmsender'")
--hdf5=ubx.block_create(ni, "H5FDdsmSender/H5FDdseSender", "hdf5")
--hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5")
--hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5", { ip="10.33.173.147", port="22000"})
--hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5", { ip="10.33.174.62", port="22000"})
hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5", { port_ip_config={ip="10.33.173.147", port="22000"}})

print("running webif init", ubx.block_init(webif1))
print("running webif start", ubx.block_start(webif1))

--[[print("creating instance of 'std_triggers/ptrig'")
ptrig1=ubx.block_create(ni, "std_triggers/ptrig", "ptrig1")
ptrig1=ubx.block_create(ni, "std_triggers/ptrig", "ptrig1",
                        {
                           period = {sec=5, usec=0 },
                           sched_policy="SCHED_OTHER", sched_priority=0,
                           trig_blocks={ { b=random_kdl1, num_steps=1, measure=0 },
                                         { b=hdf5, num_steps=1, measuer=0 }
                           } } )
]]

--[[print("connecting ports")
kdl_twist_port=ubx.port_get(random_kdl1, "base_msr_twist")
kdl_frame_port=ubx.port_get(random_kdl1, "base_msr_odom")

ubx.connect_one(kdl_twist_port, hdf5)
ubx.connect_one(kdl_frame_port, hdf5)
print("connected")
]]

io.read()

--TODO TEST:
data1 = ubx.data_alloc(ni, "struct kdl_twist", 1)
kdl_twist_port_in=ubx.port_get(random_kdl1, "base_msr_twist")
data1 = ubx.port_read(kdl_twist_port_in)
print(ubx.data_tostr(data1))
kdl_twist_port_out=ubx.port_get(hdf5, "base_msr_twist")
ubx.port_write(kdl_twist_port_out, data1)

--i1=ubx.data_alloc(ni, "struct kdl_frame", 1)
--print(ubx.data_tostr(i1))


io.read()

node_cleanup(ni)



