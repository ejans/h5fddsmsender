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

print("creating instance of 'webif/webif'")
webif1=ubx.block_create(ni, "webif/webif", "webif1", { port="8888" })

print("creating instance of 'std_blocks/ptrig'")
ptrig1=ubx.block_create(ni, "std_blocks/ptrig", "ptrig1")

print("creating instance of 'random_kdl/random_kdl'")
random_kdl1=ubx.block_create(ni, "random_kdl/random_kdl", "random_kdl1", {min_max_config={min=32, max=127}})

print("creating instance of 'std_blocks/h5fddsmsender'")
--hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5", { port_ip_config={ip="10.33.172.170", port="22000"}})
hdf5=ubx.block_create(ni, "std_blocks/h5fddsmsender", "hdf5", { port_ip_config={ip="192.168.10.171", port="22000"}})

print("running webif init", ubx.block_init(webif1))
print("running webif start", ubx.block_start(webif1))

print("before fifo connect")
fifo=ubx.conn_lfds_cyclic(random_kdl1, "base_msr_twist", hdf5, "base_msr_twist", 1, true);

print("start fifo")
ubx.block_start(fifo)
print("fifo block started!")

--- Move with a given twist.
-- @param twist table.
-- @param dur duration in seconds
function move_twist(twist_tab, dur)
   --set_control_mode(2) -- VELOCITY
   ubx.data_set(twist_data, twist_tab)
   local ts_start=ffi.new("struct ubx_timespec")
   local ts_cur=ffi.new("struct ubx_timespec")

   ubx.clock_mono_gettime(ts_start)
   ubx.clock_mono_gettime(ts_cur)

   while ts_cur.sec - ts_start.sec < dur do
      ubx.port_write(p_msr_twist, twist_data)
      ubx.clock_mono_gettime(ts_cur)
   end
   ubx.port_write(p_msr_twist, null_twist_data)
end

function fill_twist()
   ubx.port_write(p_msr_twist, null_twist_data)
end

function fill_twist2()
   twist_data = ubx.port_read(i_msr_twist)
   ubx.port_write(p_msr_twist, twist_data)
end

function connect_twist()
   ubx.connect_one(i_mst_twist, hdf5)
end

io.read()

node_cleanup(ni)


