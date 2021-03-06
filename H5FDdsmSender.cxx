/*=========================================================================
  A fblock that sends a HDF5 file to a server
  Copyright (C) 2013 Evert Jans

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see [http://www.gnu.org/licenses/].

=========================================================================*/
//#define DEBUG 1

#include <stdio.h>
#include <time.h>
#include <hdf5.h>
#include <cstdlib>

#include "ubx.h"

#include <H5FDdsmTest.h>
#include <H5FDdsm.h>

#include <kdl.h>
#include "../youbot_driver/types/motionctrl_jnt_state.h"
#include "../youbot_driver/types/youbot_arm_motorinfo.h"
#include "../youbot_driver/types/youbot_base_motorinfo.h"
#include "../youbot_driver/types/youbot_control_modes.h"

#include "types/H5FDdsmSender_config.h"
#include "types/H5FDdsmSender_config.h.hexarr"

ubx_type_t H5FDdsmSender_config_type = def_struct_type(struct H5FDdsmSender_config, &H5FDdsmSender_config_h);

#define FILE "youbot.h5"

/* function block meta-data
 * used by higher level functions.
 */
char h5fsnd_meta[] =
        "{ doc='A hdf5-data sender function block',"
        "  license='LGPL',"
	"  real-time=false,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t h5fsnd_config[] = {
        
        {.name="port_ip_config", .type_name="struct H5FDdsmSender_config"},

        {NULL},
};


/* we need twist and frame data from the robot */
ubx_port_t h5fsnd_ports[] = {

	{ .name="base_msr_twist", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_twist" },
	{ .name="base_msr_odom", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_frame" },
        { .name="base_motorinfo", .attrs=PORT_DIR_IN, .in_type_name="struct youbot_base_motorinfo" },
        { .name="arm1_motorinfo",  .in_type_name="struct youbot_arm_motorinfo" },
        { .name="arm1_state", .in_type_name="struct motionctrl_jnt_state" },
        { .name="arm1_gripper", .out_type_name="int32_t" },

        { NULL },
        { NULL },
};

struct H5FDdsmSender_info {

	MPI_Comm comm;
	H5FDdsmManager* dsmManager;
	
	hid_t       file_id, group_id, dataset_id, dataspace_id, hdf5Handle, fapl, attribute_id;  /* identifiers */
	hsize_t     dims[2];
	herr_t      status;
	//time_t      now;
	//char*       time_string;
        char*       ip;
        int         port;
};

/* convenience functions to read/write from the ports */
def_read_fun(read_kdl_twist, struct kdl_twist)
def_read_fun(read_kdl_frame, struct kdl_frame)
//def_read_fun(read_base_motorinfo, struct youbot_base_motorinfo)
//def_read_fun(read_arm_motorinfo, struct youbot_arm_motorinfo)
def_read_fun(read_arm_state, struct motionctrl_jnt_state)
//def_read_fun(read_gripper_state, int32_t)

void createGroup(struct H5FDdsmSender_info* sinfo, const char* name);

void createGroups(struct H5FDdsmSender_info* sinfo) {
	
	createGroup(sinfo, "/State");
	createGroup(sinfo, "/State/TimeStamp");
	createGroup(sinfo, "State/Twist");
	createGroup(sinfo, "State/Twist/RotationalVelocity");
	createGroup(sinfo, "State/Twist/LinearVelocity");
	createGroup(sinfo, "/State/BaseCartesianPosition");
	createGroup(sinfo, "/State/BaseCartesianPosition/Vector");
	createGroup(sinfo, "/State/BaseCartesianPosition/Rotation");
	createGroup(sinfo, "/State/BaseMotorinfo");
	createGroup(sinfo, "/State/ArmMotorinfo");
	createGroup(sinfo, "/State/JointStates");
}

void createGroup(struct H5FDdsmSender_info* sinfo, const char* name) {
        
	sinfo->group_id = H5Gcreate2(sinfo->hdf5Handle, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	H5Gclose(sinfo->group_id);
}

void setDataspaceId(struct H5FDdsmSender_info* sinfo, int rank, hsize_t* max_dims);

void createStringAttribute(struct H5FDdsmSender_info* sinfo, const char* name, char* data) {

        setDataspaceId(sinfo, 1, NULL);
        hid_t atype = H5Tcopy(H5T_C_S1);
        H5Tset_size(atype, strlen(data));
        H5Tset_strpad(atype,H5T_STR_NULLTERM);

        sinfo->attribute_id = H5Acreate2(sinfo->dataset_id, name, atype, sinfo->dataspace_id, H5P_DEFAULT, H5P_DEFAULT);
        H5Awrite(sinfo->attribute_id, atype, data);
        H5Aclose(sinfo->attribute_id);
}

void setDataspaceId(struct H5FDdsmSender_info* sinfo, int rank, hsize_t* max_dims) {
	
	sinfo->dataspace_id = H5Screate_simple(rank, sinfo->dims, max_dims);
}

void closeDataspace(struct H5FDdsmSender_info* sinfo) {

        H5Sclose(sinfo->dataspace_id);
}

void closeDataset(struct H5FDdsmSender_info* sinfo) {
        
        H5Dclose(sinfo->dataset_id);
}

void createDatasetChar(H5FDdsmSender_info* inf, const char* name, char* data) {
	
        hid_t atype = H5Tcopy(H5T_C_S1);
        H5Tset_size(atype, strlen(data));
        H5Tset_strpad(atype,H5T_STR_NULLTERM);

	inf->dataset_id = H5Dcreate2(inf->hdf5Handle, name, atype, inf->dataspace_id, H5P_DEFAULT, H5P_DEFAULT, 
	        H5P_DEFAULT);
	H5Dwrite(inf->dataset_id, atype, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

}

void createDatasetDouble(H5FDdsmSender_info* inf, const char* name, double* data) {
	
	inf->dataset_id = H5Dcreate2(inf->hdf5Handle, name, H5T_NATIVE_DOUBLE, inf->dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
	        H5P_DEFAULT);
	H5Dwrite(inf->dataset_id, H5T_NATIVE_DOUBLE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

}

void createDatasetInteger(H5FDdsmSender_info* inf, const char* name, int* data) {
	
	inf->dataset_id = H5Dcreate2(inf->hdf5Handle, name, H5T_NATIVE_INT, inf->dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
	        H5P_DEFAULT);
	H5Dwrite(inf->dataset_id, H5T_NATIVE_INT, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);

}

static int h5fsnd_init(ubx_block_t *c) {
	
	int ret=0;
        char *port_string;
        struct H5FDdsmSender_config* senderconf;
        unsigned int clen;

	DBG(" ");
        if ((c->private_data = calloc(1, sizeof(struct H5FDdsmSender_info)))==NULL) {

                ERR("Failed to alloc memory");
                ret=EOUTOFMEM;
                goto out;
        }

	struct H5FDdsmSender_info* inf;
	inf=(struct H5FDdsmSender_info*) c->private_data;

        /* Get config and put inside inf */
        senderconf = (struct H5FDdsmSender_config*) ubx_config_get_data_ptr(c, "port_ip_config", &clen);
        inf->ip = senderconf->ip;
        port_string = senderconf->port;
        sscanf(port_string, "%u", &inf->port);

        inf->comm = MPI_COMM_WORLD;
        inf->dsmManager = new H5FDdsmManager();
        /* Set the ip and port from config */
	senderInitIp(NULL, 0, inf->port, inf->ip, inf->dsmManager, &inf->comm);

 out:
        return ret;
}

static void h5fsnd_cleanup(ubx_block_t *c) {
        
	struct H5FDdsmSender_info* inf;
        inf=(struct H5FDdsmSender_info*) c->private_data;
        DBG(" ");
	senderFinalize(inf->dsmManager, &inf->comm);
	delete inf->dsmManager;
	free(c->private_data);
}

static int h5fsnd_start(ubx_block_t *c) {
        
	DBG("in");
        return 0; /* Ok */
}

static void h5fsnd_step(ubx_block_t *c) {

        struct H5FDdsmSender_info* inf;
	uint32_t ret;
	struct kdl_twist twist;
	struct kdl_frame frame;
        //struct youbot_base_motorinfo baseInfo;
        //struct youbot_arm_motorinfo armInfo;
        struct motionctrl_jnt_state armState;
        //int32_t gripperState;

        time_t now;
        char* time_string;

        inf=(struct H5FDdsmSender_info*) c->private_data;

	/* Set file access property list for DSM */
	inf->fapl = H5Pcreate(H5P_FILE_ACCESS);
	/* Use DSM driver */
	H5Pset_fapl_dsm(inf->fapl, inf->comm, NULL, 0);
	/* Create DSM */
	inf->hdf5Handle = H5Fcreate(FILE, H5F_ACC_TRUNC, H5P_DEFAULT, inf->fapl);
	/* Close file access property list */
	H5Pclose(inf->fapl);
        /* Create all groups */
	createGroups(inf);

	/* Receive data from robot */
        ubx_port_t* twist_port = ubx_port_get(c, "base_msr_twist");
	ubx_port_t* frame_port = ubx_port_get(c, "base_msr_odom");
        //ubx_port_t* base_info_port = ubx_port_get(c, "base_motorinfo");
        //ubx_port_t* arm_info_port = ubx_port_get(c, "arm1_motorinfo");
        ubx_port_t* arm_state_port = ubx_port_get(c, "arm1_state");
        //ubx_port_t* gripper_state_port = ubx_port_get(c, "arm1_gripper");

	ret = read_kdl_twist(twist_port, &twist);
	if(ret>0) {
	        DBG("twist changed");
	} else {
	        ERR("twist unchanged");
		goto out;
	}
	ret = read_kdl_frame(frame_port, &frame);
	if(ret>0) {
	        DBG("frame changed");
	} else {
	        ERR("frame unchanged");
		goto out;
        }
        //ret = read_base_motorinfo(base_info_port, &baseInfo);
	//if(ret>0) {
	        //DBG("baseInfo changed");
	//} else {
	        //ERR("baseInfo unchanged");
		//goto out;
        //}
        //ret = read_arm_motorinfo(arm_info_port, &armInfo);
	//if(ret>0) {
	        //DBG("armInfo changed");
	//} else {
	        //ERR("armInfo unchanged");
		//goto out;
        //}
        ret = read_arm_state(arm_state_port, &armState);
	if(ret>0) {
	        DBG("armState changed");
	} else {
	        ERR("armState unchanged");
		goto out;
        }
        //ret = read_gripper_state(gripper_state_port, &gripperState);
	//if(ret>0) {
	        //DBG("gripperState changed");
	//} else {
	        //ERR("gripperState unchanged");
		//goto out;
        //}
       
	/* get time */
	//TODO Threadsafe?
	now = time(NULL);
        /* put time in string */
	time_string = ctime(&now);
        /* View time */
        DBG("time: %s\n", time_string);

	/* Create hdf5 file and send it out */
	inf->dims[0] = 1;
	inf->dims[1] = 1;
	setDataspaceId(inf, 1, NULL);
	createDatasetChar(inf, "/State/TimeStamp/timestamp", time_string);
        /* Add attribute */
	createStringAttribute(inf, "TimestampAttribute", time_string);
        closeDataspace(inf);
        closeDataset(inf);
	
	/* twist lin */
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/LinearVelocity/x", &twist.vel.x);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/LinearVelocity/y", &twist.vel.y);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/LinearVelocity/z", &twist.vel.z);
        closeDataspace(inf);
        closeDataset(inf);

        /* twist rot */
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/RotationalVelocity/x", &twist.rot.x);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/RotationalVelocity/y", &twist.rot.y);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/Twist/RotationalVelocity/z", &twist.rot.z);
        closeDataspace(inf);
        closeDataset(inf);

	/* Vector */
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/BaseCartesianPosition/Vector/x", &frame.p.x);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/BaseCartesianPosition/Vector/y", &frame.p.y);
        closeDataspace(inf);
        closeDataset(inf);
	
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/BaseCartesianPosition/Vector/z", &frame.p.z);
        closeDataspace(inf);
        closeDataset(inf);
	
	// Rotation
	inf->dims[0] = 9;
	inf->dims[1] =1;
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/BaseCartesianPosition/Rotation/rotation", frame.M.data);
        closeDataspace(inf);
        closeDataset(inf);

        /*
        // BaseMotorInfo
	inf->dims[0] = 4;
	inf->dims[1] =1;
	setDataspaceId(inf, 1, NULL);
	//createDatasetDouble(inf, "/State/BaseMotorInfo/Position", baseInfo.pos);
	createDatasetInteger(inf, "/State/BaseMotorInfo/Position", baseInfo.pos);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetInteger(inf, "/State/BaseMotorInfo/Velocity", baseInfo.vel);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetInteger(inf, "/State/BaseMotorInfo/Current", baseInfo.cur);
        closeDataspace(inf);
        closeDataset(inf);

        // ArmMotorInfo 
	inf->dims[0] = 5;
	inf->dims[1] =1;
	setDataspaceId(inf, 1, NULL);
	createDatasetInteger(inf, "/State/ArmMotorInfo/Position", armInfo.pos);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetInteger(inf, "/State/ArmMotorInfo/Velocity", armInfo.vel);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetInteger(inf, "/State/ArmMotorInfo/Current", armInfo.cur);
        closeDataspace(inf);
        closeDataset(inf);
        */

        // JointState
	inf->dims[0] = 5;
	inf->dims[1] =1;
	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/JointStates/Position", armState.pos);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/JointStates/Velocity", armState.vel);
        closeDataspace(inf);
        closeDataset(inf);

	setDataspaceId(inf, 1, NULL);
	createDatasetDouble(inf, "/State/JointStates/Effort", armState.eff);
        closeDataspace(inf);
        closeDataset(inf);

        // GripperState
	//inf->dims[0] = 1;
	//inf->dims[1] =1;
	//setDataspaceId(inf, 1, NULL);
	//createDatasetInteger(inf, "/State/GripperState", &gripperState);
        //closeDataspace(inf);
        //closeDataset(inf);
 out:

	H5Fclose(inf->hdf5Handle);
}

ubx_block_t h5fsnd_comp = {
        
	.name = "std_blocks/h5fddsmsender",
	.type = BLOCK_TYPE_COMPUTATION,
	.meta_data = h5fsnd_meta,
	.configs = h5fsnd_config,
	.ports = h5fsnd_ports,

	/* ops */
	.init = h5fsnd_init,
	.start = h5fsnd_start,
	.step = h5fsnd_step,
	.cleanup = h5fsnd_cleanup,
};

static int h5fsnd_mod_init(ubx_node_info_t* ni)
{
        DBG(" ");
        ubx_type_register(ni, &H5FDdsmSender_config_type);
        return ubx_block_register(ni, &h5fsnd_comp);
}

static void h5fsnd_mod_cleanup(ubx_node_info_t *ni)
{
        DBG(" ");
        ubx_block_unregister(ni, "std_blocks/h5fddsmsender");
}

UBX_MODULE_INIT(h5fsnd_mod_init)
UBX_MODULE_CLEANUP(h5fsnd_mod_cleanup)
