/*
 * A fblock that sends a HDF5 file to a server
 */

#include <stdio.h>
#include <time.h>
#include <hdf5.h>
#include <cstdlib>

#include "ubx.h"

#include <H5FDdsmTest.h>
#include <H5FDdsm.h>

#include <kdl.h>

#define FILE "youbot.h5"

/* function block meta-data
 * used by higher level functions.
 */
char h5fsnd_meta[] =
        "{ doc='A hdf5-data sender function block',"
        "  license='LGPL',"
	"  real-time=true?,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t h5fsnd_config[] = {
        
        //TODO Load configuration
	//{.name="port", .type_name="int"}, 
	{.name="port", .type_name="char", .value={.len=10}}, 
	{.name="ip", .type_name="char", .value={.len=11}}, /* car[11] */

        {NULL},
};


/* we need twist and frame data from the robot */
ubx_port_t h5fsnd_ports[] = {

	{ .name="base_msr_twist", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_twist" },
	{ .name="base_msr_odom", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_frame" },

        { NULL },
        { NULL },
};

struct H5FDdsmSender_info {

        //TODO Add kdl structs to this to get the right data inside h5 file
	MPI_Comm comm;
	H5FDdsmManager* dsmManager;
	
	hid_t       file_id, group_id, dataset_id, dataspace_id, hdf5Handle, fapl;  /* identifiers */
	hsize_t     dims[2];
	herr_t      status;
	time_t      now;
	char*       time_string;
        char*       ip;
        int         port;
        kdl_twist   twist;
        kdl_frame   frame;

};

/* convenience functions to read/write from the ports */
def_read_fun(read_kdl_twist, struct kdl_twist)

def_read_fun(read_kdl_frame, struct kdl_frame)


/* To get some values if we want to test without a robot */
struct H5FDdsmSender_info* randomize(struct H5FDdsmSender_info* info) {

        //TODO Implement
        //ret.base_cart_pos.p.x = 1.2;
        //ret.base_cart_pos.p.y = 3.2;
        //ret.base_cart_pos.p.z = 5.2;
//
        //int i;
        //for (i=0;i<9;i++)
                //ret.base_cart_pos.M.data[i] = i + 1.1;
//
        //for (i=0;i<5;i++)
                //ret.arm_jnt_pos[i] = i + 1.2;
        return info;
}

/*
struct kdl_twist randomTwist() {
        
	struct kdl_twist ret;
	ret.vel.x = 1.324;
	ret.vel.y = 2.224;
	ret.vel.z = 9.724;
	ret.rot.x = 8.224;
	ret.rot.y = 2.324;
	ret.rot.z = 4.724;

	return ret;
}
*/

void randomTwist(struct H5FDdsmSender_info* info) {
        
        info->twist.vel.x = 1.234;
        info->twist.vel.y = 1.234;
        info->twist.vel.z = 1.234;
	info->twist.rot.x = 3.53;
	info->twist.rot.y = 3.53;
	info->twist.rot.z = 3.53;

}

/*
struct kdl_frame randomFrame() {
        
	struct kdl_frame ret;
	ret.p.x = 1.322;
	ret.p.y = 1.436;
	ret.p.z = 3.245;
	ret.M.data[0] = 3.5343;
	ret.M.data[1] = 4.235;
	ret.M.data[2] = 4.523;
	ret.M.data[3] = 4.351;
	ret.M.data[4] = 6.521;
	ret.M.data[5] = 5.234;
	ret.M.data[6] = 2.523;
	ret.M.data[7] = 1.623;
	ret.M.data[8] = 2.632;

	return ret;
}
*/

void randomFrame(struct H5FDdsmSender_info* info) {

	info->frame.p.x = 1.322;
	info->frame.p.y = 1.436;
	info->frame.p.z = 3.245;
	info->frame.M.data[0] = 3.5343;
	info->frame.M.data[1] = 4.235;
	info->frame.M.data[2] = 4.523;
	info->frame.M.data[3] = 4.351;
	info->frame.M.data[4] = 6.521;
	info->frame.M.data[5] = 5.234;
	info->frame.M.data[6] = 2.523;
	info->frame.M.data[7] = 1.623;
	info->frame.M.data[8] = 2.632;

}

void createGroup(struct H5FDdsmSender_info* sinfo, const char* name);

void createGroups(struct H5FDdsmSender_info* sinfo);

void init(struct H5FDdsmSender_info* sinfo) {

	//randomize(sinfo);
		
        sinfo->comm = MPI_COMM_WORLD;
        sinfo->dsmManager = new H5FDdsmManager();
        //TODO Set the ip and port from config
	//senderInit(NULL, 0, sinfo->dsmManager, &sinfo->comm);
	//char ip[] = "10.33.173.147";
        //char ip[] = "10.33.174.62";
	senderInitIp(NULL, 0, sinfo->port, sinfo->ip, sinfo->dsmManager, &sinfo->comm);
        /* We made a new function which has IP and port as argument */
	//senderInitIp(NULL, 0, 22000, ip, sinfo->dsmManager, &sinfo->comm);
        //TODO We breakup this function to be able to change the hostname 
        //and port and to be able to resend after first send, see H5FDdsmTest.cxx
	/*
	H5FDdsmInt32   nlocalprocs, rank;
        MPI_Init(NULL, 0);
        MPI_Comm_rank(&sinfo->comm, &rank);
	MPI_Comm_size(&sinfo->comm, &nlocalprocs);
	sinfo->dsmManager->SetServerHostName("192.168.10.171"); 
	sinfo->dsmManager->SetServerPort(22000); 
	sinfo->dsmManager->SetMpiComm(&sinfo->comm);
	sinfo->dsmManager->SetIsServer(H5FD_DSM_FALSE);
	sinfo->dsmManager->Create();
	H5FD_dsm_set_manager(sinfo->dsmManager);
	*/

/*
	// Set file access property list for DSM
	sinfo->fapl = H5Pcreate(H5P_FILE_ACCESS);
	// Use DSM driver
	H5Pset_fapl_dsm(sinfo->fapl, sinfo->comm, NULL, 0);
	// Create DSM
	sinfo->hdf5Handle = H5Fcreate(FILE, H5F_ACC_TRUNC, H5P_DEFAULT, sinfo->fapl);
	// Close file access property list
	H5Pclose(sinfo->fapl);
        // Create all groups
	createGroups(sinfo);
*/
}

void createGroups(struct H5FDdsmSender_info* sinfo) {
	
	createGroup(sinfo, "/State");
	createGroup(sinfo, "/State/TimeStamp");
	//createGroup(sinfo.hdf5Handle, "/State/ArmJointPosition");
	createGroup(sinfo, "State/Twist");
	createGroup(sinfo, "State/Twist/RotationalVelocity");
	createGroup(sinfo, "State/Twist/LinearVelocity");
	createGroup(sinfo, "/State/BaseCartesianPosition");
	createGroup(sinfo, "/State/BaseCartesianPosition/Vector");
	createGroup(sinfo, "/State/BaseCartesianPosition/Rotation");
}

void createGroup(struct H5FDdsmSender_info* sinfo, const char* name) {
        
	sinfo->group_id = H5Gcreate2(sinfo->hdf5Handle, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	H5Gclose(sinfo->group_id);
}

void setDataspaceId(struct H5FDdsmSender_info* sinfo, int rank, hsize_t* current_dims, hsize_t* max_dims) {
	
	sinfo->dataspace_id = H5Screate_simple(rank, current_dims, max_dims);
}

hid_t createDatasetChar(hid_t handle, const char* name, hid_t dataspace_id, char* data) {
	
	hid_t dataset_id = H5Dcreate2(handle, name, H5T_NATIVE_CHAR, dataspace_id, H5P_DEFAULT, H5P_DEFAULT, 
	        H5P_DEFAULT);
	H5Dwrite(dataset_id, H5T_NATIVE_CHAR, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
	H5Sclose(dataspace_id);
	H5Dclose(dataset_id);
	return dataset_id;
}

hid_t createDatasetDouble(hid_t handle, const char* name, hid_t dataspace_id, double* data) {
	
	hid_t dataset_id = H5Dcreate2(handle, name, H5T_IEEE_F64BE, dataspace_id, H5P_DEFAULT, H5P_DEFAULT,
	        H5P_DEFAULT);
	H5Dwrite(dataset_id, H5T_IEEE_F64BE, H5S_ALL, H5S_ALL, H5P_DEFAULT, data);
	H5Sclose(dataspace_id);
	H5Dclose(dataset_id);
	return dataset_id;
}

static int h5fsnd_init(ubx_block_t *c) {
	
	int ret=0;
        char* port_string;

	DBG(" ");
        if ((c->private_data = calloc(1, sizeof(struct H5FDdsmSender_info)))==NULL) {

                ERR("Failed to alloc memory");
                ret=EOUTOFMEM;
                goto out;
        }

	struct H5FDdsmSender_info* inf;

	inf=(struct H5FDdsmSender_info*) c->private_data;

        /* get config and put inside inf */
        unsigned int ip_len, port_len;
        inf->ip = (char *) ubx_config_get_data_ptr(c, "ip", &ip_len);
        //printf("%s\n", inf->ip);
        port_string = (char *) ubx_config_get_data_ptr(c, "port", &port_len);
        //printf("%s\n", port_string);
        sscanf(port_string, "%u", &inf->port);
        //printf("%d\n", inf->port);
	// initiate
	init(inf);


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
	//struct kdl_twist twist;
	//struct kdl_frame fr;

        inf=(struct H5FDdsmSender_info*) c->private_data;

	// Set file access property list for DSM
	inf->fapl = H5Pcreate(H5P_FILE_ACCESS);
	// Use DSM driver
	H5Pset_fapl_dsm(inf->fapl, inf->comm, NULL, 0);
	// Create DSM
	inf->hdf5Handle = H5Fcreate(FILE, H5F_ACC_TRUNC, H5P_DEFAULT, inf->fapl);
	// Close file access property list
	H5Pclose(inf->fapl);
        // Create all groups
	createGroups(inf);

	//TODO goto off for testing

	
	// Receive data from robot
        ubx_port_t* twist_port = ubx_port_get(c, "base_msr_twist");
	ubx_port_t* frame_port = ubx_port_get(c, "base_msr_odom");
	ret = read_kdl_twist(twist_port, &inf->twist);
	if(ret>0) {
	        DBG("twist changed");
	} else {
	        ERR("twist unchanged");
		//goto out;
	}
	ret = read_kdl_frame(frame_port, &inf->frame);
	if(ret>0) {
	        DBG("frame changed");
	} else {
	        ERR("frame unchanged");
		//goto out;
        }

        // Randomdata
	//inf->twist = randomTwist();
	//inf->frame = randomFrame();
	randomTwist(inf);
	randomFrame(inf);
	// get time
	//TODO Threadsafe?
	inf->now = time(NULL);
        // put time in string
	inf->time_string = ctime(&inf->now);
        // View time
        //DBG("time: %s\n", inf->time_string);
        printf("time: %s\n", inf->time_string);

	// Create hdf5 file and send it out!
	// Timestamp
	inf->dims[0] = 24;
	inf->dims[1] = 1;
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetChar(inf->hdf5Handle, "/State/TimeStamp/timestamp", inf->dataspace_id,
	        inf->time_string);
	
	// Arm joint position
	//inf.dims[0] = 5;
	//inf.dims[1] =1;
	//
	//inf.dataspace_id = setDataspaceId(2, inf.dims, NULL);
	//inf.dataset_id = createDatasetDouble(inf.hdf5Handle, "/State/Timestamp/arm_jnt_pos", inf.dataspace_id, arm_jnt_pos);

	// twist lin
	inf->dims[0] = 1;
	inf->dims[1] = 1;
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/x", inf->dataspace_id,
	        &inf->twist.vel.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/y", inf->dataspace_id,
	        &inf->twist.vel.y);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/z", inf->dataspace_id,
	        &inf->twist.vel.z);

        // twist rot
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/x", inf->dataspace_id,
	        &inf->twist.rot.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/y", inf->dataspace_id,
	        &inf->twist.rot.y);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/z", inf->dataspace_id,
	        &inf->twist.rot.z);
	// Vector
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/x", inf->dataspace_id, 
	        &inf->frame.p.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/y", inf->dataspace_id,
	        &inf->frame.p.y);
	
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/z", inf->dataspace_id,
	        &inf->frame.p.z);
	
	// Rotation
	inf->dims[0] = 9;
	inf->dims[1] =1;
	setDataspaceId(inf, 2, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Rotation/rotation", inf->dataspace_id,
	        inf->frame.M.data);

 //out:

	H5Fclose(inf->hdf5Handle);
	
        // Don't finalize here!!
	//senderFinalize(inf->dsmManager, &inf->comm);

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
        return ubx_block_register(ni, &h5fsnd_comp);
}

static void h5fsnd_mod_cleanup(ubx_node_info_t *ni)
{
        DBG(" ");
        ubx_block_unregister(ni, "std_blocks/h5fddsmsender");
}


module_init(h5fsnd_mod_init)
module_cleanup(h5fsnd_mod_cleanup)
