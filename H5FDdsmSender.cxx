/*
 * A fblock that sends a HDF5 file to a server
 */

#include <stdio.h>
#include <time.h>
#include <hdf5.h>
//#include "/home/evert/local/include/hdf5.h"
#include <cstdlib>

#include "ubx.h"

//#include "/home/evert/h5fddsm/h5fddsm-0.9.9/Testing/H5FDdsmTest.h"
#include <H5FDdsmTest.h>
//#include "/home/evert/h5fddsm/h5fddsm-0.9.9/src/H5FDdsm.h"
#include <H5FDdsm.h>

#include "vector.h"
#include "rotation.h"
#include "frame.h"
#include "twist.h"
#include "state.h"

#define FILE "youbot.h5"

/* function block meta-data
 * used by higher level functions.
 */
char h5fsnd_meta[] =
        "{ doc='A hdf5-data sender function block',"
        "  license='LGPL',"
	"  real-time=true,"
	"}";

/* configuration
 * upon cloning the following happens:
 *   - value.type is resolved
 *   - value.data will point to a buffer of size value.len*value.type->size
 *
 * if an array is required, then .value = { .len=<LENGTH> } can be used.
 */
ubx_config_t h5fsnd_config[] = {
        
	{ .name="port_config", .type_name = "unsigned int" },
	{ .name="ip", .type_name="char" },

        { NULL },
};


/* we need a trigger and data from the robot */
ubx_port_t h5fsnd_ports[] = {

	{ .name="base_msr_twist", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_twist" },
	{ .name="base_msr_odom", .attrs=PORT_DIR_IN, .in_type_name="struct kdl_frame" },

        { NULL },
        { NULL },
};

struct H5FDdsmSender_info {

	MPI_Comm comm;
	H5FDdsmManager* dsmManager;
	//TODO These arguments? hardcode?
	//senderInit(argc, argv, dsmManager, &comm);
	//senderInit("", 0, dsmManager, &comm);
	
	hid_t       file_id, group_id, dataset_id, dataspace_id, hdf5Handle, fapl;  /* identifiers */
	hsize_t     dims[2];
	herr_t      status;
	time_t      now;
	char*       time_string;
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


void createGroup(struct H5FDdsmSender_info* sinfo, const char* name);

void createGroups(struct H5FDdsmSender_info* sinfo);

void init(struct H5FDdsmSender_info* sinfo) {

	//randomize(sinfo);
		
        sinfo->comm = MPI_COMM_WORLD;
        sinfo->dsmManager = new H5FDdsmManager();
	//senderInit("", 0, sinfo->dsmManager, &sinfo->comm);
	senderInit(NULL, 0, sinfo->dsmManager, &sinfo->comm);
	// Create Array
	//int array[3] = { 1, 2, 3 };
	//int read_array[3];
	//Unused
	//hsize_t arraySize = 3;
	// Set file access property list for DSM
	sinfo->fapl = H5Pcreate(H5P_FILE_ACCESS);
	// Use DSM driver
	H5Pset_fapl_dsm(sinfo->fapl, sinfo->comm, NULL, 0);
	// Create DSM
	sinfo->hdf5Handle = H5Fcreate(FILE, H5F_ACC_TRUNC, H5P_DEFAULT, sinfo->fapl);
	// Close file access property list
	H5Pclose(sinfo->fapl);

	createGroups(sinfo);
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

/*
void createGroup(hid_t handle, char* name) {
	
	group_id = H5Gcreate2(handle, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	H5Gclose(group_id);
}

*/

void createGroup(struct H5FDdsmSender_info* sinfo, const char* name) {
        
	sinfo->group_id = H5Gcreate2(sinfo->hdf5Handle, name, H5P_DEFAULT, H5P_DEFAULT, H5P_DEFAULT);
	H5Gclose(sinfo->group_id);
}

void setDataspaceId(struct H5FDdsmSender_info* sinfo, int rank, hsize_t* current_dims, hsize_t* max_dims) {
	
	sinfo->dataspace_id = H5Screate_simple(rank, current_dims, max_dims);
	//sinfo->dataspace_id = H5Screate_simple(rank, current_dims);
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

	DBG(" ");
        if ((c->private_data = calloc(1, sizeof(struct H5FDdsmSender_info)))==NULL) {

                ERR("Failed to alloc memory");
                ret=EOUTOFMEM;
                goto out;
        }

	struct H5FDdsmSender_info* inf;

	inf=(struct H5FDdsmSender_info*) c->private_data;
	
	// initiate
	init(inf);


 out:
        return ret;
}

static void h5fsnd_cleanup(ubx_block_t *c) {
        
	struct H5FDdsmSender_info* inf;
        inf=(struct H5FDdsmSender_info*) c->private_data;
        DBG(" ");
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
	struct kdl_frame fr;

        inf=(struct H5FDdsmSender_info*) c->private_data;

	//TODO goto off for testing

	
	// Receive data from robot
        ubx_port_t* twist_port = ubx_port_get(c, "base_msr_twist");
	ubx_port_t* frame_port = ubx_port_get(c, "base_msr_odom");
	ret = read_kdl_twist(twist_port, &twist);
	if(ret>0) {
	        DBG("twist changed");
	} else {
	        ERR("twist unchanged");
		//goto out;
	}
	ret = read_kdl_frame(frame_port, &fr);
	if(ret>0) {
	        DBG("frame changed");
	} else {
	        ERR("frame unchanged");
		//goto out;
        }

        // Randomdata
	twist = randomTwist();
	fr = randomFrame();
	// get time
	//TODO Threadsafe?
	inf->now = time(NULL);
	inf->time_string = ctime(&inf->now);

	// Create hdf5 file and send it out!
	// Timestamp
	inf->dims[0] = 1;
	inf->dims[1] =1;
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetChar(inf->hdf5Handle, "/State/Timestamp/timestamp", inf->dataspace_id,
	        inf->time_string);
	
	// Arm joint position
	//inf.dims[0] = 5;
	//inf.dims[1] =1;
	//
	//inf.dataspace_id = setDataspaceId(2, inf.dims, NULL);
	//inf.dataset_id = createDatasetDouble(inf.hdf5Handle, "/State/Timestamp/arm_jnt_pos", inf.dataspace_id, arm_jnt_pos);

	// twist lin
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/x", inf->dataspace_id,
	        &twist.vel.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/y", inf->dataspace_id,
	        &twist.vel.y);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/LinearVelocity/z", inf->dataspace_id,
	        &twist.vel.z);

        // twist rot
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/x", inf->dataspace_id,
	        &twist.rot.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/y", inf->dataspace_id,
	        &twist.rot.y);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/Twist/RotationalVelocity/z", inf->dataspace_id,
	        &twist.rot.z);
	// Vector
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/x", inf->dataspace_id, 
	        &fr.p.x);

	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/y", inf->dataspace_id,
	        &fr.p.y);
	
	setDataspaceId(inf, 1, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Vector/z", inf->dataspace_id,
	        &fr.p.z);
	
	// Rotation
	inf->dims[0] = 9;
	inf->dims[1] =1;
	setDataspaceId(inf, 2, inf->dims, NULL);
	inf->dataset_id = createDatasetDouble(inf->hdf5Handle, "/State/BaseCartesianPosition/Rotation/rotation", inf->dataspace_id,
	        fr.M.data);

 //out:

	H5Fclose(inf->hdf5Handle);
	
	senderFinalize(inf->dsmManager, &inf->comm);
}

ubx_block_t h5fsnd_comp = {
        
	//.name = "H5FDdsmSender/H5FDdseSender",
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
        //ubx_block_unregister(ni, "H5FDdseSender/H5FDdseSender");
        ubx_block_unregister(ni, "std_blocks/h5fddsesender");
}


module_init(h5fsnd_mod_init)
module_cleanup(h5fsnd_mod_cleanup)

/*
int main(int argc, char * argv[]) {

	init();
	createGroups();

	// Timestamp
	dataspace_id = setDataspaceId(1, 1, NULL);
	dataset_id = createDatasetChar(hdf5Handle, "/State/Timestamp/timestamp", dataspace_id, time_string);
	
	// Arm joint position
	dims[0] = 5;
	dims[1] =1;
	
	dataspace_id = setDataspaceId(2, dims, NULL);
	dataset_id = createDatasetDouble(hdf5Handle, "/State/Timestamp/arm_jnt_pos", dataspace_id, arm_jnt_pos);

	// Vector
	dataspace_id = setDataspaceId(1, 1, NULL);
	dataset_id = createDatasetDouble(hdf5Handle, "/State/BaceCartesianPosition/Vector/x", dataspace_id, kdl_vector_x);

	dataspace_id = setDataspaceId(1, 1, NULL);
	dataset_id = createDatasetDouble(hdf5Handle, "/State/BaceCartesianPosition/Vector/y", dataspace_id, kdl_vector_y);
	
	dataspace_id = setDataspaceId(1, 1, NULL);
	dataset_id = createDatasetDouble(hdf5Handle, "/State/BaceCartesianPosition/Vector/z", dataspace_id, kdl_vector_z);
	
	// Rotation
	dims[0] = 9;
	dims[1] =1;
	dataspace_id = setDataspaceId(2, dims, NULL);
	dataset_id = createDatasetDouble(hdf5Handle, "/State/BaseCartesianPosition/Rotation/rotation", dataspace_id, kdl_rotation_data);

	H5Fclose(hdf5Handle);
	
	senderFinalize(dsmManager, &comm);
	delete dsmManager;
	return(EXIT_SUCCESS);
}
*/
