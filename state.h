struct youbot_state {
    
    struct timespec timestamp;
    struct kdl_frame base_cart_pos;
    double arm_jnt_pos[5];
};
