
struct H5FDdsmSender_config {

	char ip[20];
	std::string port;
	std::string file_name;
	std::vector<std::string> groups;
	std::vector<std::array<std::string, 2>> dataset_char;
	std::vector<std::array<std::string, 2>> dataset_double;
	std::vector<std::array<std::string, 2>> dataset_integer;

};
