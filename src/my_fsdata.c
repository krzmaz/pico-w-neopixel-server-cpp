static const unsigned char data_res_json[] = {
	/* /res.json */
	0x2f, 0x72, 0x65, 0x73, 0x2e, 0x6a, 0x73, 0x6f, 0x6e, 0,
	0x48, 0x54, 0x54, 0x50, 0x2f, 0x31, 0x2e, 0x30, 0x20, 0x32, 
	0x30, 0x30, 0x20, 0x4f, 0x4b, 0xd, 0xa, 0x53, 0x65, 0x72, 
	0x76, 0x65, 0x72, 0x3a, 0x20, 0x6c, 0x77, 0x49, 0x50, 0x2f, 
	0x70, 0x72, 0x65, 0x2d, 0x30, 0x2e, 0x36, 0x20, 0x28, 0x68, 
	0x74, 0x74, 0x70, 0x3a, 0x2f, 0x2f, 0x77, 0x77, 0x77, 0x2e, 
	0x73, 0x69, 0x63, 0x73, 0x2e, 0x73, 0x65, 0x2f, 0x7e, 0x61, 
	0x64, 0x61, 0x6d, 0x2f, 0x6c, 0x77, 0x69, 0x70, 0x2f, 0x29, 
	0xd, 0xa, 0x43, 0x6f, 0x6e, 0x74, 0x65, 0x6e, 0x74, 0x2d, 
	0x74, 0x79, 0x70, 0x65, 0x3a, 0x20, 0x74, 0x65, 0x78, 0x74, 
	0x2f, 0x6a, 0x73, 0x6f, 0x6e, 0xd, 0xa, 0xd, 0xa, 
	0x7b, 0x7d, };

const struct fsdata_file file_res_json[] = {{NULL, data_res_json, data_res_json + 10, sizeof(data_res_json) - 10, FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT}};

#define FS_ROOT file_res_json

#define FS_NUMFILES 1
