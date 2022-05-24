
#ifndef _RPC_TYPES_H_
#define _RPC_TYPES_H_

struct GetRequest {
	uint32_t timestamp;
	char key[320000];
};

struct GetResponse {
	uint32_t timestamp;
	char value[320000];
};


#endif	// _RPC_TYPES_H_
