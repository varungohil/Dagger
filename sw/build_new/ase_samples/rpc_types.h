
#ifndef _RPC_TYPES_H_
#define _RPC_TYPES_H_

struct LoopBackArgs {
	uint64_t data;
};

struct AddArgs {
	uint64_t a;
	uint64_t b;
};

struct NumericalResult {
	uint64_t data;
};


#endif	// _RPC_TYPES_H_
