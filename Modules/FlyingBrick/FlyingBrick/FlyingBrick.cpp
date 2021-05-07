#include "MSFS/MSFS.h"

#include "FlyingBrick.h"
#include <iostream>
#include <fstream>

extern "C" MODULE_EXPORT void test(void)
{
	// TODO
}

extern "C" MSFS_CALLBACK void module_init(void)
{
	std::ofstream log("\\work\\FlyingBrick.wasm.log", std::ios_base::app);

	log << "Hello there, module_init here." << std::endl;

	std::cout << "====== FlyingBrick: Let's see if writing to stdout goes anywhere" << std::endl;
}

extern "C" MSFS_CALLBACK void module_deinit(void)
{
	std::ofstream log("\\work\\FlyingBrick.wasm.log", std::ios_base::app);

	log << "Hello there, module_deinit here." << std::endl;
}