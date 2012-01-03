#include <database.h>

#include "grid.h"

grid_handle grid_load(const char* filename)
{
	Grid* grid = new Grid();
	
	if (!grid->open(filename)) {
		delete grid;
		return 0L;
	}
	
	return static_cast<grid_handle>(grid);
}

unsigned long grid_x(grid_handle handle)
{
	return static_cast<Grid*>(handle)->getXDim();
}

unsigned long grid_y(grid_handle handle)
{
	return static_cast<Grid*>(handle)->getYDim();
}

float grid_get_value(grid_handle handle, unsigned long x, unsigned long y)
{
	return static_cast<Grid*>(handle)->get(x, y);
}