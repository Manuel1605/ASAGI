#include <asagi.h>
#include <stdio.h>

using namespace asagi;

int main (void)
{
	Grid* grid = Grid::create(Grid::FLOAT);
	
	if (!grid->open("../data/tohoku_1850m_bath.nc")) {
		printf("Could not load file\n");
		return 1;
	}
	
	printf("Range X: %f-%f\n", grid->getXMin(), grid->getXMax());
	printf("Range Y: %f-%f\n", grid->getYMin(), grid->getYMax());
	
	printf("Value at 5x10: %f\n", grid->getFloat(5, 10));
	printf("Value at 5x10.1: %f\n", grid->getFloat(5, 10.1));
	printf("Value at -1x-15.32: %f\n", grid->getFloat(-1, -15.32));
	
	return 0;
}