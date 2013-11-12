#include "numagrid.h"

#include "constants.h"

#ifdef PNG_ENABLED
#include "io/pngwriter.h"
#endif

#include "types/basictype.h"

#include <cstdlib>
#include <cmath>
#include <limits>

/**
 * @param container The container, this numagrid belongs to
 * @param hint Optimization hints
 */
grid::NumaGrid::NumaGrid(const GridContainer &container,
	unsigned int hint)
	: Grid(container, hint)
{
    m_blocksPerThread = 1;
}

grid::NumaGrid::~NumaGrid()
{
}

asagi::Grid::Error grid::NumaGrid::open(const char* filename){
    if(m_blocksPerThread<0)
        m_blocksPerThread=(getBlocksPerNode()+(ThreadHandler::tCount-1))/ThreadHandler::tCount;
    return Grid::open(filename);
}