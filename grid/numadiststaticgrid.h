/* 
 * File:   numadiststaticgrid.h
 * Author: root
 *
 * Created on 12. November 2013, 03:13
 */

#ifndef NUMADISTSTATICGRID_H
#define	NUMADISTSTATICGRID_H

#include "numalocalcachegrid.h"
#include "numalocalstaticgrid.h"

#ifndef THREADSAFETY
#include <mutex>
#endif // THREADSAFETY

#include "blocks/blockmanager.h"

namespace grid
{

/**
 * Simple grid implementation, that distributes the grid at the beginning
 * across all MPI tasks. If a block is not available, it is transfered via
 * MPI and stored in a cache.
 */
class NumaDistStaticGrid : public NumaLocalStaticGrid, public NumaLocalCacheGrid
{
private:
    /** MPI window for communication */
	MPI_Win m_window;
public:
	NumaDistStaticGrid(const NumaGridContainer &container,
		unsigned int hint = asagi::Grid::NO_HINT);
	virtual ~NumaDistStaticGrid();
	
protected:
	asagi::Grid::Error init();
	
	void getAt(void* buf, types::Type::converter_t converter,
		unsigned long x, unsigned long y = 0, unsigned long z = 0);

	void getBlock(unsigned long block,
		long oldBlock,
		unsigned long cacheIndex,
		unsigned char* cache);

};

}

#endif	/* NUMADISTSTATICGRID_H */

