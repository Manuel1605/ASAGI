/* 
 * File:   numagrid.h
 * Author: root
 *
 * Created on 10. November 2013, 21:27
 */

#ifndef NUMAGRID_H
#define	NUMAGRID_H
#include "grid/grid.h"
#include "grid/constants.h"
#include "grid/numagridcontainer.h"
#include "grid/threadhandler.h"

#include "perf/counter.h"

#include "types/type.h"

#include "utils/logger.h"

#include <string>

namespace grid {

    /**
     * @brief Base class for a grid
     * 
     * A grid stores one level of detail of an adaptive grid
     */
    class NumaGrid : public Grid {	
    private:
        /** Number of cached blocks on each node */
	long m_blocksPerThread;
    public:
        NumaGrid(const GridContainer &container,
                unsigned int hint = asagi::Grid::NO_HINT);
        virtual ~NumaGrid();
        asagi::Grid::Error open(const char* filename);

    protected:
 	
	/**
	 * @return The number of blocks we should store on this Thread
	 */
	unsigned long getBlocksPerThread() const
	{
		return m_blocksPerThread;
	}
	
	
	/**
	 * @return The of blocks that are stored on one thread
	 */
	unsigned long getThreadBlockCount()
	{
		return (getLocalBlockCount()+ThreadHandler::tCount-1) / ThreadHandler::tCount;
	}
	
	/**
	 * @param id Local block id
	 * @return The thread rank, that stores the block
	 */
	int getBlockThreadRank(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id % getMPISize();
#else // ROUND_ROBIN
		return id / getThreadBlockCount();
#endif // ROUND_ROBIN
	}
	
	/**
	 * @param id Global block id
	 * @return The offset of the block on the thread
	 */
	unsigned long getBlockThreadOffset(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id / getMPISize();
#else // ROUND_ROBIN
		return id % getThreadBlockCount();
#endif // ROUND_ROBIN
	}
        unsigned long getThreadId(unsigned long block) {
            return ThreadHandler::threadHandle[(block / getThreadBlockCount())];
        }
        
        
	
	/**
	 * @param id Local block id
	 * @return The corresponding global id
	 */
/*	unsigned long getGlobalBlock(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id * getMPISize() + getMPIRank();
#else // ROUND_ROBIN
		return id + getMPIRank() * getLocalBlockCount();
#endif // ROUND_ROBIN
	}*/
    };
}

#endif	/* NUMAGRID_H */

