/**
 * @file
 *  This file is part of ASAGI.
 * 
 *  ASAGI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  ASAGI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with ASAGI.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Diese Datei ist Teil von ASAGI.
 *
 *  ASAGI ist Freie Software: Sie koennen es unter den Bedingungen
 *  der GNU General Public License, wie von der Free Software Foundation,
 *  Version 3 der Lizenz oder (nach Ihrer Option) jeder spaeteren
 *  veroeffentlichten Version, weiterverbreiten und/oder modifizieren.
 *
 *  ASAGI wird in der Hoffnung, dass es nuetzlich sein wird, aber
 *  OHNE JEDE GEWAEHELEISTUNG, bereitgestellt; sogar ohne die implizite
 *  Gewaehrleistung der MARKTFAEHIGKEIT oder EIGNUNG FUER EINEN BESTIMMTEN
 *  ZWECK. Siehe die GNU General Public License fuer weitere Details.
 *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.
 * 
 * @copyright 2012-2013 Sebastian Rettenberger <rettenbs@in.tum.de>
 * @copyright 2013-2014 Manuel Fasching <manuel.fasching@tum.de>
 */

#ifndef NUMAGRID_H
#define	NUMAGRID_H
#include "grid/grid.h"
#include "grid/constants.h"
#include "numagridcontainer.h"
#include "threadhandler.h"

#include "perf/counter.h"

#include "types/type.h"

#include "utils/logger.h"

#include <string>

namespace grid {

    /**
     * @brief Base class for a NumGrid
     * 
     * A grid stores one level of detail of an adaptive grid
     * It is derived from Grid, and adds the Numa specific Functions
     */
    class NumaGrid : public Grid {	
    private:
        /** Number of cached blocks on each thread */
	//long m_blocksPerThread;
    public:
        NumaGrid(const GridContainer &container,
                unsigned int hint = asagi::Grid::NO_HINT);
        virtual ~NumaGrid();
        //asagi::Grid::Error open(const char* filename);

    protected:
 	
	/**
	 * @return The number of blocks we should store on this Thread
	 */
	unsigned long getBlocksPerThread() const
	{
		//return m_blocksPerThread;
                return (getBlocksPerNode()+(ThreadHandler::tCount-1))/ThreadHandler::tCount;
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
	 * @return The offset of the block in the memspace of the thread
	 */
	unsigned long getBlockThreadOffset(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id / getMPISize();
#else // ROUND_ROBIN
		return id % getThreadBlockCount();
#endif // ROUND_ROBIN
	}
        
        /**
	 * @param block Global block id
	 * @return The ThreadID which holds the block
	 */
        unsigned long getThreadId(unsigned long block) {
            return ThreadHandler::threadHandle[(block / getThreadBlockCount())];
        }
        
        
};
}

#endif	/* NUMAGRID_H */

