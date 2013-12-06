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

#ifndef NUMADISTSTATICGRID_H
#define	NUMADISTSTATICGRID_H

#include "numalocalcachegrid.h"
#include "numalocalstaticgrid.h"
#include "threadhandler.h"
#include <mutex>
#include <map>

#include "allocator/mpiallocator.h"
#include "types/type.h"
#include "blocks/blockmanager.h"

namespace grid
{

/**
 * Simple grid implementation, that distributes the grid at the beginning
 * across all MPI tasks. If a block is not available, it is transfered via
 * MPI, or via memcpy and stored in a cache.
 */
class NumaDistStaticGrid : public NumaLocalStaticGrid, public NumaLocalCacheGrid
{
         
private:
        /** ID of the grid. For Multilevelsupport */
        unsigned int m_id;
public:

	NumaDistStaticGrid(const GridContainer &container, ThreadHandler &threadHandle,
		unsigned int hint = asagi::Grid::NO_HINT, unsigned int id=0);
	virtual ~NumaDistStaticGrid();
	
protected:
	asagi::Grid::Error init();
	
	void getAt(void* buf, types::Type::converter_t converter,
		unsigned long x, unsigned long y = 0, unsigned long z = 0);

	void getBlock(unsigned long block,
		long oldBlock,
		unsigned long cacheIndex,
		unsigned char* cache);
        
        /**
	 * We can free all netCDF related resources, because we use MPI to
	 * transfer blocks
	 */
	bool keepFileOpen() const
	{
		return false;
	}

};

}

#endif	/* NUMADISTSTATICGRID_H */

