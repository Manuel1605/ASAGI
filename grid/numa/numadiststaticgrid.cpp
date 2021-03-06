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

#include "numadiststaticgrid.h"

#include <cassert>
#include <malloc.h>
#include <stdlib.h>

#include "allocator/mpiallocator.h"
#include "types/type.h"
#include "threadhandler.h"


/**
 * @see StaticGrid::StaticGrid()
 */

grid::NumaDistStaticGrid::NumaDistStaticGrid(const GridContainer &container, ThreadHandler &threadHandle,
        unsigned int hint, unsigned int id)
: Grid(container, threadHandle, hint ),
NumaLocalStaticGrid(container, threadHandle, hint, id, allocator::MPIAllocator<unsigned char>::allocator),
NumaLocalCacheGrid(container, threadHandle, hint, id ),
m_id(id){
}

grid::NumaDistStaticGrid::~NumaDistStaticGrid() {
}

asagi::Grid::Error grid::NumaDistStaticGrid::init() {
    unsigned long blockSize = getTotalBlockSize();
    unsigned long masterBlockCount = getLocalBlockCount();
    asagi::Grid::Error error;

    // Create the local cache
    error = NumaLocalCacheGrid::init();
    if (error != asagi::Grid::SUCCESS)
        return error;

    // Distribute the blocks
    error = NumaLocalStaticGrid::init();
    if (error != asagi::Grid::SUCCESS)
        return error;
    
    // Create the mpi window for distributed blocks
    if(pthread_equal(m_threadHandle.getMasterthreadId(), pthread_self())){
        if (MPI_Win_create(m_threadHandle.getStaticPtr(m_threadHandle.getMasterthreadId(),m_id),
                getType().getSize() * blockSize * masterBlockCount,
                getType().getSize(),
                MPI_INFO_NULL,
                getMPICommunicator(),
                &m_threadHandle.mpiWindow) != MPI_SUCCESS)
            return asagi::Grid::MPI_ERROR;
    }
    return asagi::Grid::SUCCESS;
}

void grid::NumaDistStaticGrid::getAt(void* buf, types::Type::converter_t converter,
        unsigned long x, unsigned long y, unsigned long z) {
    unsigned long block = getBlockByCoords(x, y, z);
    int remoteMPIRank = getBlockRank(block);
    unsigned long remoteThreadId = getThreadId(block);
    if (remoteMPIRank == getMPIRank() && pthread_equal(remoteThreadId, pthread_self())) {
        // Nice, this is a block where we are the master
        NumaLocalStaticGrid::getAt(buf, converter, x, y, z);
        return;
    }
    // This function will call getBlock, if we need to transfer the block
    NumaLocalCacheGrid::getAt(buf, converter, x, y, z);
}

/**
 * Transfer the block form the remote rank, that holds it,
 * or if it´s the same rank, copy it by using memcpy.
 */
void grid::NumaDistStaticGrid::getBlock(unsigned long block,
        long oldBlock,
        unsigned long cacheIndex,
        unsigned char *cache) {
    unsigned long blockSize = getTotalBlockSize();
    int remoteRank = getBlockRank(block);
    incCounter(perf::Counter::MPI);
    int mpiResult;
     
    if (remoteRank == getMPIRank()) {
        //The block is located in the same NUMA Domain, but in the memspace of another thread.
        pthread_t remoteId = getThreadId(block);
        size_t offset = getType().getSize()*blockSize*getBlockThreadOffset(block);
        //copy the block
     //   std::cout << "Memcpy Thread: " << remoteId << " Pointer: " << &(m_threadHandle.getStaticPtr(remoteId, m_id));
        memcpy(cache, m_threadHandle.getStaticPtr(remoteId, m_id) + offset, getType().getSize()*blockSize);
    } 
    else {
        
        //This section is critical. Only one Thread is allowed to access.
        //TODO: Find a better solution than pthread mutex.
        unsigned long offset = getBlockOffset(block);
        NDBG_UNUSED(mpiResult);
        mpiResult = m_threadHandle.getBlock(cache,
                blockSize,
                getType().getMPIType(),
                remoteRank,
                offset * blockSize,
                blockSize,
                getType().getMPIType(),
                m_threadHandle.mpiWindow);
        assert(mpiResult == MPI_SUCCESS);

    }



}
