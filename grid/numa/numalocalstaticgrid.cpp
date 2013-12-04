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

#include "numalocalstaticgrid.h"
#include "threadhandler.h"
#include <pthread.h>
#include <stdio.h>


unsigned int g_thread;

/**
 * @see Grid::Grid()
 */

grid::NumaLocalStaticGrid::NumaLocalStaticGrid(const GridContainer &container,
        unsigned int hint, unsigned int id, 
        const allocator::Allocator<unsigned char> &allocator)
: Grid(container, hint),
        m_id(id),
 m_allocator(allocator) {
    m_data = 0L;
}

grid::NumaLocalStaticGrid::~NumaLocalStaticGrid() {
   // if(pthread_equal(ThreadHandler::masterthreadId, pthread_self()))
    //    m_allocator.free(ThreadHandler::staticPtr[ThreadHandler::masterthreadId][m_id]);
}

asagi::Grid::Error grid::NumaLocalStaticGrid::init() {
    if(pthread_equal(ThreadHandler::masterthreadId, pthread_self()))
        g_thread=0;
    unsigned long blockSize = getTotalBlockSize();
    size_t block[3];
    unsigned long masterBlockCount = getThreadBlockCount()*ThreadHandler::tCount;
    
    //the first thread allocates the memory.
    if (g_thread==0) {
        asagi::Grid::Error error;
        error = m_allocator.allocate(getType().getSize() * blockSize * masterBlockCount, m_data);
        if (error != asagi::Grid::SUCCESS)
            return error;
        // Load the blocks from the file, which we control
      /*  for (unsigned long i = 0; i < getLocalBlockCount(); i++) {
            if (getGlobalBlock(i) >= getBlockCount()){
                // Last process(es) may control less blocks
                break;
            }
            // Get x, y and z coordinates of the block
            getBlockPos(getGlobalBlock(i), block);
            
            // Get x, y and z coordinates of the first value in the block
            for (unsigned char j = 0; j < 3; j++)
                block[j] *= getBlockSize(j);
            getType().load(getInputFile(),
                    block, getBlockSize(),
                    &m_data[getType().getSize() * blockSize * i]);
        }*/
        ThreadHandler::staticPtr[pthread_self()][m_id] = m_data;
    }
    else {
        //The memory was already allocated by the masterthread.
   
                //Simply shift the Pointer in the right space.
                m_data = (ThreadHandler::staticPtr[ThreadHandler::masterthreadId])[m_id] + (getType().getSize() * blockSize * g_thread * getThreadBlockCount());
                (ThreadHandler::staticPtr[pthread_self()])[m_id]=m_data;
    }
    // Load the blocks from the file, which we control
        for (unsigned long i = g_thread*getThreadBlockCount(); i < (g_thread*getThreadBlockCount()+getThreadBlockCount()); i++) {
            if (getGlobalBlock(i) >= getBlockCount()){
                // Last process(es) may control less blocks
                break;
            }
            // Get x, y and z coordinates of the block
            getBlockPos(getGlobalBlock(i), block);
            
            // Get x, y and z coordinates of the first value in the block
            for (unsigned char j = 0; j < 3; j++)
                block[j] *= getBlockSize(j);
            getType().load(getInputFile(),
                    block, getBlockSize(),
                    &m_data[getType().getSize() * blockSize * (i-g_thread*getThreadBlockCount())]);
        }
    
    //I have finished. Set the counter for the next Thread.
    g_thread++;
    return asagi::Grid::SUCCESS;
}

void grid::NumaLocalStaticGrid::getAt(void* buf, types::Type::converter_t converter,
        long unsigned int x, long unsigned int y, long unsigned int z) {

    unsigned long blockSize = getTotalBlockSize();
    unsigned long block = getBlockByCoords(x, y, z);
    int remoteMPIRank = getBlockRank(block);
    unsigned long remoteThreadId = getThreadId(block);
    NDBG_UNUSED(remoteMPIRank);
    unsigned long offset = getBlockThreadOffset(block);

    // Offset inside the block
    x %= getBlockSize(0);
    y %= getBlockSize(1);
    z %= getBlockSize(2);
    assert(remoteMPIRank == getMPIRank());
    assert(remoteThreadId == pthread_self());
    if(pthread_equal(remoteThreadId, pthread_self())){
        (getType().*converter)(&m_data[getType().getSize() *
                (blockSize * offset // jump to the correct block
                + (z * getBlockSize(1) + y) * getBlockSize(0) + x) // correct value inside the block
                ],
                buf);
    }
}
