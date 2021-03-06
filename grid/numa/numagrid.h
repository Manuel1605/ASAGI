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
#include "grid/constants.h"
#include "numagridcontainer.h"
#include "threadhandler.h"

#include "perf/counter.h"

#include "types/type.h"

#include "utils/logger.h"

#include <string>

namespace grid {

    class Grid{


public:
    /**
     * @brief Base class for a NumGrid
     * 
     * A grid stores one level of detail of an adaptive grid
     * It is derived from Grid, and adds the Numa specific Functions
     */

	/** Total number of elements in x, y and z dimension */
	 unsigned long m_dim[3];
	
	/** Offset of the grid */
	 double m_offset[3];
	
	/** Minimum possible coordinate in each dimension */
	 double m_min[3];
	/** Maximum possible coordinate in each dimension */
	 double m_max[3];
	
	/**
	 * 1/scaling in most cases (exceptions: scaling = 0
	 * and scaling = inf), used to convert coordinates to indices
	 */
	 double m_scalingInv[3];
	
	/** Number of blocks in x, y and z dimension */
	 unsigned long m_blocks[3];
	
	/** Number of values in x, y and z dimension in one block */
	 size_t m_blockSize[3];
	
	/** Number of cached blocks on each node */
	 long m_blocksPerNode;
	
	/**
	 * Difference between the hands of the 2-handed clock algorithm.
	 * Subclasses my require this to initialize the
	 * {@link blocks::BlockManager}.
	 */
	 long m_handSpread;
	
	/**
	 * 0, 1 or 2 if x, y or z is a time dimension (z is default if
	 * the HAS_TIME hint is specified);
	 * -2 if we don't have any time dimension;
	 * -1 if the time dimension is not (yet) specified
	 * <br>
	 * Declare as signed, to remove compiler warning
	 */
	 signed char m_timeDimension;
         

	/** Access counters for this grid (level) */
	perf::Counter m_counter;
        
	public:
        Grid(const GridContainer &container, ThreadHandler &threadHandle,
                unsigned int hint = asagi::Grid::NO_HINT);
        virtual ~Grid();
	 /** The container we belong too */
	const GridContainer &m_container;
        
        /** The ThreadHandle we belong too */
         ThreadHandler &m_threadHandle;
         
	/** Name of the variable in the netcdf file (default: "z") */
	 std::string m_variableName;
	asagi::Grid::Error setParam(const char* name, const char* value);
	
	asagi::Grid::Error open(const char* filename);
	
	/**
	 * @return The minimal possible coordinate in x dimension
	 */
	double getXMin() const
	{
		return m_min[0];
	}
	/**
	 * @return The minimal possible coordinate in y dimension
	 */
	double getYMin() const
	{
		return m_min[1];
	}
	/**
	 * @return The minimal possible coordinate in z dimension
	 */
	double getZMin() const
	{
		return m_min[2];
	}
	/**
	 * @return The maximal possible coordinate in x dimension
	 */
	double getXMax() const
	{
		return m_max[0];
	}
	/**
	 * @return The minimal possible coordinate in y dimension
	 */
	double getYMax() const
	{
		return m_max[1];
	}
	/**
	 * @return The minimal possible coordinate in z dimension
	 */
	double getZMax() const
	{
		return m_max[2];
	}
	
	unsigned char getByte(double x, double y = 0, double z = 0);
	int getInt(double x, double y = 0, double z = 0);
	long getLong(double x, double y = 0, double z = 0);
	float getFloat(double x, double y = 0, double z = 0);
	double getDouble(double x, double y = 0, double z = 0);
	void getBuf(void* buf, double x, double y = 0, double z = 0);
	
	bool exportPng(const char* filename);

	/**
	 * @return The value of a counter
	 */
	unsigned long getCounter(const char* name)
	{
		return m_counter.get(name);
	}

private:
	void getAt(void* buf, types::Type::converter_t converter,
		   double x, double y = 0, double z = 0);
	
	/**
	 * This function is used by {@link exportPng(const char*)},
	 * which only works on floats
	 */
	float getAtFloat(unsigned long x, unsigned long y);

protected:
#ifndef ASAGI_NOMPI
	/**
	 * @return The MPI communicator used for this grid
	 */
	MPI_Comm getMPICommunicator() const
	{
		return m_container.getMPICommunicator();
	}
#endif // ASAGI_NOMPI
	/**
	 * @return The current MPI rank
	 */
	int getMPIRank() const
	{
		return m_container.getMPIRank();
	}
	/**
	 * @return The size of the MPI communicator
	 */
	int getMPISize() const
	{
		return m_container.getMPISize();
	}
	
	/**
	 * @return The type for this grid
	 */
	types::Type& getType() const
	{
		return m_container.getType();
	}
	
	/**
	 * @return The number of cells in x dimension
	 */
	unsigned long getXDim() const
	{
		return m_dim[0];
	}
	/**
	 * @return The number of cells in y dimension
	 */
	unsigned long getYDim() const
	{
		return m_dim[1];
	}
	/**
	 * @return The number of cells in z dimension
	 */
	unsigned long getZDim() const
	{
		return m_dim[2];
	}
	
	/**
	 * @return The number of blocks we should store on this node
	 */
	unsigned long getBlocksPerNode() const
	{
		return m_blocksPerNode;
	}
	
	/**
	 * @return The difference of the 2 hands in the clock algorithm
	 *  configured by the user
	 */
	long getHandsDiff() const
	{
		return m_handSpread;
	}
	
	/**
	 * @return The number of values in each direction in a block
	 */
	const size_t* getBlockSize() const
	{
		return m_blockSize;
	}
	
	/**
	 * @return The number of values in direction i in a block
	 */
	size_t getBlockSize(unsigned int i) const
	{
		return m_blockSize[i];
	}

	/**
	 * @return The number of values in each block
	 */
	unsigned long getTotalBlockSize() const
	{
		return m_blockSize[0] * m_blockSize[1] * m_blockSize[2];
	}
	
	/**
	 * @return The number of blocks in the grid
	 */
	unsigned long getBlockCount() const
	{
		return m_blocks[0] * m_blocks[1] * m_blocks[2];
	}
	
	/**
	 * Calculates the position of <code>block</code> in the grid
	 * 
	 * @param block The global block id
	 * @param[out] pos Position (offset) of the block in each dimension
	 */
	void getBlockPos(unsigned long block,
		size_t *pos) const
	{
		pos[0] = block % m_blocks[0];
		pos[1] = (block / m_blocks[0]) % m_blocks[1];
		pos[2] = block / (m_blocks[0] * m_blocks[1]);
	}
	
	/**
	 * @return The global block id that stores the value at (x, y, z)
	 */
	unsigned long getBlockByCoords(unsigned long x, unsigned long y,
		unsigned long z) const
	{
		return (((z / m_blockSize[2]) * m_blocks[1]
			+ (y / m_blockSize[1])) * m_blocks[0])
			+ (x / m_blockSize[0]);
	}
	
	/**
	 * @param id Global block id
	 * @return The rank, that stores the block
	 */
	int getBlockRank(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id % getMPISize();
#else // ROUND_ROBIN
		return id / getLocalBlockCount();
#endif // ROUND_ROBIN
	}
	
	/**
	 * @param id Global block id
	 * @return The offset of the block on the rank
	 */
	unsigned long getBlockOffset(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id / getMPISize();
#else // ROUND_ROBIN
		return id % getLocalBlockCount();
#endif // ROUND_ROBIN
	}
	
	/**
	 * @param id Local block id
	 * @return The corresponding global id
	 */
	unsigned long getGlobalBlock(unsigned long id) const
	{
#ifdef ROUND_ROBIN
		return id * getMPISize() + getMPIRank();
#else // ROUND_ROBIN
		return id + getMPIRank() * getLocalBlockCount();
#endif // ROUND_ROBIN
	}
        /**
	 * @return The of blocks that are stored on this node. It has to be a multiple of the amount of Threads.
         * 
	 */
	unsigned long getLocalBlockCount() const
	{
             //return (( ((getBlockCount() + getMPISize() - 1) / getMPISize())+m_threadHandle.getThreadCount()-1)/m_threadHandle.getThreadCount())*m_threadHandle.getThreadCount();
            return (getBlockCount() + getMPISize() - 1) / getMPISize();
	}
        
        /**
	 * @return The amount of blocks that are stored on one thread.
	 */
	unsigned long getThreadBlockCount() const
        {
		return (getLocalBlockCount() + m_threadHandle.getThreadCount() - 1) / m_threadHandle.getThreadCount();
	}
        
        
        /**
	 * @return the number of blocks we should store on this thread node. For Cache.
	 */
	unsigned long getBlocksPerThread() const
        {
                return (getBlocksPerNode() + m_threadHandle.getThreadCount() - 1 ) / m_threadHandle.getThreadCount();
	}
	
	/**
	 * @param id Global block id
	 * @return The thread rank, that stores the block
	 */
	int getBlockThreadRank(unsigned long id) const
	{
            return (getBlockOffset(id) / getThreadBlockCount());
	}
	
	/**
	 * @param id Global block id
	 * @return The offset of the block in the memspace of the thread
	 */
	unsigned long getBlockThreadOffset(unsigned long id) const
	{
            return getBlockOffset(id) % getThreadBlockCount();
        }
        
        /**
	 * @param block Global block id
	 * @return The ThreadID which holds the block
	 */
        unsigned long getThreadId(unsigned long block) const {
            return m_threadHandle.getThreadId(getBlockThreadRank(block));
        }
        
	
	/**
	 * This function is called after opening the NetCDF file. Subclasses
	 * should override it to initialize the grid.
	 */
	virtual asagi::Grid::Error init() = 0;
	
	/**
	 * Subclasses should override this and return false, if they still need
	 * to access the input file after initialization.
	 * 
	 * @return True if the input file should be accessable after
	 * {@link init()} was called.
	 */
	virtual bool keepFileOpen() const
	{
		return false;
	}
	
	/**
	 * Writes the value at the specified index into the buffer, after
	 * converting it with the converter
	 */
	virtual void getAt(void* buf, types::Type::converter_t converter,
		unsigned long x, unsigned long y = 0, unsigned long z = 0) = 0;

	/**
	 * Used by subclasses to increment counter.
	 * perf::Counter::ACCESS is already handled by this class!
	 */
	void incCounter(perf::Counter::CounterType type)
	{
		m_counter.inc(type);
	}

private:
	/** The smallest number we can represent in a double */
	static constexpr double NUMERIC_PRECISION = 1e-10;
private:
	static void h2rgb(float h, unsigned char &red, unsigned char &green,
		unsigned char &blue);
	static double getInvScaling(double scaling);
	static double round(double value);
	
               
};
}

#endif	/* NUMAGRID_H */

