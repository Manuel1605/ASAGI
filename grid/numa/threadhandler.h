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

#ifndef GRID_THREADHANDLER_H
#define GRID_THREADHANDLER_H

#include <asagi.h>
#include "fortran/pointerarray.h"
#include "types/type.h"
#include <map>



namespace grid
{

    class Grid;
/**
 * A Handler class which is responsible for registering threads.
 * Threads can communicate only via this class.
 */
class ThreadHandler : public asagi::Grid
{
public:
	/** Possible positions of the values in a grid */
	enum ValuePos { CELL_CENTERED, VERTEX_CENTERED };
        /** Number of levels the grid container has */
	const unsigned int m_levels;
        /** Optimization hint */
        const unsigned int m_hint;
        /** Type of the grid */
        const asagi::Grid::Type m_type1;
        
        /**
         * Array of Gridhandles
         */
        asagi::Grid** m_gridHandle;
 
        std::map<pthread_t, asagi::Grid*> m_gridMap;
        
        /** Pointer for NumaLocalStaticGrid */
        std::map<pthread_t, unsigned char**> m_staticPtr;
        
        /** Id of the Masterthread */
        pthread_t m_masterthreadId;
        
        /** Array of threadHandles */
        pthread_t* m_threadHandle;
        
        /** Number of threads this handler has to manage*/
        unsigned int m_tCount;
        
        io::NetCdfReader *m_inputFile;
#ifndef ASAGI_NOMPI        
        /** Shared Window Object */
        MPI_Win mpiWindow;
        
        /** Mutex for MPI_Get */
        pthread_spinlock_t spinlock;
#endif
        /** 
        * Flag which indicates if the Source File can be closed.
        * It is set, after the last but one has called open(). The next one, can close the file.
        */
        bool m_closeFile;
        
        
private:
	/** Id of the grid, used for the fortran <-> c interface */
	int m_id;
#ifndef ASAGI_NOMPI
	/** The communicator we use */
	MPI_Comm m_communicator;
#endif // ASAGI_NOMPI
	/** Rank of this process */
	int m_mpiRank;
	/** Size of the MPI communicator */
	int m_mpiSize;
	
	/**
	 * The type of values we save in the grid.
	 * This class implements all type specific operations.
	 */
	types::Type *m_type;
	
	/** True if the container should skip MPI calls like MPI_Comm_dup */
	bool m_noMPI;
                    
        
        
        
        /**
         * Counts how many Threads are already registered
         */
        unsigned int m_registerCount;

        /**
         * Counts how many Threads have called open()
         */
        unsigned int m_openCount;
protected:
	
	
	/** Minimum in x dimension (set by subclasses) */
	double m_minX;
	/** Minimum in y dimension (set by subclasses) */
	double m_minY;
	/** Minimum in z dimension (set by subclasses) */
	double m_minZ;
	/** Maximum in x dimension (set by subclasses) */
	double m_maxX;
	/** Maximum in y dimension (set by subclasses) */
	double m_maxY;
	/** Maximum in z dimension (set by subclasses) */
	double m_maxZ;
	
	/** Value Position (cell-centered || vertex-centered) */
	ValuePos m_valuePos;
        
public:
	ThreadHandler(Type type, unsigned int hint, unsigned int levels, unsigned int tCount);
	virtual ~ThreadHandler();
        
	
#ifndef ASAGI_NOMPI
	Error setComm(MPI_Comm comm = MPI_COMM_WORLD);
#endif // ASAGI_NOMPI
	virtual Error setParam(const char* name, const char* value,
		unsigned int level = 0);
	/**
	 * The default implementation make sure a communicator is set.
	 * <br>
	 * Subclasses should override this function and call open from the 
	 * parent class before doing anythin.
	 */
	virtual Error open(const char* filename, unsigned int level = 0);
	
        virtual Error registerThread();
        
        double getXMin() const
	{
		return m_minX;
	}
        double getYMin() const
	{
		return m_minY;
	}
	double getZMin() const
	{
		return m_minZ;
	}
	double getXMax() const
        {
        
		return m_maxX;
	}
	double getYMax() const
	{
		return m_maxY;
	}
	double getZMax() const
	{
		return m_maxZ;
	}
	
	unsigned int getVarSize() const
	{
		return m_type->getSize();
	}
	
	unsigned char getByte1D(double x, unsigned int level = 0)
	{
		return getByte3D(x, 0, 0, level);
	}
	int getInt1D(double x, unsigned int level = 0)
	{
		return getInt3D(x, 0, 0, level);
	}
	long getLong1D(double x, unsigned int level = 0)
	{
		return getLong3D(x, 0, 0, level);
	}
	float getFloat1D(double x, unsigned int level = 0)
	{
		return getFloat3D(x, 0, 0, level);
	}
	double getDouble1D(double x, unsigned int level = 0)
	{
		return getDouble3D(x, 0, 0, level);
	}
	void getBuf1D(void* buf, double x, unsigned int level = 0)
	{
		getBuf3D(buf, x, 0, 0, level);
	}
	
	unsigned char getByte2D(double x, double y, unsigned int level = 0)
	{
		return getByte3D(x, y, 0, level);
	}
	int getInt2D(double x, double y, unsigned int level = 0)
	{
		return getInt3D(x, y, 0, level);
	}
	long getLong2D(double x, double y, unsigned int level = 0)
	{
		return getLong3D(x, y, 0, level);
	}
	float getFloat2D(double x, double y, unsigned int level = 0)
	{
		return getFloat3D(x, y, 0, level);
	}
	double getDouble2D(double x, double y, unsigned int level = 0)
	{
		return getDouble3D(x, y, 0, level);
	}
	void getBuf2D(void* buf, double x, double y, unsigned int level = 0)
	{
		getBuf3D(buf, x, y, 0, level);
	}
        unsigned char getByte3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	int getInt3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	long getLong3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	float getFloat3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	double getDouble3D(double x, double y = 0, double z = 0,
		unsigned int level = 0);
	void getBuf3D(void* buf, double x, double y = 0, double z = 0,
		unsigned int level = 0);
	
	bool exportPng(const char* filename, unsigned int level = 0);

	unsigned long getCounter(const char* name, unsigned int level = 0);
	
	
	/**
	 * @return The type assosiated with this grid
	 */
	types::Type& getType() const
	{
		return *m_type;
	}
	
	/**
	 * @return The position of values in the grid
	 */
	ValuePos getValuePos() const
	{
		return m_valuePos;
	}
	
	/**
	 * Converts the C pointer of the grid to the Fortran identifier
	 * 
	 * @return The unique index of the Threadhandler 
         *
	 */
	int c2f() const
	{
		return m_id;
	}

protected:
	grid::Grid* createGrid(unsigned int hint, unsigned int id) const;

private:
	/** The index -> pointer translation array */
	static fortran::PointerArray<ThreadHandler> m_pointers;

public:
        void setStaticPtr(pthread_t threadId, unsigned char *ptr, unsigned int id){
            m_staticPtr[threadId][id]=ptr;
           // printf("Set Pointer: %p", m_staticPtr[threadId][id] );
        }
        unsigned char* getStaticPtr(pthread_t threadId, unsigned int id){
            //printf("Get Pointer: %p", m_staticPtr[threadId][id] );
            return m_staticPtr[threadId][id];
        }
        pthread_t getMasterthreadId() const{
            return m_masterthreadId;
        }
        pthread_t getThreadId(unsigned int threadRank) const{
            return m_threadHandle[threadRank];
        }
        unsigned int getThreadRank(pthread_t threadId) const{
            for(unsigned int i=0; i<m_tCount; i++){
                if(m_threadHandle[i]==threadId){
                    return i;
                }
            }
            return 0;
        }
        unsigned int getThreadCount() const{
            return m_tCount;
        }
        bool isCloseFile() const{
            return m_closeFile;
        }
        
        /**
	 * @return The input file used for this grid
	 */
	io::NetCdfReader& getInputFile() const
	{
		return *m_inputFile;
	}
        
       /* 
        void* getNumaDistStaticGridHandle(){
            return m_numaDistStaticGridHandle;
        }
        void setNumaDistStaticGridHandle(void *grid){
            m_numaDistStaticGridHandle = grid;
        }*/
	/**
	 * Converts a Fortran index to a c/c++ pointer
	 */
	static ThreadHandler* f2c(int i)
	{
		return m_pointers.get(i);
	}
};

}

#endif // THREAD_GRID_CONTAINER_THREADGRIDCONTAINER_H

