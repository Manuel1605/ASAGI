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
 */

#include "grid/gridcontainer.h"
#include "grid/numa/threadhandler.h"
#include "asagi_f90.h"

// Init functions

int f90grid_create(grid_type type, int hint, int levels)
{
	return static_cast<grid::GridContainer*>(
		asagi::Grid::create(type, hint, levels))->c2f();
}

int f90grid_create_for_numa(grid_type type, int hint, int levels, int tcount)
{       
	return static_cast<grid::ThreadHandler*>(
		asagi::Grid::createForNuma(type, hint, levels, tcount))->c2f();
}

int f90grid_create_array(grid_type basic_type, int hint, int levels)
{
	return static_cast<grid::GridContainer*>(
		asagi::Grid::createArray(basic_type, hint, levels))->c2f();
}

int f90grid_create_struct(int count,
	int blockLength[],
	long displacements[],
	grid_type types[],
	int hint, int levels)
{
	// Not sure of reinterpret_cast is save here
	return static_cast<grid::GridContainer*>(
		asagi::Grid::createStruct(count,
			reinterpret_cast<unsigned int*>(blockLength),
			reinterpret_cast<unsigned long*>(displacements),
			types,
			hint, levels))->c2f();
}
grid_error f90grid_set_comm(int grid_id, int comm)
{
#ifdef ASAGI_NOMPI
	// We do not use a preprocessor in the fortran include file to disable
	// this completly. Instead this function always return an error.
	return asagi::Grid::MPI_ERROR;
#elif NUMA_SUPPORT
        // MPI_Comm_f2c expects an MPI_Fint, however iso_c_bindings
	// already converts this parameter into c integer
	return grid::ThreadHandler::f2c(grid_id)->setComm(MPI_Comm_f2c(comm));
#else
	// MPI_Comm_f2c expects an MPI_Fint, however iso_c_bindings
	// already converts this parameter into c integer
	return grid::GridContainer::f2c(grid_id)->setComm(MPI_Comm_f2c(comm));
#endif // ASAGI_NOMPI
}

grid_error f90grid_set_param(int grid_id, const char* name,
	const char* value, int level)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->setParam(name, value, level);
#else
	return grid::GridContainer::f2c(grid_id)->setParam(name, value, level);
#endif
}

grid_error f90grid_open(int grid_id, const char* filename,
	int level)
{
#ifdef NUMA_SUPPORT
            return grid::ThreadHandler::f2c(grid_id)->open(filename, level);
#else
            return grid::GridContainer::f2c(grid_id)->open(filename, level);
#endif

}
grid_error f90grid_register_thread(int grid_id)
{
            return grid::ThreadHandler::f2c(grid_id)->registerThread();
}

// Min/Max functions

double f90grid_min_x(int grid_id)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getXMin();
#else
        return grid::GridContainer::f2c(grid_id)->getXMin();
#endif
}
double f90grid_min_y(int grid_id)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getYMin();
#else
        return grid::GridContainer::f2c(grid_id)->getYMin();
#endif
}
double f90grid_min_z (int grid_id)
{
	#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getZMin();
#else
        return grid::GridContainer::f2c(grid_id)->getZMin();
#endif
}
double f90grid_max_x(int grid_id)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getXMax();
#else
        return grid::GridContainer::f2c(grid_id)->getXMax();
#endif
}
double f90grid_max_y(int grid_id)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getYMax();
#else
        return grid::GridContainer::f2c(grid_id)->getYMax();
#endif
}
double f90grid_max_z (int grid_id)
{
#ifdef NUMA_SUPPORT
        return grid::ThreadHandler::f2c(grid_id)->getZMax();
#else
        return grid::GridContainer::f2c(grid_id)->getZMax();
#endif
}

double f90grid_delta_x(int grid_id)
{
	return grid::GridContainer::f2c(grid_id)->getXDelta();
}
double f90grid_delta_y(int grid_id)
{
	return grid::GridContainer::f2c(grid_id)->getYDelta();
}
double f90grid_delta_z(int grid_id)
{
	return grid::GridContainer::f2c(grid_id)->getZDelta();
}

int f90grid_var_size (int grid_id)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getVarSize();
#else
	return grid::GridContainer::f2c(grid_id)->getVarSize();
#endif
}

// 1d functions

unsigned char f90grid_get_byte_1d(int grid_id, double x, int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getByte1D(x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getByte1D(x, level);
#endif
}
int f90grid_get_int_1d(int grid_id, double x, int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getInt1D(x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getInt1D(x, level);
#endif
}
long f90grid_get_long_1d(int grid_id, double x, int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getLong1D(x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getLong1D(x, level);
#endif
}
float f90grid_get_float_1d(int grid_id, double x, int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getFloat1D(x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getFloat1D(x, level);
#endif
}
double f90grid_get_double_1d(int grid_id, double x, int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getDouble1D(x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getDouble1D(x, level);
#endif
}
void f90grid_get_buf_1d(int grid_id, void* buf, double x,
	int level)
{
#ifdef NUMA_SUPPORT
    	return grid::ThreadHandler::f2c(grid_id)->getBuf1D(buf, x, level);
#else
	return grid::GridContainer::f2c(grid_id)->getBuf1D(buf, x, level);
#endif
}

// 2d functions

unsigned char f90grid_get_byte_2d(int grid_id, double x, double y,
	int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getByte2D(x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getByte2D(x, y, level);
#endif
}
int f90grid_get_int_2d(int grid_id, double x, double y, int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getInt2D(x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getInt2D(x, y, level);
#endif
}
long f90grid_get_long_2d(int grid_id, double x, double y,
	int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getLong2D(x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getLong2D(x, y, level);
#endif
}
float f90grid_get_float_2d(int grid_id, double x, double y,
	int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getFloat2D(x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getFloat2D(x, y, level);
#endif
}
double f90grid_get_double_2d(int grid_id, double x, double y,
	int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getDouble2D(x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getDouble2D(x, y, level);
#endif
}
void f90grid_get_buf_2d(int grid_id, void* buf, double x, double y,
	int level)
{
#ifdef NUMA_SUPPORT
	return grid::ThreadHandler::f2c(grid_id)->getBuf2D(buf, x, y, level);
#else
        return grid::GridContainer::f2c(grid_id)->getBuf2D(buf, x, y, level);
#endif
}

// 3d functions

unsigned char f90grid_get_byte_3d(int grid_id, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getByte3D(x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getByte3D(x, y, z, level);
#endif
}
int f90grid_get_int_3d(int grid_id, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getInt3D(x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getInt3D(x, y, z, level);
#endif
}
long f90grid_get_long_3d(int grid_id, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getLong3D(x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getLong3D(x, y, z, level);
#endif
}
float f90grid_get_float_3d(int grid_id, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getFloat3D(x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getFloat3D(x, y, z, level);
#endif
}
double f90grid_get_double_3d(int grid_id, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getDouble3D(x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getDouble3D(x, y, z, level);
#endif
}
void f90grid_get_buf_3d(int grid_id, void* buf, double x, double y, double z,
	int level)
{
#ifdef NUMA_SUPPORT 
	return grid::ThreadHandler::f2c(grid_id)->getBuf3D(buf, x, y, z, level);
#else
        return grid::GridContainer::f2c(grid_id)->getBuf3D(buf, x, y, z, level);
#endif
}

// destructor

void f90grid_close(int grid_id)
{
#ifdef NUMA_SUPPORT
        delete grid::ThreadHandler::f2c(grid_id);
#else
	delete grid::GridContainer::f2c(grid_id);
#endif
}
