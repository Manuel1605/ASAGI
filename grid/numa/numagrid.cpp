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

#include "numagrid.h"

#include "grid/constants.h"

#include "types/basictype.h"

#include <cstdlib>
#include <cmath>
#include <limits>


/**
 * @param container The container, this numagrid belongs to
 * @param hint Optimization hints
 */    

        io::NetCdfReader *grid::g_inputFile;
	
	unsigned long grid::g_dim[3];
	
	double grid::g_offset[3];
	
	double grid::g_min[3];
	double grid::g_max[3];

	double grid::g_scalingInv[3];
	
	unsigned long grid::g_blocks[3];
	
	size_t grid::g_blockSize[3];
	
	long grid::g_blocksPerNode;

	long  grid::g_handSpread;
	signed char grid::g_timeDimension;
        perf::Counter grid::g_counter;
grid::Grid::Grid(const GridContainer &container,
	unsigned int hint)
	: m_container(container), m_variableName("z")
{
    if(pthread_equal(ThreadHandler::masterthreadId, pthread_self())){
        g_inputFile = 0L;
	
	g_blockSize[0] = g_blockSize[1] = g_blockSize[2] = 0;
	
	g_blocksPerNode = -1;
	
	g_handSpread = -1;

	if (hint & asagi::Grid::HAS_TIME)
		g_timeDimension = -1;
	else
		g_timeDimension = -2;
    }
}

grid::Grid::~Grid()
{
    //delete g_inputFile;
}
/**
 * Accepts the following parameters:
 * @li @b variable-name
 * @li @b time-dimension
 * @li @b x-block-size
 * @li @b y-block-size
 * @li @b z-block-size
 * @li @b block-cache-size
 * @li @b cache-hand-spread
 * 
 * @see asagi::Grid::setParam(const char*, const char*, unsigned int)
 */
asagi::Grid::Error grid::Grid::setParam(const char* name, const char* value)
{
	long blockSize;
	
	if (strcmp(name, "variable-name") == 0) {
		m_variableName = value;
		return asagi::Grid::SUCCESS;
	}
	
	if (strcmp(name, "time-dimension") == 0) {
		if (g_timeDimension <= -2)
			// HAS_TIME hint was not specified
			//-> ignore this variable
			return asagi::Grid::SUCCESS;
		
		// Value should be x, y or z
		for (signed char i = 0; i < 3; i++) {
			if (strcmp(value, DIMENSION_NAMES[i]) == 0) {
				g_timeDimension = i;
				return asagi::Grid::SUCCESS;
			}
		}
		
		return asagi::Grid::INVALID_VALUE;
	}
	
	if (strcmp(&name[1], "-block-size") == 0) {
		// Check for [xyz]-block-size
		
		for (signed char i = 0; i < 3; i++) {
			if (name[0] == DIMENSION_NAMES[i][0]) {
				blockSize = atol(value);
				
				if (blockSize <= 0)
					return asagi::Grid::INVALID_VALUE;
				
				g_blockSize[i] = blockSize;
				return asagi::Grid::SUCCESS;
			}
		}
	}
	
	if (strcmp(name, "block-cache-size") == 0) {
		g_blocksPerNode = atol(value);
		
		if (g_blocksPerNode < 0)
			// We set a correct value later
			return asagi::Grid::INVALID_VALUE;
		
		if ((g_blocksPerNode == 0) && (getMPISize() > 1))
			logWarning() << "Empty block cache size may lead to failures!";
		
		return asagi::Grid::SUCCESS;
	}
	
	if ((strcmp(name, "cache-hand-spread") == 0)
		|| (strcmp(name, "cache-hand-difference") == 0)) { // Obsolete name
		g_handSpread = atol(value);
		
		return asagi::Grid::SUCCESS;
	}
	
	return asagi::Grid::UNKNOWN_PARAM;
}

/**
 * Reads a grid form the file and initializes all variables
 */
asagi::Grid::Error grid::Grid::open(const char* filename)
{
    asagi::Grid::Error error;
    if(pthread_equal(ThreadHandler::masterthreadId, pthread_self())){
	double scaling[3];
	
	// Open NetCDF file
	g_inputFile = new io::NetCdfReader(filename, getMPIRank());
	if ((error = g_inputFile->open(m_variableName.c_str()))
		!= asagi::Grid::SUCCESS)
            return error;

        
	
#ifdef __INTEL_COMPILER
	#pragma unroll(3)
#endif // __INTEL_COMPILER
	for (unsigned char i = 0; i < 3; i++) {
		// Get dimension size
		g_dim[i] = g_inputFile->getSize(i);
	
		// Get offset and scaling
		g_offset[i] = g_inputFile->getOffset(i);
	
		scaling[i] = g_inputFile->getScaling(i);
	}

	// Set time dimension
	if (g_timeDimension == -1) {
		// Time grid, but time dimension not specified
		g_timeDimension = g_inputFile->getDimensions() - 1;
		logDebug(getMPIRank()) << "Assuming time dimension"
			<< DIMENSION_NAMES[g_timeDimension];
	}
	
	// Set block size in time dimension
	if ((g_timeDimension >= 0) && (g_blockSize[g_timeDimension] == 0)) {
		logDebug(getMPIRank()) << "Setting block size in time dimension"
			<< DIMENSION_NAMES[g_timeDimension] << "to 1";
		g_blockSize[g_timeDimension] = 1;
	}
	
	// Set default block size and calculate number of blocks
#ifdef __INTEL_COMPILER
	#pragma unroll(3)
#endif // __INTEL_COMPILER
	for (unsigned char i = 0; i < 3; i++) {

		if (g_blockSize[i] == 0)
			// Setting default block size, if not yet set
			g_blockSize[i] = 50;

		// A block size large than the dimension does not make any sense

		if (g_dim[i] < g_blockSize[i]) {
			logDebug(getMPIRank()) << "Shrinking" << DIMENSION_NAMES[i]
				<< "block size to" << g_dim[i];
			g_blockSize[i] = g_dim[i];
		}
               // std::cout << "Dim: " << g_dim[i] << " Blocksize: " << g_blockSize[i] << std::endl;

		// Integer way of rounding up
		g_blocks[i] = (g_dim[i] + g_blockSize[i] - 1) / g_blockSize[i];
		g_scalingInv[i] = getInvScaling(scaling[i]);

		// Set min/max
		if (std::isinf(scaling[i])) {
			g_min[i] = -std::numeric_limits<double>::infinity();
			g_max[i] = std::numeric_limits<double>::infinity();
		} else {
			// Warning: min and max are inverted of scaling is negative
			double min = g_offset[i];
			double max = g_offset[i] + scaling[i] * (g_dim[i] - 1);

			if (m_container.getValuePos()
				== GridContainer::CELL_CENTERED) {
				// Add half a cell on both ends
				min -= scaling[i] * (0.5 - NUMERIC_PRECISION);
				max += scaling[i] * (0.5 - NUMERIC_PRECISION);
			}

			g_min[i] = std::min(min, max);
			g_max[i] = std::max(min, max);
		}

	}
	
	// Set default cache size
	if (g_blocksPerNode < 0)
		// Default value
		g_blocksPerNode = 80;
	
	// Init type
	if ((error = getType().check(*g_inputFile)) != asagi::Grid::SUCCESS)
		return error;
        
        // Init subclass
    }
	error = init();
        if (!keepFileOpen()) {
		// input file no longer needed, so we close
		delete g_inputFile;
		g_inputFile = 0L;
	}
	return error;
}

/**
 * @return The value at (x,y,z) as a byte
 */
unsigned char grid::Grid::getByte(double x, double y, double z)
{
	unsigned char buf;
	getAt(&buf, &types::Type::convertByte, x, y, z);
	
	return buf;
}

/**
 * @return The value at (x,y,z) as an integer
 */
int grid::Grid::getInt(double x, double y, double z)
{
	int buf;
	getAt(&buf, &types::Type::convertInt, x, y, z);

	return buf;
}

/**
 * @return The value at (x,y,z) as a long
 */
long grid::Grid::getLong(double x, double y, double z)
{
	long buf;
	getAt(&buf, &types::Type::convertLong, x, y, z);

	return buf;
}

/**
 * @return The value at (x,y,z) as a float
 */
float grid::Grid::getFloat(double x, double y, double z)
{
	float buf;
	getAt(&buf, &types::Type::convertFloat, x, y, z);

	return buf;
}

/**
 * @return The value at (x,y,z) as a double
 */
double grid::Grid::getDouble(double x, double y, double z)
{
	double buf;
	getAt(&buf, &types::Type::convertDouble, x, y, z);

	return buf;
}

/**
 * Copys the value at (x,y,z) into the buffer
 */
void grid::Grid::getBuf(void* buf, double x, double y, double z)
{
	getAt(buf, &types::Type::convertBuffer, x, y, z);
}

/**
 * @see asagi::Grid::exportPng()
 */
bool grid::Grid::exportPng(const char* filename)
{
    return false;
}

/**
 * Converts the coordinates to indices and writes the value at the position
 * into the buffer
 */
void grid::Grid::getAt(void* buf, types::Type::converter_t converter,
	double x, double y, double z)
{
	x = round((x - g_offset[0]) * g_scalingInv[0]);
	y = round((y - g_offset[1]) * g_scalingInv[1]);
	z = round((z - g_offset[2]) * g_scalingInv[2]);

	assert(x >= 0 && x < getXDim()
		&& y >= 0 && y < getYDim()
		&& z >= 0 && z < getZDim());

	g_counter.inc(perf::Counter::ACCESS);

	getAt(buf, converter, static_cast<unsigned long>(x),
		static_cast<unsigned long>(y), static_cast<unsigned long>(z));
}

float grid::Grid::getAtFloat(unsigned long x, unsigned long y)
{
	float buf;
	
	getAt(&buf, &types::Type::convertFloat, x, y);
	
	return buf;
}

/**
 * Calculates a nice rgb representation for a value between 0 and 1
 */
void grid::Grid::h2rgb(float h, unsigned char &red, unsigned char &green,
	unsigned char &blue)
{
	// h from 0..1
	
	h *= 6;
	float x = fmod(h, 2) - 1;
	if (x < 0)
		x *= -1;
	x = 1 - x;
	
	// <= checks are not 100% correct, it should be <
	// but it solves the "largest-value" issue
	if (h <= 1) {
		red = 255;
		green = x * 255;
		blue = 0;
		return;
	}
	if (h <= 2) {
		red = x * 255;
		green = 255;
		blue = 0;
		return;
	}
	if (h <= 3) {
		red = 0;
		green = 255;
		blue = x * 255;
		return;
	}
	if (h <= 4) {
		red = 0;
		green = x * 255;
		blue = 255;
		return;
	}
	if (h <= 5) {
		red = x * 255;
		green = 0;
		blue = 255;
	}
	// h < 6
	red = 255;
	green = 0;
	blue = x * 255;
}

/**
 * Calculates 1/scaling, except for scaling = 0 and scaling = inf. In this
 * case it returns 0
 */
double grid::Grid::getInvScaling(double scaling)
{
	if ((scaling == 0) || isinf(scaling))
		return 0;
	
	return 1/scaling;
}

/**
 * Implementation for round-to-nearest
 */
double grid::Grid::round(double value)
{
	return floor(value + 0.5);
}
