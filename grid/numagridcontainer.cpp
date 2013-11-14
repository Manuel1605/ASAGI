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
 *op
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

#include "numagridcontainer.h"
#include "numalocalstaticgrid.h"
#include "numalocalcachegrid.h"
#include "numadiststaticgrid.h"

#include <algorithm>

/**
 * @see GridContainer::GridContainer()
 */
grid::NumaGridContainer::NumaGridContainer(Type type, bool isArray, unsigned int hint, unsigned int levels) : GridContainer(type, isArray, hint, levels) {
    m_grids = new grid::Grid*[m_levels];
    for (unsigned int i = 0; i < levels; i++)
        m_grids[i] = createGrid(hint, i);
}

grid::NumaGridContainer::~NumaGridContainer() {
    for (unsigned int i = 0; i < m_levels; i++)
        delete m_grids[i];
    delete [] m_grids;
}

unsigned char grid::NumaGridContainer::getByte3D(double x, double y,
        double z, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getByte(x, y, z);
}

int grid::NumaGridContainer::getInt3D(double x, double y,
        double z, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getInt(x, y, z);
}

long grid::NumaGridContainer::getLong3D(double x, double y,
        double z, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getLong(x, y, z);
}

float grid::NumaGridContainer::getFloat3D(double x, double y,
        double z, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getFloat(x, y, z);
}

double grid::NumaGridContainer::getDouble3D(double x, double y, double z,
        unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getDouble(x, y, z);
}

void grid::NumaGridContainer::getBuf3D(void* buf, double x, double y, double z,
        unsigned int level) {
    assert(level < m_levels);

    m_grids[level]->getBuf(buf, x, y, z);
}

bool grid::NumaGridContainer::exportPng(const char* filename, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->exportPng(filename);
}

unsigned long grid::NumaGridContainer::getCounter(const char* name, unsigned int level) {
    assert(level < m_levels);

    return m_grids[level]->getCounter(name);
}

asagi::Grid::Error grid::NumaGridContainer::setParam(const char* name, const char* value,
        unsigned int level) {
    if(pthread_equal(pthread_self(), ThreadHandler::masterthreadId)){
        Error result;
	result = GridContainer::setParam(name, value, level);
	if (result != UNKNOWN_PARAM)
		return result;
	
	assert(level < m_levels);
	return m_grids[level]->setParam(name, value);
    }
    return MPI_ERROR;
}

asagi::Grid::Error grid::NumaGridContainer::open(const char* filename, unsigned int level) {
    Error result;
    if(pthread_equal(pthread_self(), ThreadHandler::masterthreadId)){
        result = GridContainer::open(filename, level);
	if (result != SUCCESS)
		return result;
    }
    
    result = m_grids[level]->open(filename);
    if (result != SUCCESS)
        return result;
    m_minX = std::max(m_minX, m_grids[level]->getXMin());
    m_minY = std::max(m_minY, m_grids[level]->getYMin());
    m_minZ = std::max(m_minZ, m_grids[level]->getZMin());

    m_maxX = std::min(m_maxX, m_grids[level]->getXMax());
    m_maxY = std::min(m_maxY, m_grids[level]->getYMax());
    m_maxZ = std::min(m_maxZ, m_grids[level]->getZMax());

    return result;
}

asagi::Grid::Error grid::NumaGridContainer::setComm(MPI_Comm comm)
{
    if(pthread_equal(ThreadHandler::masterthreadId, pthread_self())){
	return GridContainer::setComm(comm);
    }
}


/**
 * Creates a new grid according to the hints
 */
grid::Grid* grid::NumaGridContainer::createGrid(
	unsigned int hint, unsigned int id) const
{
       /* if (hint & PASS_THROUGH)
		return new PassThroughGrid(*this, hint);*/

#ifndef ASAGI_NOMPI
	if (hint & NOMPI) {
#endif // ASAGI_NOMPI
		if (hint & SMALL_CACHE)
			return new NumaLocalCacheGrid(*this, hint, id);

		return new NumaLocalStaticGrid(*this, hint, id);
#ifndef ASAGI_NOMPI
	}
#endif // ASAGI_NOMPI

#ifndef ASAGI_NOMPI
	//if (hint & LARGE_GRID)
		//return new DistCacheGrid(*this, hint, id);

	return new NumaDistStaticGrid(*this, hint, id);
#endif // ASAGI_NOMPI
}
    // Fortran <-> c translation array
fortran::PointerArray<grid::GridContainer>
	grid::GridContainer::m_pointers;