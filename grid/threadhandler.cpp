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

#include "threadhandler.h"
#include "adaptivegridcontainer.h"
#include "simplegridcontainer.h"
#include <pthread.h>

#ifndef ASAGI_NOMPI
#endif // ASAGI_NOMPI
pthread_t grid::ThreadHandler::m_masterthreadId;
pthread_mutex_t grid::ThreadHandler::mutex = PTHREAD_MUTEX_INITIALIZER;
unsigned int count;

/**
 * @param type The basic type of the values
 * @param isArray True if the type is an array, false if it is a basic
 *  type
 * @param hint Any performance hints
 * @param levels The number of levels
 * @param tCount The amount of threads
 */
grid::ThreadHandler::ThreadHandler(Type type, unsigned int hint, unsigned int levels, unsigned int tCount) : m_levels(levels), m_hint(hint), m_tCount(tCount) {
    pthread_mutex_lock(&mutex);
    count = 0;
    if (hint & ADAPTIVE)
        masterHandle = new grid::AdaptiveGridContainer(type, hint, levels);
    else
        masterHandle = new grid::SimpleGridContainer(type, hint, levels);
    m_masterthreadId = pthread_self();
    pthread_mutex_unlock(&mutex);
}

/** 
 * Creates a GridContainer for a Thread.
 */
bool grid::ThreadHandler::registerThread() {
    pthread_mutex_lock(&mutex);
    if (m_hint & ADAPTIVE)
        threadHandle[count] = new grid::AdaptiveGridContainer(this);
    else
        threadHandle[count] = new grid::SimpleGridContainer(this);
    count++;
    pthread_mutex_unlock(&mutex);
    return true;
}

/** 
 * Creates a GridContainer for a Thread.
 */
grid::ThreadHandler::~ThreadHandler() {
}

unsigned char grid::ThreadHandler::getByte3D(double x, double y, double z,
        unsigned int level) {
    return '0';

}

int grid::ThreadHandler::getInt3D(double x, double y, double z,
        unsigned int level) {
    return 0;
}

long grid::ThreadHandler::getLong3D(double x, double y, double z,
        unsigned int level) {
    return 0;
}

float grid::ThreadHandler::getFloat3D(double x, double, double,
        unsigned int level) {
    return 0.0;
}

double grid::ThreadHandler::getDouble3D(double x, double y, double z,
        unsigned int level) {
    return 0.0;
}

void grid::ThreadHandler::getBuf3D(void* buf, double x, double y, double z,
        unsigned int level) {

}

bool grid::ThreadHandler::exportPng(const char* filename, unsigned int level) {
    return false;
}

unsigned long grid::ThreadHandler::getCounter(const char* name, unsigned int level) {
    return 0;
}
#ifndef ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setComm(MPI_Comm comm) {
   
    if (m_communicator != MPI_COMM_NULL)
        // set communicator only once
        return SUCCESS;

    if (MPI_Comm_dup(comm, &m_communicator) != MPI_SUCCESS)
        return MPI_ERROR;

    MPI_Comm_rank(m_communicator, &m_mpiRank);
    MPI_Comm_size(m_communicator, &m_mpiSize);

    return SUCCESS;
     
}
#endif // ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setParam(const char* name,
        const char* value,
        unsigned int level) {
    if (strcmp(name, "value-position") == 0) {
        if (strcmp(value, "cell-centered") == 0) {
            m_valuePos = CELL_CENTERED;
            return SUCCESS;
        }

        if (strcmp(value, "vertex-centered") == 0) {
            m_valuePos = VERTEX_CENTERED;
            return SUCCESS;
        }

        return INVALID_VALUE;
    }

    return UNKNOWN_PARAM;
}

asagi::Grid::Error grid::ThreadHandler::open(const char* filename,
        unsigned int level) {
    assert(level < m_levels);

#ifdef ASAGI_NOMPI
    return SUCCESS;
#else // ASAGI_NOMPI
    // Make sure we have our own communicator
    if (m_noMPI)
        return SUCCESS;
    return setComm();
#endif // ASAGI_NOMPI
  
}



