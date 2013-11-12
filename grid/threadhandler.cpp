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
#include "numagridcontainer.h"
#include <pthread.h>
#include <iostream>

#ifndef ASAGI_NOMPI
#endif // ASAGI_NOMPI
unsigned int count;
pthread_mutex_t grid::ThreadHandler::mutex = PTHREAD_MUTEX_INITIALIZER;
unsigned char* grid::ThreadHandler::localStaticGridMemPtr;
unsigned char* grid::ThreadHandler::localCacheGridMemPtr;
pthread_t* grid::ThreadHandler::threadHandle;
unsigned int grid::ThreadHandler::tCount;
pthread_t grid::ThreadHandler::masterthreadId;



/**
 * @param type The basic type of the values
 * @param isArray True if the type is an array, false if it is a basic
 *  type
 * @param hint Any performance hints
 * @param levels The number of levels
 * @param tCount The amount of threads
 */
grid::ThreadHandler::ThreadHandler(Type type, unsigned int hint, unsigned int levels, unsigned int tCount) : m_levels(levels), m_hint(hint), m_type1(&type) {
    count = 0;
    ThreadHandler::tCount= tCount;
    gridHandle = new asagi::Grid*[tCount];
    threadHandle = (pthread_t*) malloc(sizeof(pthread_t)*tCount);
}

/** 
 * Creates a GridContainer for a Thread.
 */
bool grid::ThreadHandler::registerThread() {
    pthread_mutex_lock(&mutex);
    if(count==0)
        masterthreadId = pthread_self();
    threadHandle[count] = pthread_self();
    if (m_hint & ADAPTIVE){
         gridHandle[count] = new grid::AdaptiveGridContainer(asagi::Grid::Type::FLOAT, false, m_hint, m_levels, masterthreadId);
    }
    else{
         gridHandle[count] = new grid::NumaGridContainer(asagi::Grid::Type::FLOAT, false, m_hint, m_levels);
    }
    gridMap[pthread_self()] = gridHandle[count];
    count++;
    if(count==tCount)
        regCompleted=true;
    pthread_mutex_unlock(&mutex);
    return true;
}

/** 
 * Destructor
 */
grid::ThreadHandler::~ThreadHandler() {
}

unsigned char grid::ThreadHandler::getByte3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getByte2D(x,y);
}

int grid::ThreadHandler::getInt3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getInt2D(x,y);
}

long grid::ThreadHandler::getLong3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getLong2D(x,y);
}

float grid::ThreadHandler::getFloat3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getFloat2D(x,y);
}

double grid::ThreadHandler::getDouble3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getDouble2D(x,y);

}

void grid::ThreadHandler::getBuf3D(void* buf, double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getBuf2D(buf, x,y);
}

bool grid::ThreadHandler::exportPng(const char* filename, unsigned int level) {
    return gridMap[pthread_self()]->exportPng(filename, level);
}

unsigned long grid::ThreadHandler::getCounter(const char* name, unsigned int level) {
    return gridMap[pthread_self()]->getCounter(name, level);
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
    pthread_mutex_lock(&mutex);
        assert(level < m_levels);
        gridMap[pthread_self()]->open(filename,level);
    #ifdef ASAGI_NOMPI
        return SUCCESS;
    #else // ASAGI_NOMPI
       pthread_mutex_unlock(&mutex);
        return SUCCESS;
    //return setComm();
#endif // ASAGI_NOMPI
  
}



