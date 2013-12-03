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

#include "threadhandler.h"
#include "numagridcontainer.h"
#include "numalocalcachegrid.h"
#include "numalocalstaticgrid.h"
#include <pthread.h>
#include <iostream>

pthread_mutex_t grid::ThreadHandler::mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t grid::ThreadHandler::cond  = PTHREAD_COND_INITIALIZER;
std::map<pthread_t, unsigned char**> grid::ThreadHandler::staticPtr;
pthread_t* grid::ThreadHandler::threadHandle;
unsigned int grid::ThreadHandler::tCount;
pthread_t grid::ThreadHandler::masterthreadId;


/**
 * @param type The basic type of the values
 * @param hint Any performance hints
 * @param levels The number of levels
 * @param tCount The amount of threads
 */
grid::ThreadHandler::ThreadHandler(Type type, unsigned int hint, unsigned int levels, unsigned int tCount) : m_levels(levels), m_hint(hint), m_type1(type) {
    // Prepare for fortran <-> c translation
    m_id = m_pointers.add(this);
    m_count = 0;
    ThreadHandler::tCount= tCount;
    gridHandle = new asagi::Grid*[tCount];
    threadHandle = (pthread_t*) malloc(sizeof(pthread_t)*tCount);
}

/** 
 * Destructor
 */
grid::ThreadHandler::~ThreadHandler() {
    pthread_cond_destroy(&cond);
    pthread_mutex_destroy(&mutex);
    for(unsigned int i=0; i< tCount; i++){
        delete gridHandle[i];
    }
    delete[] gridHandle;
    free(threadHandle);
    
    // Remove from fortran <-> c translation
    m_pointers.remove(m_id);
}

unsigned char grid::ThreadHandler::getByte3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getByte3D(x,y,z,level);
}

int grid::ThreadHandler::getInt3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getInt3D(x,y,z,level);
}

long grid::ThreadHandler::getLong3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getLong3D(x,y,z,level);
}

float grid::ThreadHandler::getFloat3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getFloat3D(x,y,z,level);
}

double grid::ThreadHandler::getDouble3D(double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getDouble3D(x,y,z,level);

}

void grid::ThreadHandler::getBuf3D(void* buf, double x, double y, double z,
        unsigned int level) {
    return gridMap[pthread_self()]->getBuf3D(buf,x,y,z,level);
}

bool grid::ThreadHandler::exportPng(const char* filename, unsigned int level) {
    return gridMap[pthread_self()]->exportPng(filename, level);
}

unsigned long grid::ThreadHandler::getCounter(const char* name, unsigned int level) {
    return gridMap[pthread_self()]->getCounter(name, level);
}
#ifndef ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setComm(MPI_Comm comm) {
     return gridMap[pthread_self()]->setComm(comm);  
}
#endif // ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setParam(const char* name,
        const char* value,
        unsigned int level) {
    return gridMap[pthread_self()]->setParam(name, value, level);
}


asagi::Grid::Error grid::ThreadHandler::open(const char* filename,
        unsigned int level) {

    pthread_mutex_lock(&mutex);

    if(m_count==0){
        masterthreadId = pthread_self();
        NumaLocalStaticGrid::thread=0;
    }

    threadHandle[m_count] = pthread_self();
    gridHandle[m_count] = new grid::NumaGridContainer(m_type1, false, m_hint, m_levels);
    gridMap[pthread_self()] = gridHandle[m_count];
    staticPtr[pthread_self()] = (unsigned char**) malloc(sizeof(unsigned char*)*m_levels);

        gridMap[pthread_self()]->open(filename,level);

        m_minX = gridMap[pthread_self()]->getXMin();
        m_minY = gridMap[pthread_self()]->getYMin();
        m_minZ = gridMap[pthread_self()]->getZMin();

        m_maxX = gridMap[pthread_self()]->getXMax();
        m_maxY = gridMap[pthread_self()]->getYMax();
        m_maxZ = gridMap[pthread_self()]->getZMax();

        m_count++;     
       
        //Send signal and set condition. The Masterthread has allocated the Memory
        if(m_count==tCount)
                pthread_cond_broadcast(&cond);
        
        //Wait until every thread has called open()
        while(m_count<tCount){
            pthread_cond_wait(&cond, &mutex);
        }
    pthread_mutex_unlock(&mutex);
    return SUCCESS;  
}

// Fortran <-> c translation array
fortran::PointerArray<grid::ThreadHandler>
	grid::ThreadHandler::m_pointers;
