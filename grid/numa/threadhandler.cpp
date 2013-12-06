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
#include "numasimplegridcontainer.h"
#include "numalocalcachegrid.h"
#include "numalocalstaticgrid.h"
#include <pthread.h>
#include <iostream>


/* Inatialize the global variables. These are shared by all threads. */
pthread_mutex_t mutexOpen       =    PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutexRegister   =    PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t condRegister     =    PTHREAD_COND_INITIALIZER;
pthread_cond_t condOpen         =    PTHREAD_COND_INITIALIZER;


/**
 * @param type The basic type of the values
 * @param hint Any performance hints
 * @param levels The number of levels
 * @param tCount The amount of threads
 */
grid::ThreadHandler::ThreadHandler(Type type, unsigned int hint, unsigned int levels, unsigned int tCount) : m_levels(levels), m_hint(hint), m_type1(type), m_tCount(tCount) {
    // Prepare for fortran <-> c translation
    m_id = m_pointers.add(this);
    m_registerCount = 0;
    m_openCount = 0;
    m_gridHandle = new asagi::Grid*[m_tCount];
    m_threadHandle = (pthread_t*) malloc(sizeof(pthread_t)*m_tCount);
    m_masterthreadId=0L;
#ifndef NOMPI
    mpiWindow = MPI_WIN_NULL;
    pthread_spin_init(&mpiMutex, 0);
#endif
}

/** 
 * Destructor
 */
grid::ThreadHandler::~ThreadHandler() {
#ifndef ASAGI_NOMPI
        if(mpiWindow!=MPI_WIN_NULL){
            MPI_Win_free(&mpiWindow);
        }
#endif

        if(m_gridHandle!=NULL)
            for(unsigned int i=0; i< m_tCount; i++){
                delete m_gridHandle[i];
            }
        delete[] m_gridHandle;
        m_gridHandle=NULL;
        free(m_threadHandle);
        m_threadHandle=NULL;

        // Remove from fortran <-> c translation
        if(m_id!=-1){
                m_pointers.remove(m_id);
                m_id=-1;
        }

}

asagi::Grid::Error grid::ThreadHandler::registerThread(){
        pthread_mutex_lock(&mutexRegister);
        if(m_registerCount == 0)
                m_masterthreadId = pthread_self();
        m_threadHandle[m_registerCount] = pthread_self();
        m_gridHandle[m_registerCount] = new grid::SimpleGridContainer(*this, m_type1, false, m_hint, m_levels);
        m_staticPtr[pthread_self()] = (unsigned char**) malloc(sizeof(unsigned char*)*m_levels);
        m_gridMap[pthread_self()] = m_gridHandle[m_registerCount];
        //m_gridMap[pthread_self()]->setThreadHandle(*this);
        m_registerCount++;     
        
        if(m_registerCount==m_tCount){
            pthread_cond_broadcast(&condRegister);      //Each thread has called registerThread().
        }
        
        //Wait until every thread has called registerThread()
        while(m_registerCount<m_tCount){
            pthread_cond_wait(&condRegister, &mutexRegister);
        }

        pthread_mutex_unlock(&mutexRegister);
        return SUCCESS;
}

unsigned char grid::ThreadHandler::getByte3D(double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getByte3D(x,y,z,level);
}

int grid::ThreadHandler::getInt3D(double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getInt3D(x,y,z,level);
}

long grid::ThreadHandler::getLong3D(double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getLong3D(x,y,z,level);
}

float grid::ThreadHandler::getFloat3D(double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getFloat3D(x,y,z,level);
}

double grid::ThreadHandler::getDouble3D(double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getDouble3D(x,y,z,level);

}

void grid::ThreadHandler::getBuf3D(void* buf, double x, double y, double z,
        unsigned int level) {
    return m_gridMap[pthread_self()]->getBuf3D(buf,x,y,z,level);
}

bool grid::ThreadHandler::exportPng(const char* filename, unsigned int level) {
    return m_gridMap[pthread_self()]->exportPng(filename, level);
}

unsigned long grid::ThreadHandler::getCounter(const char* name, unsigned int level) {
    return m_gridMap[pthread_self()]->getCounter(name, level);
}
#ifndef ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setComm(MPI_Comm comm) {
     return m_gridMap[pthread_self()]->setComm(comm);  
}
#endif // ASAGI_NOMPI

asagi::Grid::Error grid::ThreadHandler::setParam(const char* name,
        const char* value,
        unsigned int level) {
    return m_gridMap[pthread_self()]->setParam(name, value, level);
}


asagi::Grid::Error grid::ThreadHandler::open(const char* filename,
        unsigned int level) {
    //check if I'm the next one.
    while(m_openCount < m_tCount){
        pthread_mutex_lock(&mutexOpen);
        if(pthread_equal(pthread_self(), m_threadHandle[m_openCount])){
            if(m_openCount == m_tCount - 1){
                m_closeFile=true; //I´m the last one. The file can be be closed, after I've loaded the values.
            }
            m_gridMap[pthread_self()]->open(filename,level);
            if(pthread_equal(m_masterthreadId, pthread_self())){
                m_minX=m_gridMap[pthread_self()]->getXMin();
                m_minY=m_gridMap[pthread_self()]->getYMin();
                m_minZ=m_gridMap[pthread_self()]->getZMin();
                m_maxX=m_gridMap[pthread_self()]->getXMax();
                m_maxY=m_gridMap[pthread_self()]->getYMax();
                m_maxZ=m_gridMap[pthread_self()]->getZMax();
            }
            m_openCount++;     

            //Send signal and set condition. The Masterthread has allocated the Memory
            if(m_openCount==m_tCount){
                    pthread_cond_broadcast(&condOpen);
            }
            
            //Wait until every thread has called open()
            while(m_openCount<m_tCount){
                pthread_cond_wait(&condOpen, &mutexOpen);
            }
        }
        pthread_mutex_unlock(&mutexOpen);

    }
    return SUCCESS;  
}



// Fortran <-> c translation array
fortran::PointerArray<grid::ThreadHandler>
	grid::ThreadHandler::m_pointers;
