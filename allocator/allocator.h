/**
 * @file
 *  This file is part of ASAGI
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
 * @copyright 2013 Sebastian Rettenberger <rettenbs@in.tum.de>
 */

#ifndef ALLOCATOR_ALLOCATOR_H
#define ALLOCATOR_ALLOCATOR_H

#include <asagi.h>

/**
 * Provides classes that can be used to allocate and free
 * memory.
 */
namespace allocator
{

/**
 * Pure virtual base class that defines all functions provided
 * by allocators
 *
 * @tparam T The type that is allocated
 *
 * @warning Subclasses can not have a destructor, because this class
 *  does not have a virtual one. A virtual constructor here leads to
 *  crashes on some systems.
 */
template<typename T>
class Allocator
{
public:
	/**
	 * Allocates sizeof(T)*size bytes and saves the pointer in ptr.
	 *
	 * @return asagi::Grid::SUCCESS if the memory was allocated
	 */
	virtual asagi::Grid::Error allocate(size_t size, T* &ptr) const = 0;
	
        /**
	 * Frees the memory allocated with allocate()
	 */
	virtual void free(T* ptr) const = 0;
};

}

#endif /* ALLOCATOR_ALLOCATOR_H */
