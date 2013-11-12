/* 
 * File:   numaallocator.h
 * Author: root
 *
 * Created on 7. November 2013, 15:28
 */

#ifndef NUMAALLOCATOR_H
#define	NUMAALLOCATOR_H
#include <asagi.h>

#include "allocator/allocator.h"

namespace allocator
{

/**
 * This allocator uses default C++ malloc/delete mechanism
 */
template<typename T>
class NumaAllocator : public Allocator<T>
{
public:
	/**
	 * Empty constructor, required by newer gcc versions
	 */
	NumaAllocator()
	{
	}

	asagi::Grid::Error allocate(size_t size, T* &ptr) const
	{
		ptr = (T*) malloc(size);
		return asagi::Grid::SUCCESS;
	}

	void free(T *ptr) const
	{
		free(ptr);
	}

public:
	static const NumaAllocator<T> allocator;
};

/**
 * Provides a default instance for each type
 */
template<typename T>
const NumaAllocator<T> NumaAllocator<T>::allocator;

}


#endif	/* NUMAALLOCATOR_H */

