/**
 * @file
 *  This file is part of ASAGI.
 * 
 *  ASAGI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of
 *  the License, or  (at your option) any later version.
 *
 *  ASAGI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with ASAGI.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Diese Datei ist Teil von ASAGI.
 *
 *  ASAGI ist Freie Software: Sie koennen es unter den Bedingungen
 *  der GNU Lesser General Public License, wie von der Free Software
 *  Foundation, Version 3 der Lizenz oder (nach Ihrer Option) jeder
 *  spaeteren veroeffentlichten Version, weiterverbreiten und/oder
 *  modifizieren.
 *
 *  ASAGI wird in der Hoffnung, dass es nuetzlich sein wird, aber
 *  OHNE JEDE GEWAEHELEISTUNG, bereitgestellt; sogar ohne die implizite
 *  Gewaehrleistung der MARKTFAEHIGKEIT oder EIGNUNG FUER EINEN BESTIMMTEN
 *  ZWECK. Siehe die GNU Lesser General Public License fuer weitere Details.
 *
 *  Sie sollten eine Kopie der GNU Lesser General Public License zusammen
 *  mit diesem Programm erhalten haben. Wenn nicht, siehe
 *  <http://www.gnu.org/licenses/>.
 * 
 * @copyright 2012-2015 Sebastian Rettenberger <rettenbs@in.tum.de>
 */

#include "globaltest.h"
#include "tests.h"

#include "grid/grid.h"
#include "grid/simplecontainer.h"
#include "grid/level/full.h"
#include "types/basictype.h"

class FullTest : public CxxTest::TestSuite
{
	grid::Grid* c;
	grid::level::FullDefault<magic::NullType, magic::NullType, types::BasicType<float>>* grid;
public:
	void setUp(void)
	{
		// Set up a 1d grid
		c = new grid::Grid(asagi::Grid::FLOAT);
		c->open("../../../" NC_1D);
		grid = &dynamic_cast<grid::SimpleContainer<grid::level::FullDefault<magic::NullType,
					magic::NullType, types::BasicType<float>>,
				magic::NullType, magic::NullType,
				types::BasicType<float>>*>(c->m_containers[0])->m_levels[0];

		TS_ASSERT(grid);
	}
	
	void tearDown(void)
	{
		delete c;
	}
	
	void testTotalBlockSize(void)
	{
		TS_ASSERT_EQUALS(grid->totalBlockSize(), 64ul);
		/*
		TS_ASSERT_EQUALS(grid->m_blockSize[0], 5u);
		
		grid->setParam("y-block-size", "7");
		TS_ASSERT_EQUALS(grid->m_blockSize[1], 7u);
		
		grid->setParam("z-block-size", "42");
		TS_ASSERT_EQUALS(grid->m_blockSize[2], 42u);
		
		TS_ASSERT_EQUALS(grid->setParam("block-cache-size", "100"),
			asagi::Grid::SUCCESS);
		TS_ASSERT_EQUALS(grid->m_blocksPerNode, 100);
		*/
	}

	void testLocal2global(void)
	{
		TS_ASSERT_EQUALS(grid->local2global(0), 0ul);
		TS_ASSERT_EQUALS(grid->local2global(1), 1ul);
	}

	void testGetXMax(void)
	{
		//TS_ASSERT_EQUALS(c->getFloat1D(c->getXMax()), NC_WIDTH-1);
	}

	void testGetXDelta(void)
	{
		//TS_ASSERT_DELTA(c->getXDelta(), 1.0, 0.001);
	}
};
