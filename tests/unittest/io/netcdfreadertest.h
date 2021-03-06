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
 * @copyright 2012 Sebastian Rettenberger <rettenbs@in.tum.de>
 */

#include "globaltest.h"
#include "tests.h"

#undef NDEBUG

#include "io/netcdfreader.h"

class NetCdfReaderTest : public CxxTest::TestSuite
{
	io::NetCdfReader* file3d;
public:
	void setUp(void)
	{
		file3d = new io::NetCdfReader("../../" NC_3D, 0);
		file3d->open();
	}
	
	void tearDown(void)
	{
		delete file3d;
	}
	
	void testGetDimensions(void)
	{
		TS_ASSERT_EQUALS(file3d->getDimensions(), 3);
	}
	
	void testGetSize(void)
	{
		TS_ASSERT_EQUALS(file3d->getSize(0),
			static_cast<unsigned long>(NC_WIDTH));
		TS_ASSERT_EQUALS(file3d->getSize(1),
			static_cast<unsigned long>(NC_LENGTH));
		TS_ASSERT_EQUALS(file3d->getSize(2),
			static_cast<unsigned long>(NC_HEIGHT));
	}
	
	/**
	 * @todo This test only tests 3d grids at the moment
	 */
	void testGetBlock(void)
	{
		float values[10][10][10];

		size_t offsets[3] = {0, 0, 0};
		size_t sizes[3] = {10, 10, 10};
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<float>(values, offsets, sizes));
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < 10; j++)
				for (int k = 0; k < 10; k++)
					TS_ASSERT_EQUALS(values[i][j][k], calcValueAt(i, j, k));
		
		size_t offsets2[3] = {9, 4, 45};
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<float>(values, offsets2, sizes));
		for (int i = 0; i < NC_HEIGHT-45; i++)
			for (int j = 0; j < 10; j++)
				for (int k = 0; k < 10; k++)
					TS_ASSERT_EQUALS(values[i][j][k], calcValueAt(i+45, j+4, k+9));

		size_t offsets3[3] = {0, 46, 21};
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<float>(values, offsets3, sizes));
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < NC_LENGTH-46; j++)
				for (int k = 0; k < 10; k++)
					TS_ASSERT_EQUALS(values[i][j][k], calcValueAt(i+21, j+46, k+0));

		size_t offsets4[3] = {47, 10, 0};
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<float>(values, offsets4, sizes));
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < 10; j++)
				for (int k = 0; k < NC_LENGTH-47; k++)
					TS_ASSERT_EQUALS(values[i][j][k], calcValueAt(i+0, j+10, k+47));

		float values2[50][50][50];
		size_t offsets5[3] = {50, 50, 0};
		size_t sizes2[3] = {50, 50, 50};
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<float>(values2, offsets5, sizes2));
		for (int i = 0; i < 50; i++)
			for (int j = 0; j < NC_LENGTH-50; j++)
				for (int k = 0; k < NC_WIDTH-50; k++)
					TS_ASSERT_EQUALS(values2[i][j][k], calcValueAt(i, j+50, k+50));

		// Test float to double conversion
		double values3[10][10][10];
		TS_ASSERT_THROWS_NOTHING(file3d->getBlock<double>(values3, offsets3, sizes));
		for (int i = 0; i < 10; i++)
			for (int j = 0; j < NC_LENGTH-46; j++)
				for (int k = 0; k < 10; k++)
					TS_ASSERT_EQUALS(values3[i][j][k], calcValueAt(i+21, j+46, k+0));

		// Test void
		/*TS_ASSERT_THROWS_NOTHING*/(file3d->getBlock<void>(values2, offsets5, sizes2));
		for (int i = 0; i < 50; i++)
			for (int j = 0; j < NC_LENGTH-50; j++)
				for (int k = 0; k < NC_WIDTH-50; k++)
					TS_ASSERT_EQUALS(values2[i][j][k], calcValueAt(i, j+50, k+50));
	}

private:
	/**
	 * @return The value that should be in the netcdf file at x, y, z
	 */
	float calcValueAt(int z, int y, int x)
	{
		return (z * NC_LENGTH + y) * NC_WIDTH + x;
	}
};
