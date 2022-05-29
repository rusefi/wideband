/*
 * FragmentEntry.cpp
 *
 *  Created on: Jan 5, 2022
 * @author Andrey Belomutskiy, (c) 2012-2022
 */

#include <string.h>

#include "efilib.h"

#include "FragmentEntry.h"
#include "livedata.h"

void copyRange(uint8_t* destination, FragmentList src, size_t skip, size_t size) {
	int fragmentIndex = 0;

	// Find which fragment to start - skip any full fragments smaller than `skip` parameter
	while (skip > src.fragments[fragmentIndex].size && fragmentIndex <= src.count) {
		skip -= src.fragments[fragmentIndex].size;
		fragmentIndex++;
	}

	int destinationIndex = 0;

	while (size > 0) {
		if (fragmentIndex >= src.count) {
			// somehow we are past the end of fragments - fill with zeros
			memset(destination + destinationIndex, 0, size);
			return;
		}

		int copyNowSize = minI(size, src.fragments[fragmentIndex].size - skip);
		const uint8_t* fromBase = src.fragments[fragmentIndex].data;
		if (!fromBase) {
			// we have no buffer for this fragment - fill with zeroes
			memset(destination + destinationIndex, 0, copyNowSize);
		} else {
			memcpy(destination + destinationIndex, fromBase + skip, copyNowSize);
		}
		destinationIndex += copyNowSize;
		skip = 0;
		size -= copyNowSize;
		fragmentIndex++;
	}
}

static const FragmentEntry fragments[] = {
	//reinterpret_cast</* const */ livedata_common_s*>(&livedata_common),
	//reinterpret_cast</* const */ livedata_afr_s*>(&livedata_afr),
	FragmentEntry((uint8_t *)&livedata_common, sizeof(livedata_common)),
	FragmentEntry((uint8_t *)&livedata_afr, sizeof(livedata_afr)),
};

FragmentList getFragments() {
	return { fragments, efi::size(fragments) };
}
