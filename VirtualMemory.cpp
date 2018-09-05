#include <cmath>
#include <algorithm>
#include "VirtualMemory.h"
#include "PhysicalMemory.h"


void clearTable(uint64_t frameIndex) {
    for (uint64_t i = 0; i < PAGE_SIZE; ++i) {
        PMwrite(frameIndex * PAGE_SIZE + i, 0);
    }
}

void VMinitialize() {
    clearTable(0);
}


/**
 * Checks if a frame is empty (containing 0 in all of it's rows).
 * @param frameIndex the frame number to check.
 * @return true iff frame is empty.
 */
bool _isFrameEmpty(uint64_t frameIndex) {
    int numOfZeroRows = 0;
    for (uint64_t frameRow = 0; frameRow < PAGE_SIZE; ++frameRow) {
        word_t val;
        PMread(frameIndex * PAGE_SIZE + frameRow, &val);
        if (val == 0)
            numOfZeroRows++;
    }
    return numOfZeroRows == PAGE_SIZE;
}


/**
 * Supplying an empty frame in the RAM. done by traversing the tree according to DFS algorithm
 * @param maxFrame max index of the frame seen during the recursive run. if 0 is returned take maxFrame + 1 as frame.
 * @param frameIndex index of current frame.
 * @param page the page we are looking a frame for.
 * @param currentDepth current recursive depth.
 * @param maxCyclicDistance the max cyclic distance for the eviction.
 * @param frameToEvict will hold the index of the frame to evict in case 1 is returned.
 * @param pageToEvict wiil hold the page index to evict in case 1 is returned.
 * @param frameToAvoid a frame we use while traversing the tree
 * @param emptyFrame a reference that will hold the empty frame if there is one.
 * @param addressToEmpty the address to the empty frame.
 * @return 0 or 1. 0 means take (maxFrame + 1) as next frame and 1 means evict frameToEvict and use it.
 */
int _find_frame(uint64_t &maxFrame, uint64_t frameIndex, uint64_t page, int currentDepth, int &maxCyclicDistance,
                uint64_t &frameToEvict, uint64_t &pageToEvict, uint64_t frameToAvoid, uint64_t p,
                uint64_t &emptyFrame, uint64_t &addressToEmpty) {
    maxFrame = std::max(maxFrame, frameIndex);

    if (currentDepth == TABLES_DEPTH){ // if we reached a leaf of the tree we calc distance
        if (frameIndex != frameToAvoid){
            int dist = std::min(((int)NUM_PAGES - std::abs((int)page - (int)p)), std::abs((int)page - (int)p));
            maxCyclicDistance = std::max(maxCyclicDistance, dist);
            if (dist == maxCyclicDistance) {
                frameToEvict = frameIndex;
                pageToEvict = p;
            }
        }
        return 1; // 1 == evict frameToEvict and use it
    }
    int numOfZeroRows = 0;
    for (uint64_t frameRow = 0; frameRow < PAGE_SIZE; ++frameRow) {
        word_t val;
        PMread(frameIndex * PAGE_SIZE + frameRow, &val);
        if (val != 0){
            _find_frame(maxFrame, (uint64_t)val, page, currentDepth + 1, maxCyclicDistance, frameToEvict,
                    pageToEvict, frameToAvoid, (p << OFFSET_WIDTH) + frameRow, emptyFrame, addressToEmpty);
        } else numOfZeroRows++;
    }
    if (numOfZeroRows == PAGE_SIZE){
        if (frameIndex != frameToAvoid){
            emptyFrame = frameIndex;
            addressToEmpty = (p << (OFFSET_WIDTH * (TABLES_DEPTH - currentDepth)));
        }
    }
    if (currentDepth == 0){ // back from the recursive call checks if maxFrame + 1 is empty, or if found an empty frame along the way
        // ,else evict
        if (maxFrame + 1 < NUM_FRAMES) {
            emptyFrame = maxFrame + 1;
            clearTable(emptyFrame);
            return 0; // 0 == take (maxFrame + 1) as next frame
        }
        if (emptyFrame != 0)
            return 0;

        return 1; // 1 == evict frameToEvict and use it
    }
    return 2;
}



/**
 * unlinks the parent of an allocated frame
 * @param page - the page which leads the path to the given frame
 * @param frame - the frame which needs to be unlinked from it's parent
 * @param add_width - the width of the address
 */
void _unlink_parent(uint64_t page, word_t frame, int add_width)
{
    word_t address = 0;
    uint64_t currentOffset, currentRAMRow;
    for (int i = 1; i <= (add_width / OFFSET_WIDTH); ++i) {
        currentOffset = (page >> (add_width - (i * OFFSET_WIDTH))) % (1LL << OFFSET_WIDTH);
        currentRAMRow = (address * PAGE_SIZE) + currentOffset;
        PMread(currentRAMRow, &address);
        if (address == frame) //if currentRAMRow links to page-> link it to 0
        {
            PMwrite(currentRAMRow, 0); //links page's parent to 0
            break;
        }
    }
}

/**
 * this function builds the path in the virtual memory tree towards the given page
 * @param page - the destination page need to be written to / read from
 * @return the final physical frame in which the page was stored
 */
word_t _traverse_tree(uint64_t page)
{
    word_t address = 0;
    // next 3 rows handles a situation where PAGE_SIZE doesn't divide VIRTUAL_ADDRESS_WIDTH
    int rounded_add_width = VIRTUAL_ADDRESS_WIDTH - OFFSET_WIDTH;
    if (rounded_add_width % OFFSET_WIDTH)
        rounded_add_width += (OFFSET_WIDTH - (rounded_add_width % OFFSET_WIDTH));

    uint64_t currentOffset;
    uint64_t currentRAMRow = 0; //first frame access is 0
    for (int i = 1; i <= std::ceil((float(rounded_add_width) / OFFSET_WIDTH)); ++i) {
        //next row reads the next offset (row in frame)
        currentOffset = (page >> (rounded_add_width - (i * OFFSET_WIDTH))) % (1LL << OFFSET_WIDTH);
        //nextRAMrow gets the specific memory address according to the frame and offset
        currentRAMRow = (address * PAGE_SIZE) + currentOffset;
        uint64_t frameToAvoid = address;
        PMread(currentRAMRow, &address); //reads the next address
        if (address == 0)
        {
            //finds an empty frame by calling _find_frame
            uint64_t maxFrame = 0;
            int maxCyclicDistance = -1;
            uint64_t frameToEvict, pageToEvict;
            uint64_t p = 0;
            uint64_t addressToEmpty = 0;
            uint64_t emptyFrame = 0;
            if (!(word_t)_find_frame(maxFrame, 0, page, 0, maxCyclicDistance, frameToEvict,
                                     pageToEvict, frameToAvoid, p, emptyFrame, addressToEmpty)) {
                // found an empty frame
                address = emptyFrame;
                if (maxFrame + 1 >= NUM_FRAMES) //empty frame was used before->need to unlink parent
                    _unlink_parent(addressToEmpty, emptyFrame, rounded_add_width);
            }
            else //RAM was full- need to evict a frame
            {
                PMevict(frameToEvict, pageToEvict);
                clearTable(frameToEvict);
                address = frameToEvict;
                _unlink_parent(pageToEvict, frameToEvict,rounded_add_width);
            }
            PMwrite(currentRAMRow, address); //writes the new empty frame in currentRAMRow
        }
    }
    //at the end of the loop, address should hold the final frame in physical mem
    PMrestore(address, page);
    return address;
}

int VMread(uint64_t virtualAddress, word_t* value) {
    //splits the virtualAddress to page and offset
    uint64_t offset = virtualAddress % (2 << (OFFSET_WIDTH - 1));
    uint64_t page = virtualAddress >> OFFSET_WIDTH;
    //next row gets the final frame which page was stored in
    word_t final_physical_add = _traverse_tree(page);
    //next row reads the value in RAM according to the final frame which was allocated
    PMread(final_physical_add * PAGE_SIZE + offset, value);
    return 1;
}

int VMwrite(uint64_t virtualAddress, word_t value) {
    //splits the virtualAddress to page and offset
    uint64_t offset = virtualAddress % (2 << (OFFSET_WIDTH - 1));
    uint64_t page = virtualAddress >> OFFSET_WIDTH;
    //next row gets the final frame which page was stored in
    word_t final_physical_add = _traverse_tree(page);
    //next row writes the value in RAM according to the final frame which was allocated
    PMwrite(final_physical_add * PAGE_SIZE + offset, value);
    return 1;
}
