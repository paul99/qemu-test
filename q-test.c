
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#ifdef __mips__
#include <asm/cachectl.h>
#endif
#include <assert.h>
#define PAGE_SIZE 4 * 1024
#define PAGE_STRIDE PAGE_SIZE / 4
// ----- Fields offset and length.
const int kOpcodeShift   = 26;
const int kOpcodeBits    = 6;
const int kRsShift       = 21;
const int kRsBits        = 5;
const int kRtShift       = 16;
const int kRtBits        = 5;
const int kRdShift       = 11;
const int kRdBits        = 5;
const int kSaShift       = 6;
const int kSaBits        = 5;
const int kFunctionShift = 0;
const int kFunctionBits  = 6;
const int kLuiShift      = 16;

const unsigned int kImm16Shift = 0;
const unsigned int kImm16Bits  = 16;
const unsigned int kImm26Mask    = ((1 << 26) - 1);

unsigned int   J    =   ((0 << 3) + 2) << 26;
unsigned int  JAL   =   ((0 << 3) + 3) << 26;
unsigned int  LW    =   ((4 << 3) + 3) << 26;
unsigned int  SW    =   ((5 << 3) + 3) << 26;
unsigned int ADDIU     =   ((1 << 3) + 1) << 26;

void FlushICache(void* start, size_t size) {

#ifdef __mips__
  // Nothing to do, flushing no instructions.
  if (size == 0) {
    return;
  }
  int res;
  // See http://www.linux-mips.org/wiki/Cacheflush_Syscall.
  res = syscall(__NR_cacheflush, start, size, ICACHE);
  if (res) {
    assert(!res);
  }
#endif

}

void initSetup(unsigned int* exec_buff) {

  unsigned int* address;
  unsigned int inst;
  const int  kImm16Mask  = ((1 << kImm16Bits) - 1) << kImm16Shift;

  inst = ADDIU | ( 29  << kRsShift) | ( 29 << kRtShift)
      | ((-4) & kImm16Mask);
  exec_buff[0] = inst; // addiu sp, sp, -4

  inst = SW | ( 29  << kRsShift) | ( 31 << kRtShift)
      | ( 0 & kImm16Mask);
  exec_buff[1] = inst; // sw ra, 0(sp);
  int i;
  for (i=0; i < 5; i++) {

    address = &exec_buff[(i+2) * PAGE_STRIDE  + 256];
    inst =  (((unsigned int)address & 0x0fffffff) >> 2 ) | 0x0c000000;

    exec_buff[(i + 1) * PAGE_STRIDE - 1] = inst; // jal inserting at page boundary

  }
  inst = LW | ( 29  << kRsShift) | ( 31 << kRtShift)
      | ( 0 & kImm16Mask);
  exec_buff[7 * PAGE_STRIDE - 4] = inst; // lw ra, 0(sp);

   inst = ADDIU | ( 29  << kRsShift) | ( 29 << kRtShift)
      | ((4) & kImm16Mask);
  exec_buff[7 * PAGE_STRIDE - 3 ] = inst; // addiu sp, sp, 4

  exec_buff[7 * PAGE_STRIDE + 2] =  0x03E00008;  // jr ra;

  FlushICache( (void*)exec_buff, 8 * PAGE_SIZE);
}


void modifyCode(unsigned int* exec_buff, unsigned int targetOffsetInPage,
                  unsigned int jumpAddrOffset) {
  // Patch the jals at end of each page with new address.
  int i;
  unsigned int* address;
  unsigned int inst;

  for (i=0; i < 5; i++) {

    address = &exec_buff[(i+2) * PAGE_STRIDE  + targetOffsetInPage/4];
    inst =  (((unsigned int)address & 0x0fffffff) >> 2 ) | 0x0c000000;

    exec_buff[(i + 1) * jumpAddrOffset + 8] = inst; // jal inserting
    FlushICache( (void*)&exec_buff[(i + 1) * PAGE_SIZE - 1], 4);
  }

}


void AllocateBuffers(void** buffers_list, int num_buffs ) {

  int i;
  for(i = 0; i < num_buffs; i++) {
    posix_memalign(&buffers_list[i], PAGE_SIZE,  2 * PAGE_SIZE);
    printf("address buffer no. %d: %p\n", i, buffers_list[i]);
    memset(buffers_list[i], 0, 2 * PAGE_SIZE );
    mprotect(buffers_list[i], 2 * PAGE_SIZE, PROT_READ | PROT_WRITE | PROT_EXEC);
  }

}


void FreeBuffers(void** buffer_list, int num_buffs) {

  int i;
  for(i = 0; i < num_buffs; i++) {
      free(buffer_list[i]);
  }

}

void InitChain(void** buffers_list, int buffer_num)  {

  //First block is fixed in chain, as it contains ra store;
   unsigned int* current_buffer, *next_buffer;
   current_buffer = (unsigned int*) buffers_list[0];

  unsigned int* address;
  unsigned int inst;
  const int  kImm16Mask  = ((1 << kImm16Bits) - 1) << kImm16Shift;

  inst = ADDIU | ( 29  << kRsShift) | ( 29 << kRtShift)
      | ((-4) & kImm16Mask);
  current_buffer[0] = inst; // addiu sp, sp, -4

  inst = SW | ( 29  << kRsShift) | ( 31 << kRtShift)
      | ( 0 & kImm16Mask);
  current_buffer[1] = inst; // sw ra, 0(sp);
  FlushICache( (void*)current_buffer, 2 * PAGE_SIZE);

  int buffer_index;
  for (buffer_index = 0; buffer_index < buffer_num - 1; buffer_index++) {

    current_buffer = (unsigned int*) buffers_list[buffer_index];
    next_buffer = (unsigned int*) buffers_list[buffer_index + 1];
    address = &(next_buffer[256]);
    inst =  (((unsigned int)address & 0x0fffffff) >> 2 ) | 0x0c000000;
    current_buffer[ PAGE_STRIDE - 1] = inst; // jal inserting at page boundary
    FlushICache( (void*)current_buffer, 2 * PAGE_SIZE);
  }

  current_buffer = (unsigned int*) buffers_list[buffer_num -1 ];
    inst = LW | ( 29  << kRsShift) | ( 31 << kRtShift)
      | ( 0 & kImm16Mask);
  current_buffer[PAGE_STRIDE - 4] = inst; // lw ra, 0(sp);

   inst = ADDIU | ( 29  << kRsShift) | ( 29 << kRtShift)
      | ((4) & kImm16Mask);
  current_buffer[PAGE_STRIDE - 3 ] = inst; // addiu sp, sp, 4

  current_buffer[PAGE_STRIDE + 2] =  0x03E00008;  // jr ra;

  FlushICache( (void*)current_buffer, 2 * PAGE_SIZE);

}

void ShuffleChain(void** buffers_list, int buffer_num, int seed)  {

  int free_list[buffer_num];
  unsigned int inst;
  unsigned int* target_address;
  memset(free_list, 0, sizeof(int) * buffer_num);

  free_list[0] = 1; // First and last block cannot be selected by others.
  free_list[buffer_num - 1] = 1;

  int i;
  unsigned int* address;
  int next_block = 1;
  int index = 0, probe = 0;
  unsigned int* current_buffer = (unsigned int*) buffers_list[0];
  for(i = 0; i < buffer_num - 3; i++) {


    while(1) {

      next_block = (seed + probe) % buffer_num;
      if(free_list[next_block] == 0 && index != next_block) {
        // Found unchained block.
        free_list[next_block] = 1;
        //printf("buffer id: %x, chained index: %x\n", index, next_block);
        break;
      } else {
         probe += 1;
      }
    }



    address = (unsigned int*) buffers_list[next_block];
    target_address = &address[seed / 4];
    current_buffer = (unsigned int*) buffers_list[index];
    inst =  (((unsigned int)target_address & 0x0fffffff) >> 2 ) | 0x0c000000;
    current_buffer[PAGE_STRIDE - 1] = inst; // jal inserting at page boundary
    FlushICache( (void*)&current_buffer[PAGE_STRIDE - 1], sizeof(inst));
    index = next_block;
 }

 // Chain the next to the last block to the last fixed block.
  current_buffer = (unsigned int*) buffers_list[index];;
  //printf("buffer id: %x, chained index: 7\n", index);
  address = (unsigned int*) buffers_list[buffer_num - 1];
  target_address = &address[0];

  inst =  (((unsigned int)target_address & 0x0fffffff) >> 2 ) | 0x0c000000;
  current_buffer[PAGE_STRIDE - 1] = inst; // jal inserting at page boundary
  FlushICache( (void*)&current_buffer[PAGE_STRIDE - 1], sizeof(inst));


}

void ZapBlock(unsigned int* block, unsigned int sizeInBytes, unsigned int value) {

  unsigned int i;
  for(i=0; i < sizeInBytes/4; i++) {
     block[i] = value;
  }

}

void InsertNewBlock(void** buffer_list, int buffer_num, void** swap_buffer, int insertAtIndex) {

  void* new_buffer;
  unsigned int* current_buffer, *target_address, *previous_target_buffer;
  unsigned int inst;

  new_buffer = *swap_buffer;
  memset(new_buffer, 0, 2 * PAGE_SIZE);
  // Extract target address to the next block in chain from old block;
  current_buffer = (unsigned int*)buffer_list[0];
  int index = 1;
  do {
    //Address from jal.
    target_address =  0xf0000000 & (unsigned int)&current_buffer[PAGE_STRIDE - 1 ];
    target_address = (unsigned int)target_address | ((current_buffer[PAGE_STRIDE - 1 ] & kImm26Mask) << 2);
    previous_target_buffer = current_buffer;
    current_buffer = (unsigned int)target_address  &  (~(PAGE_SIZE - 1));
    index++;
  } while ( index < insertAtIndex);

  target_address = current_buffer[PAGE_STRIDE -1 ] & kImm26Mask;
  // Zap block with break instruction, this will provoke traps if prevoius block
  // is unchained correctly.
  ZapBlock(current_buffer, 2 * PAGE_SIZE, 0x0037ab4d);
  FlushICache( (void*)current_buffer, 2 * PAGE_SIZE );

  *swap_buffer = current_buffer;
  int i;
  for(i = 0; i < buffer_num; i++) {
    if(buffer_list[i] == current_buffer) {
      buffer_list[i] = new_buffer;
    }
  }

  current_buffer = (unsigned int*) new_buffer;
  // Set up jump in new code block.
  inst =  (unsigned int)target_address | 0x0c000000;
  current_buffer[PAGE_STRIDE - 1] = inst; // jal inserting at page boundary
  FlushICache( (void*)current_buffer, 2 * PAGE_SIZE);

  // Chain the new code block to previous block.
  target_address = &current_buffer[0];

  //printf("target address to new block: %p, current buffer: %p\n", target_address, previous_target_buffer);
  inst =  (((unsigned int)target_address & 0x0fffffff) >> 2 ) | 0x0c000000;
  previous_target_buffer[PAGE_STRIDE - 1] = inst; // jal inserting at page boundary
  FlushICache( (void*)&previous_target_buffer[PAGE_STRIDE - 1], sizeof(inst));

}

int main () {

  void  (*exec)(void);
  void* buffer, *swap_buffer;
  void* buffer_list[8];

  posix_memalign(&buffer, PAGE_SIZE, 8 * PAGE_SIZE);
  memset(buffer, 0, 8 * PAGE_SIZE );

  unsigned int* exec_buff = (unsigned int*)buffer;
  unsigned int* address;
  printf("address: %p\n", exec_buff);

  AllocateBuffers(buffer_list, 8);
  AllocateBuffers(&swap_buffer, 1);
  InitChain(buffer_list, 8);
  //initSetup(exec_buff);
  //mprotect(buffer, 8 * PAGE_SIZE, PROT_READ | PROT_WRITE | PROT_EXEC);

  int reps, iteration;
  for(reps = 0; reps < PAGE_SIZE/4; reps++)
    for(iteration = 0; iteration < PAGE_SIZE - 256; iteration++) {
      exec = (void (*)(void))buffer_list[0];
      (*exec)();

      ShuffleChain(buffer_list, 8, iteration);

      if( (reps + 4 )% 10 == 0) {
          // Cannot replace first (index 0) block and last block which
          // stores/restores ra to/from stack.
          // This is critical on qemu.
          InsertNewBlock(buffer_list, 8, &swap_buffer, 1+ iteration % 6);
      }
      //modifyCode(exec_buff, iteration, reps);
      printf("Iteration: %d\n", iteration);
  }

  FreeBuffers(buffer_list, 8);
  FreeBuffers(&swap_buffer, 1);
  free(buffer);
  return 0;
}
