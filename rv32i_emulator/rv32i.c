/*
 *  kianv c emulator RISC-V
 *
 *  copyright (c) 2023 hirosh dabui <hirosh@dabui.de>
 *
 *  permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  the software is provided "as is" and the author disclaims all warranties
 *  with regard to this software including all implied warranties of
 *  merchantability and fitness. in no event shall the author be liable for
 *  any special, direct, indirect, or consequential damages or any damages
 *  whatsoever resulting from loss of use, data or profits, whether in an
 *  action of contract, negligence or other tortious action, arising out of
 *  or in connection with the use or performance of this software.
 *
 */
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>


int fileno(FILE *stream);
struct rv32i_kian_state_t;

#define RISCV32_KIAN_IO_BASE 10000000

#define SB(state, offset, data) *(uint8_t *)(memory + offset) = data
#define SH(state, offset, data) *(uint16_t *)(memory + offset) = data
#define SW(state, offset, data) *(uint32_t *)(memory + offset) = data

#define LB(state, offset) *(int8_t *)(memory + offset)
#define LH(state, offset) *(int16_t *)(memory + offset)
#define LW(state, offset) *(uint32_t *)(memory + offset)
#define LBU(state, offset) *(uint8_t *)(memory + offset)
#define LHU(state, offset) *(uint16_t *)(memory + offset)

#define MEM_SIZE (1024 * 1024 * 32)
#define EXCEPTION(l)                                                           \
  do {                                                                         \
    printf("Exception found in %d\n", l);                                      \
    for (;;)                                                                   \
      ;                                                                        \
  } while (0)

uint32_t test_memory;
uint8_t *memory;

static void ResetKeyboardTerm() {
  struct termios term;
  tcgetattr(0, &term);
  term.c_lflag |= ICANON | ECHO;
  tcsetattr(0, TCSANOW, &term);
}

void SetKeyboardTerm() {
  struct termios term;
  tcgetattr(0, &term);
  term.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(0, TCSANOW, &term);
}

uint32_t MEMORY_IOMEM_LOAD(struct rv32i_kian_state_t *state, uint32_t addr) {
  (void) state; //unused

  //   printf("LOAD MEMORY_IOMEM:%08x\n", addr);
  if (addr == 0x10000000) {
    return test_memory;
  } else if (addr == 0x30000000) {
    return 1;
  } else if (addr == 0x3000003C) {
    char c;
    read(fileno(stdin), (char *)&c, 1) > 0 ? c : EOF;
    return c == EOF ? ~0 : c;
  } else {
    printf("read to none supported hw address:%08x\n", addr);
    EXCEPTION(__LINE__);
  }
  return 0;
}

void MEMORY_IOMEM_STORE(struct rv32i_kian_state_t *state, uint32_t addr, uint32_t data) {
  (void) state; //unused
  
  if (addr == 0x10000000) {
    test_memory = (test_memory << 8) | (data & 0xff);
  } else if (addr == 0x30000000) {
    printf("%c", data);
    fflush(stdout);
  } else {
    printf("write to none supported hw address:%08x\n", addr);
    EXCEPTION(__LINE__);
  }
}

#include "rv32i.h"

void ebreak() {
  if ((test_memory & 0x00ffffff) == 0x4f4b0a)
    printf("passed\n");
  else
    printf("failed\n");
  fflush(stdout);
  exit(0);
}

uint32_t GetInstr(struct rv32i_kian_state_t *state) { return LW(state, state->PC); }

void LoadFirmware(char *firmware) {
  FILE *fp = fopen(firmware, "rb");
  fseek(fp, 0, SEEK_END);
  long size = ftell(fp);
  fseek(fp, 0, SEEK_SET);
  fread(memory, size, 1, fp);
  fclose(fp);
}

struct rv32i_kian_state_t cpu;
int main(int argc, char **argv) {
  atexit(ResetKeyboardTerm);
  SetKeyboardTerm();
  memory = malloc(MEM_SIZE);
  memset(memory, 0, MEM_SIZE);

  LoadFirmware("firmware.bin");

  cpu.PC = 0;
  do {
    rv32i_kian_execute(&cpu, GetInstr(&cpu));
    rv32i_kian_retire(&cpu);
  } while (1);

  return 0;
}
