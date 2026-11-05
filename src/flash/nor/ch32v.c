// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Flash driver for the WCH CH32V003, CH32V00X (CH32V002, CH32V004,
 * CH32V005, CH32V006, CH32V007), CH32V20x and CH32V30x.
 *
 * Written from the reference manuals: CH32V003RM (chapter 16),
 * CH32V00XRM (chapter 18) and CH32FV2x_V3xRM (chapter 32).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/riscv/riscv.h>

#define CH32V_FLASH_BASE	0x08000000

#define FLASH_REG_BASE		0x40022000
#define FLASH_KEYR		(FLASH_REG_BASE + 0x04)
#define FLASH_OBKEYR		(FLASH_REG_BASE + 0x08)
#define FLASH_STATR		(FLASH_REG_BASE + 0x0c)
#define FLASH_CTLR		(FLASH_REG_BASE + 0x10)
#define FLASH_ADDR		(FLASH_REG_BASE + 0x14)
#define FLASH_OBR		(FLASH_REG_BASE + 0x1c)
#define FLASH_WPR		(FLASH_REG_BASE + 0x20)
#define FLASH_MODEKEYR		(FLASH_REG_BASE + 0x24)

#define FLASH_KEY1		0x45670123
#define FLASH_KEY2		0xcdef89ab

#define STATR_BSY		BIT(0)
#define STATR_WRPRTERR		BIT(4)
#define STATR_EOP		BIT(5)
#define STATR_EHMODS		BIT(7)	// CH32V20x only

#define CTLR_MER		BIT(2)
#define CTLR_OBPG		BIT(4)
#define CTLR_OBER		BIT(5)
#define CTLR_STRT		BIT(6)
#define CTLR_LOCK		BIT(7)
#define CTLR_OBWRE		BIT(9)
#define CTLR_FLOCK		BIT(15)
#define CTLR_FTPG		BIT(16)
#define CTLR_FTER		BIT(17)
#define CTLR_RSENACT		BIT(22)	// CH32V20x only
#define CTLR_EHMOD		BIT(24)	// CH32V20x only

#define OBR_RDPRT		BIT(1)

#define OPTION_BYTES_BASE	0x1ffff800
#define OPTION_RDPR_UNPROTECTED	0xa5
#define OPTION_RDPR_PROTECTED	0x00

#define ESIG_FLACAP		0x1ffff7e0
#define CHIP_ID_V003		0x1ffff7c4
#define CHIP_ID_V00X_V20X	0x1ffff704

// Index of each option byte in the order of the option byte area
enum ch32v_option {
	OPT_RDPR,
	OPT_USER,
	OPT_DATA0,
	OPT_DATA1,
	OPT_WRPR0,
	OPT_COUNT = OPT_WRPR0 + 4,
};

enum ch32v_family {
	CH32V003,
	CH32V00X,
	CH32V20X,
	CH32V30X,
};

struct ch32v_family_info {
	const char *name;
	uint32_t page_size;
	// Bytes of flash for each bit of FLASH_WPR
	uint32_t wrp_unit;
	// Option bytes with an inverse byte, from the RDPR
	unsigned int option_count;
	// Fast page programming with WRBSY and PGSTRT, not BUFLOAD
	bool pgstrt_method;
	// An option byte write needs STRT in FLASH_CTLR before it
	bool option_strt;
	// Content of an erased word
	uint32_t erased_word;
	// Size of the whole flash in KiB, 0: ESIG_FLACAP gives it
	uint32_t flash_size_kb;
};

static const struct ch32v_family_info ch32v_families[] = {
	[CH32V003] = {
		.name = "CH32V003",
		.page_size = 64,
		.wrp_unit = 1024,
		.option_count = 6,
		.erased_word = 0xffffffff,
	},
	[CH32V00X] = {
		.name = "CH32V00X",
		.page_size = 256,
		.wrp_unit = 2048,
		.option_count = 8,
		.option_strt = true,
		.erased_word = 0xffffffff,
	},
	[CH32V20X] = {
		.name = "CH32V20x",
		.page_size = 256,
		.wrp_unit = 4096,
		.option_count = 8,
		.pgstrt_method = true,
		.erased_word = 0xe339e339,
	},
	[CH32V30X] = {
		.name = "CH32V30x",
		.page_size = 256,
		.wrp_unit = 4096,
		.option_count = 8,
		.pgstrt_method = true,
		.erased_word = 0xe339e339,
		// ESIG_FLACAP gives the largest zero-wait code area (288 KiB)
		.flash_size_kb = 480,
	},
};

struct ch32v_flash_bank {
	bool probed;
	uint32_t chip_id;
	const struct ch32v_family_info *info;
};

// The loader of contrib/loaders/flash/ch32v
static const uint8_t ch32v_flash_write_code[] = {
#include "../../../contrib/loaders/flash/ch32v/ch32v.inc"
};

FLASH_BANK_COMMAND_HANDLER(ch32v_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (bank->base != CH32V_FLASH_BASE) {
		LOG_ERROR("Flash bank base must be 0x%08x", CH32V_FLASH_BASE);
		return ERROR_FLASH_BANK_INVALID;
	}

	struct ch32v_flash_bank *ch32v_info = calloc(1, sizeof(*ch32v_info));
	if (!ch32v_info)
		return ERROR_FAIL;

	bank->driver_priv = ch32v_info;
	return ERROR_OK;
}

static int ch32v_wait_busy(struct flash_bank *bank, int timeout_ms)
{
	struct target *target = bank->target;
	int64_t then = timeval_ms() + timeout_ms;

	/* BSY 0 alone can be a read before the operation started: wait for EOP
	 * (or the error flag) as well. */
	uint32_t statr;
	for (;;) {
		int retval = target_read_u32(target, FLASH_STATR, &statr);
		if (retval != ERROR_OK)
			return retval;
		if (!(statr & STATR_BSY) && (statr & (STATR_EOP | STATR_WRPRTERR)))
			break;
		if (timeval_ms() > then) {
			LOG_ERROR("Timeout while waiting for the flash controller "
					"(FLASH_STATR 0x%08" PRIx32 ")", statr);
			return ERROR_FLASH_OPERATION_FAILED;
		}
		alive_sleep(1);
	}

	int retval = target_write_u32(target, FLASH_STATR, STATR_EOP | STATR_WRPRTERR);
	if (retval != ERROR_OK)
		return retval;

	if (statr & STATR_WRPRTERR) {
		LOG_ERROR("Flash write protection error");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

// Unlocks the FPEC and, with fast, the fast program and erase mode.
static int ch32v_unlock(struct flash_bank *bank, bool fast)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;

	uint32_t ctlr;
	int retval = target_read_u32(target, FLASH_CTLR, &ctlr);
	if (retval != ERROR_OK)
		return retval;

	/* A key write to an unlocked controller locks it until the next reset,
	 * so write the keys only when the lock bit is set. */
	if (ctlr & CTLR_LOCK) {
		retval = target_write_u32(target, FLASH_KEYR, FLASH_KEY1);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, FLASH_KEYR, FLASH_KEY2);
		if (retval != ERROR_OK)
			return retval;
	}
	if (fast && (ctlr & CTLR_FLOCK)) {
		retval = target_write_u32(target, FLASH_MODEKEYR, FLASH_KEY1);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, FLASH_MODEKEYR, FLASH_KEY2);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = target_read_u32(target, FLASH_CTLR, &ctlr);
	if (retval != ERROR_OK)
		return retval;
	if ((ctlr & CTLR_LOCK) || (fast && (ctlr & CTLR_FLOCK))) {
		LOG_ERROR("Flash controller is still locked (FLASH_CTLR 0x%08" PRIx32 "). "
				"Reset the target and try again.", ctlr);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	// Clear the error flag of an earlier failed operation.
	retval = target_write_u32(target, FLASH_STATR, STATR_EOP | STATR_WRPRTERR);
	if (retval != ERROR_OK)
		return retval;

	// Program and erase fail in the enhanced read mode of the CH32V20x.
	if (ch32v_info->info->pgstrt_method) {
		uint32_t statr;
		retval = target_read_u32(target, FLASH_STATR, &statr);
		if (retval != ERROR_OK)
			return retval;
		if (statr & STATR_EHMODS) {
			ctlr &= ~CTLR_EHMOD;
			retval = target_write_u32(target, FLASH_CTLR, ctlr);
			if (retval == ERROR_OK)
				retval = target_write_u32(target, FLASH_CTLR, ctlr | CTLR_RSENACT);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	return ERROR_OK;
}

static int ch32v_lock(struct flash_bank *bank)
{
	return target_write_u32(bank->target, FLASH_CTLR, CTLR_LOCK | CTLR_FLOCK);
}

static int ch32v_check_halted(struct flash_bank *bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	return ERROR_OK;
}

static int ch32v_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;

	int retval = ch32v_unlock(bank, false);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_CTLR, CTLR_MER);
	if (retval == ERROR_OK)
		retval = target_write_u32(target, FLASH_CTLR, CTLR_MER | CTLR_STRT);
	if (retval == ERROR_OK)
		retval = ch32v_wait_busy(bank, 3000);
	/* After a mass erase of a full flash, the CH32V208 returns wrong data
	 * for the first reads: 1 ms after EOP still fails, 3 ms passes. */
	if (retval == ERROR_OK)
		alive_sleep(10);

	int retval2 = target_write_u32(target, FLASH_CTLR, 0);
	if (retval2 == ERROR_OK)
		retval2 = ch32v_lock(bank);
	return retval != ERROR_OK ? retval : retval2;
}

static int ch32v_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	int retval = ch32v_check_halted(bank);
	if (retval != ERROR_OK)
		return retval;

	if (first == 0 && last == bank->num_sectors - 1)
		return ch32v_mass_erase(bank);

	retval = ch32v_unlock(bank, true);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_CTLR, CTLR_FTER);
	for (unsigned int i = first; i <= last && retval == ERROR_OK; i++) {
		retval = target_write_u32(target, FLASH_ADDR,
				bank->base + bank->sectors[i].offset);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, FLASH_CTLR, CTLR_FTER | CTLR_STRT);
		if (retval == ERROR_OK)
			retval = ch32v_wait_busy(bank, 100);
		keep_alive();
	}

	int retval2 = target_write_u32(target, FLASH_CTLR, 0);
	if (retval2 == ERROR_OK)
		retval2 = ch32v_lock(bank);
	return retval != ERROR_OK ? retval : retval2;
}

static int ch32v_write_pages(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t address, uint32_t page_count)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;
	const uint32_t page_size = ch32v_info->info->page_size;

	struct working_area *write_algorithm;
	if (target_alloc_working_area(target, sizeof(ch32v_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_ERROR("No working area available for the flash loader");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	int retval = target_write_buffer(target, write_algorithm->address,
			sizeof(ch32v_flash_write_code), ch32v_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	uint32_t buffer_size = target_get_working_area_avail(target) / page_size * page_size;
	buffer_size = MIN(buffer_size, page_count * page_size);
	struct working_area *source;
	if (buffer_size == 0 ||
			target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		LOG_ERROR("No working area available for one flash page (%" PRIu32 " bytes)",
				page_size);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct reg_param reg_params[6];
	init_reg_param(&reg_params[0], "a0", 32, PARAM_IN_OUT);	// FLASH base, result
	init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);	// page count
	init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);	// source buffer
	init_reg_param(&reg_params[3], "a3", 32, PARAM_IN_OUT);	// flash address
	init_reg_param(&reg_params[4], "a4", 32, PARAM_OUT);	// words per page
	init_reg_param(&reg_params[5], "a5", 32, PARAM_OUT);	// method

	struct riscv_algorithm algorithm_info = {
		.common_magic = RISCV_COMMON_MAGIC,
	};

	while (page_count > 0) {
		uint32_t pages = MIN(page_count, buffer_size / page_size);

		retval = target_write_buffer(target, source->address, pages * page_size, buffer);
		if (retval != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, FLASH_REG_BASE);
		buf_set_u32(reg_params[1].value, 0, 32, pages);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, address);
		buf_set_u32(reg_params[4].value, 0, 32, page_size / 4);
		buf_set_u32(reg_params[5].value, 0, 32, ch32v_info->info->pgstrt_method);

		retval = target_run_algorithm(target, 0, NULL,
				ARRAY_SIZE(reg_params), reg_params,
				write_algorithm->address,
				write_algorithm->address + sizeof(ch32v_flash_write_code) - 4,
				10000, &algorithm_info);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to run the flash loader at 0x%" TARGET_PRIxADDR,
					write_algorithm->address);
			break;
		}

		uint32_t statr = buf_get_u32(reg_params[0].value, 0, 32);
		if (statr != 0) {
			LOG_ERROR("Flash write failed at 0x%08" PRIx32 ", FLASH_STATR 0x%08" PRIx32,
					buf_get_u32(reg_params[3].value, 0, 32), statr);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += pages * page_size;
		address += pages * page_size;
		page_count -= pages;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);
	return retval;
}

static int ch32v_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;
	const uint32_t page_size = ch32v_info->info->page_size;

	int retval = ch32v_check_halted(bank);
	if (retval != ERROR_OK)
		return retval;

	/* Fast page programming writes whole pages. The write alignment of the
	 * bank makes the flash core pad a partial page. */
	if (offset % page_size || count % page_size) {
		LOG_ERROR("Write of 0x%" PRIx32 " bytes at offset 0x%" PRIx32
				" is not aligned to the page size", count, offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = ch32v_unlock(bank, true);
	if (retval == ERROR_OK)
		retval = target_write_u32(target, FLASH_CTLR, CTLR_FTPG);
	if (retval == ERROR_OK)
		retval = ch32v_write_pages(bank, buffer, bank->base + offset,
				count / page_size);

	int retval2 = target_write_u32(target, FLASH_CTLR, 0);
	if (retval2 == ERROR_OK)
		retval2 = ch32v_lock(bank);

	return retval != ERROR_OK ? retval : retval2;
}

/* Reads the option bytes: first the values that the chip loaded at reset
 * from FLASH_OBR and FLASH_WPR, then each valid byte of the option byte area,
 * which holds also changes after the last reset. */
static int ch32v_read_options(struct flash_bank *bank, uint8_t options[OPT_COUNT])
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;

	uint32_t obr, wpr;
	int retval = target_read_u32(target, FLASH_OBR, &obr);
	if (retval == ERROR_OK)
		retval = target_read_u32(target, FLASH_WPR, &wpr);
	if (retval != ERROR_OK)
		return retval;

	options[OPT_RDPR] = (obr & OBR_RDPRT) ? OPTION_RDPR_PROTECTED : OPTION_RDPR_UNPROTECTED;
	options[OPT_USER] = obr >> 2;
	if (ch32v_info->info->pgstrt_method) {
		/* FLASH_OBR has no DATA bytes. USER is OBR bits 9..2: a CH32V307
		 * with USER 0x9f reads 0x27c. The RM table gives SRAM_CODE_MODE as
		 * OBR bits 9..8, but it is bits 9..7. */
		options[OPT_DATA0] = 0xff;
		options[OPT_DATA1] = 0xff;
	} else {
		options[OPT_DATA0] = obr >> 10;
		options[OPT_DATA1] = obr >> 18;
	}
	for (unsigned int i = 0; i < 4; i++)
		options[OPT_WRPR0 + i] = wpr >> (8 * i);

	uint8_t area[2 * OPT_COUNT];
	unsigned int count = ch32v_info->info->option_count;
	if (target_read_buffer(target, OPTION_BYTES_BASE, 2 * count, area) != ERROR_OK)
		return ERROR_OK;
	for (unsigned int i = 0; i < count; i++) {
		if ((area[2 * i] ^ area[2 * i + 1]) == 0xff)
			options[i] = area[2 * i];
	}
	return ERROR_OK;
}

static int ch32v_write_options(struct flash_bank *bank, const uint8_t options[OPT_COUNT])
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;

	uint32_t obr;
	int retval = target_read_u32(target, FLASH_OBR, &obr);
	if (retval != ERROR_OK)
		return retval;

	uint32_t ctlr;
	retval = ch32v_unlock(bank, false);
	if (retval == ERROR_OK)
		retval = target_read_u32(target, FLASH_CTLR, &ctlr);
	if (retval == ERROR_OK && !(ctlr & CTLR_OBWRE)) {
		retval = target_write_u32(target, FLASH_OBKEYR, FLASH_KEY1);
		if (retval == ERROR_OK)
			retval = target_write_u32(target, FLASH_OBKEYR, FLASH_KEY2);
		if (retval == ERROR_OK)
			retval = target_read_u32(target, FLASH_CTLR, &ctlr);
		if (retval == ERROR_OK && !(ctlr & CTLR_OBWRE)) {
			LOG_ERROR("Option bytes are still locked");
			retval = ERROR_FLASH_OPERATION_FAILED;
		}
	}

	// Each FLASH_CTLR write keeps OBWRE: a 0 locks the option bytes.
	if (retval == ERROR_OK)
		retval = target_write_u32(target, FLASH_CTLR, CTLR_OBER | CTLR_OBWRE);
	if (retval == ERROR_OK)
		retval = target_write_u32(target, FLASH_CTLR, CTLR_OBER | CTLR_STRT | CTLR_OBWRE);
	if (retval == ERROR_OK)
		retval = ch32v_wait_busy(bank, 3000);
	const bool erased = retval == ERROR_OK;

	/* The FPEC programs the inverse byte too, but the 16-bit write must hold
	 * it: with the low byte alone, the half word stays unprogrammed. The
	 * CH32V00X needs STRT before each half word, else the half word is lost.
	 * Fast page programming of the option bytes (CH32V00X RM 18.5.2) does
	 * not work through the debug module: FTPG does not set.
	 * Set OBPG alone first: with OBPG and STRT in one write after the
	 * erase, the CH32V00X loses the first half word (RDPR). */
	uint32_t obpg = CTLR_OBPG | CTLR_OBWRE;
	if (retval == ERROR_OK)
		retval = target_write_u32(target, FLASH_CTLR, obpg);
	if (ch32v_info->info->option_strt)
		obpg |= CTLR_STRT;
	for (unsigned int i = 0; i < ch32v_info->info->option_count && retval == ERROR_OK; i++) {
		uint16_t hword = options[i] | (uint16_t)(~options[i] & 0xff) << 8;
		retval = target_write_u32(target, FLASH_CTLR, obpg);
		if (retval == ERROR_OK)
			retval = target_write_u16(target, OPTION_BYTES_BASE + 2 * i, hword);
		if (retval == ERROR_OK)
			retval = ch32v_wait_busy(bank, 3000);
		/* An unprotect erases the main flash after the RDPR write. The
		 * CH32V00X needs about 110 ms for it, and BSY does not show it. */
		if (retval == ERROR_OK && i == OPT_RDPR && (obr & OBR_RDPRT) &&
				options[OPT_RDPR] == OPTION_RDPR_UNPROTECTED)
			alive_sleep(110);
	}
	if (erased && retval != ERROR_OK)
		LOG_ERROR("Option bytes are erased but not written again. After a reset, "
				"the chip is read protected. 'ch32v unlock' removes the "
				"protection and erases the flash.");

	int retval2 = target_write_u32(target, FLASH_CTLR, 0);
	if (retval2 == ERROR_OK)
		retval2 = ch32v_lock(bank);
	if (retval == ERROR_OK && retval2 == ERROR_OK)
		LOG_INFO("Option bytes written. Reset the target to load them.");
	return retval != ERROR_OK ? retval : retval2;
}

static int ch32v_protect_check(struct flash_bank *bank)
{
	uint32_t wpr;
	int retval = target_read_u32(bank->target, FLASH_WPR, &wpr);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
		bank->prot_blocks[i].is_protected = !(wpr & BIT(i));
	return ERROR_OK;
}

static int ch32v_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int retval = ch32v_check_halted(bank);
	if (retval != ERROR_OK)
		return retval;

	uint8_t options[OPT_COUNT];
	retval = ch32v_read_options(bank, options);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		// A bit of 0 protects the block.
		if (set)
			options[OPT_WRPR0 + i / 8] &= ~BIT(i % 8);
		else
			options[OPT_WRPR0 + i / 8] |= BIT(i % 8);
	}
	return ch32v_write_options(bank, options);
}

static int ch32v_erase_check(struct flash_bank *bank)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	uint8_t erased[4];
	h_u32_to_le(erased, ch32v_info->info->erased_word);

	uint8_t *buffer = malloc(ch32v_info->info->page_size);
	if (!buffer)
		return ERROR_FAIL;

	int retval = ERROR_OK;
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		struct flash_sector *sector = &bank->sectors[i];
		retval = target_read_buffer(bank->target, bank->base + sector->offset,
				sector->size, buffer);
		if (retval != ERROR_OK)
			break;
		sector->is_erased = 1;
		for (uint32_t j = 0; j < sector->size; j++) {
			if (buffer[j] != erased[j % 4]) {
				sector->is_erased = 0;
				break;
			}
		}
		keep_alive();
	}
	free(buffer);
	return retval;
}

static int ch32v_probe(struct flash_bank *bank)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;
	struct target *target = bank->target;

	ch32v_info->probed = false;

	/* The CH32V003 has its chip ID at 0x1ffff7c4. There, the CH32V00X reads
	 * 0xffffffff. The CH32V00X, the CH32V20x and the CH32V30x have it at
	 * 0x1ffff704, which is in the boot code of the CH32V003. */
	uint32_t chip_id;
	int retval = target_read_u32(target, CHIP_ID_V003, &chip_id);
	if (retval != ERROR_OK)
		return retval;
	enum ch32v_family family;
	if ((chip_id >> 20) == 0x003) {
		family = CH32V003;
	} else {
		retval = target_read_u32(target, CHIP_ID_V00X_V20X, &chip_id);
		if (retval != ERROR_OK)
			return retval;
		switch (chip_id >> 20) {
		case 0x002:
		case 0x004:
		case 0x005:
		case 0x006:
		case 0x007:
			family = CH32V00X;
			break;
		default:
			if ((chip_id >> 24) == 0x20) {
				family = CH32V20X;
			} else if ((chip_id >> 24) == 0x30) {
				family = CH32V30X;
			} else {
				LOG_ERROR("Unknown chip ID 0x%08" PRIx32, chip_id);
				return ERROR_FLASH_OPERATION_FAILED;
			}
			break;
		}
	}

	uint16_t size_kb;
	retval = target_read_u16(target, ESIG_FLACAP, &size_kb);
	if (retval != ERROR_OK)
		return retval;
	if (size_kb == 0 || size_kb == 0xffff) {
		LOG_ERROR("Flash size register reads 0x%04" PRIx16, size_kb);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	const struct ch32v_family_info *info = &ch32v_families[family];
	const uint32_t flash_kb = info->flash_size_kb ? info->flash_size_kb : size_kb;
	ch32v_info->chip_id = chip_id;
	ch32v_info->info = info;

	free(bank->sectors);
	free(bank->prot_blocks);
	bank->size = flash_kb * 1024;
	bank->num_sectors = bank->size / info->page_size;
	bank->sectors = alloc_block_array(0, info->page_size, bank->num_sectors);
	// FLASH_WPR has 32 bits. The last one covers the rest of a larger flash.
	bank->num_prot_blocks = MIN(DIV_ROUND_UP(bank->size, info->wrp_unit), 32u);
	bank->prot_blocks = alloc_block_array(0, info->wrp_unit, bank->num_prot_blocks);
	if (!bank->sectors || !bank->prot_blocks)
		return ERROR_FAIL;
	struct flash_sector *last = &bank->prot_blocks[bank->num_prot_blocks - 1];
	last->size = bank->size - last->offset;

	bank->write_start_alignment = info->page_size;
	bank->write_end_alignment = info->page_size;

	LOG_INFO("%s, chip ID 0x%08" PRIx32 ", %" PRIu32 " KiB flash",
			info->name, chip_id, flash_kb);
	if (info->flash_size_kb) {
		// SRAM_CODE_MODE, OBR bits 9..7: 00x, 01x, 10x, 110, 111
		static const uint16_t code_kb[] = { 192, 192, 224, 224, 256, 256, 128, 288 };
		uint32_t obr;
		if (target_read_u32(target, FLASH_OBR, &obr) == ERROR_OK)
			LOG_INFO("Zero-wait code area: %" PRIu16 " KiB", code_kb[(obr >> 7) & 7]);
	}
	ch32v_info->probed = true;
	return ERROR_OK;
}

static int ch32v_auto_probe(struct flash_bank *bank)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;

	if (ch32v_info->probed)
		return ERROR_OK;
	return ch32v_probe(bank);
}

static int ch32v_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct ch32v_flash_bank *ch32v_info = bank->driver_priv;

	if (!ch32v_info->probed) {
		command_print_sameline(cmd, "Not probed");
		return ERROR_OK;
	}
	command_print_sameline(cmd, "%s, chip ID 0x%08" PRIx32,
			ch32v_info->info->name, ch32v_info->chip_id);
	return ERROR_OK;
}

static int ch32v_get_probed_bank(struct command_invocation *cmd, struct flash_bank **bank)
{
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32v_check_halted(*bank);
	if (retval != ERROR_OK)
		return retval;
	return ch32v_auto_probe(*bank);
}

COMMAND_HANDLER(ch32v_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = ch32v_get_probed_bank(CMD, &bank);
	if (retval != ERROR_OK)
		return retval;

	retval = ch32v_mass_erase(bank);
	if (retval != ERROR_OK) {
		command_print(CMD, "ch32v mass erase failed");
		return retval;
	}
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;
	command_print(CMD, "ch32v mass erase complete");
	return ERROR_OK;
}

static int ch32v_set_rdpr(struct command_invocation *cmd, uint8_t rdpr)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = ch32v_get_probed_bank(cmd, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint8_t options[OPT_COUNT];
	retval = ch32v_read_options(bank, options);
	if (retval != ERROR_OK)
		return retval;

	options[OPT_RDPR] = rdpr;
	// An unprotect also ends the write protection.
	if (rdpr == OPTION_RDPR_UNPROTECTED)
		memset(&options[OPT_WRPR0], 0xff, 4);
	return ch32v_write_options(bank, options);
}

COMMAND_HANDLER(ch32v_handle_lock_command)
{
	// 0xcc is not used: it is a permanent protection on other WCH chips.
	return ch32v_set_rdpr(CMD, OPTION_RDPR_PROTECTED);
}

COMMAND_HANDLER(ch32v_handle_unlock_command)
{
	return ch32v_set_rdpr(CMD, OPTION_RDPR_UNPROTECTED);
}

COMMAND_HANDLER(ch32v_handle_options_read_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = ch32v_get_probed_bank(CMD, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint32_t obr, wpr;
	uint8_t options[OPT_COUNT];
	retval = target_read_u32(bank->target, FLASH_OBR, &obr);
	if (retval == ERROR_OK)
		retval = target_read_u32(bank->target, FLASH_WPR, &wpr);
	if (retval == ERROR_OK)
		retval = ch32v_read_options(bank, options);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "FLASH_OBR 0x%08" PRIx32 ", FLASH_WPR 0x%08" PRIx32, obr, wpr);
	command_print(CMD, "read protection: %s", (obr & OBR_RDPRT) ? "on" : "off");
	command_print(CMD, "RDPR 0x%02" PRIx8 ", USER 0x%02" PRIx8
			", DATA0 0x%02" PRIx8 ", DATA1 0x%02" PRIx8
			", WRPR 0x%02" PRIx8 " 0x%02" PRIx8 " 0x%02" PRIx8 " 0x%02" PRIx8,
			options[OPT_RDPR], options[OPT_USER],
			options[OPT_DATA0], options[OPT_DATA1],
			options[OPT_WRPR0], options[OPT_WRPR0 + 1],
			options[OPT_WRPR0 + 2], options[OPT_WRPR0 + 3]);
	return ERROR_OK;
}

COMMAND_HANDLER(ch32v_handle_options_write_command)
{
	if (CMD_ARGC != 2 && CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = ch32v_get_probed_bank(CMD, &bank);
	if (retval != ERROR_OK)
		return retval;

	uint8_t options[OPT_COUNT];
	retval = ch32v_read_options(bank, options);
	if (retval != ERROR_OK)
		return retval;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[1], options[OPT_USER]);
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[2], options[OPT_DATA0]);
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[3], options[OPT_DATA1]);
	}
	return ch32v_write_options(bank, options);
}

static const struct command_registration ch32v_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = ch32v_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase the entire flash.",
	},
	{
		.name = "lock",
		.handler = ch32v_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Set the read protection (RDPR 0x00). "
			"Takes effect after a reset.",
	},
	{
		.name = "unlock",
		.handler = ch32v_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Remove the read and write protection (RDPR 0xa5, WRPR 0xff). "
			"A protected chip erases its flash. Takes effect after a reset.",
	},
	{
		.name = "options_read",
		.handler = ch32v_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Show the option bytes.",
	},
	{
		.name = "options_write",
		.handler = ch32v_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id user_byte [data0 data1]",
		.help = "Write the USER option byte and the user data bytes. "
			"Takes effect after a reset.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration ch32v_command_handlers[] = {
	{
		.name = "ch32v",
		.mode = COMMAND_ANY,
		.help = "ch32v flash command group",
		.usage = "",
		.chain = ch32v_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver ch32v_flash = {
	.name = "ch32v",
	.commands = ch32v_command_handlers,
	.flash_bank_command = ch32v_flash_bank_command,
	.erase = ch32v_erase,
	.protect = ch32v_protect,
	.write = ch32v_write,
	.read = default_flash_read,
	.probe = ch32v_probe,
	.auto_probe = ch32v_auto_probe,
	.erase_check = ch32v_erase_check,
	.protect_check = ch32v_protect_check,
	.info = ch32v_get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
