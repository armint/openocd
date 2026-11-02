/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Fast page programming for the flash of the WCH CH32V003, CH32V00X and
 * CH32V20x. The caller unlocks LOCK and FLOCK and sets FTPG in FLASH_CTLR.
 *
 * Built for RV32E without compressed instructions, so that it runs on all
 * three families and ends with a 4-byte ebreak at the end of the code.
 *
 * In:	a0 = FLASH register base (0x40022000)
 *	a1 = number of pages (> 0)
 *	a2 = source buffer
 *	a3 = flash address of the first page
 *	a4 = words per page
 *	a5 = 0: BUFRST/BUFLOAD/STRT method (CH32V003, CH32V00X)
 *	     1: WRBSY/PGSTRT method (CH32V20x)
 * Out:	a0 = 0 on success, else FLASH_STATR of the failed page
 *	a3 = flash address of the failed page (on failure)
 *
 * On the CH32V20x, a word write to a protected page sets WRPRTERR at once,
 * and WRBSY stays set. The loader checks WRPRTERR in that wait.
 */

#define FLASH_STATR	0x0c
#define FLASH_CTLR	0x10
#define FLASH_ADDR	0x14

#define STATR_BSY	(1 << 0)
#define STATR_WRBSY	(1 << 1)
#define STATR_WRPRTERR	(1 << 4)
#define STATR_EOP	(1 << 5)

#define CTLR_STRT	(1 << 6)
#define CTLR_FTPG	(1 << 16)
#define CTLR_BUFLOAD	(1 << 18)
#define CTLR_BUFRST	(1 << 19)
#define CTLR_PGSTRT	(1 << 21)

	.text
	.globl	_start
_start:
page_loop:
	mv	s1, a3
	mv	t2, a4
	bnez	a5, word_loop
	li	t0, CTLR_FTPG | CTLR_BUFRST
	sw	t0, FLASH_CTLR(a0)
	jal	wait_bsy

word_loop:
	lw	t0, 0(a2)
	sw	t0, 0(a3)
	addi	a2, a2, 4
	addi	a3, a3, 4
	bnez	a5, wait_wrbsy
	li	t0, CTLR_FTPG | CTLR_BUFLOAD
	sw	t0, FLASH_CTLR(a0)
	jal	wait_bsy
	j	next_word
wait_wrbsy:
	lw	t0, FLASH_STATR(a0)
	andi	t1, t0, STATR_WRPRTERR
	bnez	t1, failed
	andi	t1, t0, STATR_WRBSY
	bnez	t1, wait_wrbsy
next_word:
	addi	t2, t2, -1
	bnez	t2, word_loop

	bnez	a5, start_pgstrt
	sw	s1, FLASH_ADDR(a0)
	li	t0, CTLR_FTPG | CTLR_STRT
	j	start_page
start_pgstrt:
	li	t0, CTLR_FTPG | CTLR_PGSTRT
start_page:
	sw	t0, FLASH_CTLR(a0)
	jal	wait_bsy

	lw	t0, FLASH_STATR(a0)
	andi	t1, t0, STATR_WRPRTERR
	bnez	t1, failed
	li	t1, STATR_EOP
	sw	t1, FLASH_STATR(a0)

	addi	a1, a1, -1
	bnez	a1, page_loop
	li	a0, 0
	j	done

failed:
	li	t1, STATR_EOP | STATR_WRPRTERR
	sw	t1, FLASH_STATR(a0)
	mv	a0, t0
	mv	a3, s1
	j	done

wait_bsy:
	lw	t0, FLASH_STATR(a0)
	andi	t0, t0, STATR_BSY
	bnez	t0, wait_bsy
	ret

done:
	ebreak
