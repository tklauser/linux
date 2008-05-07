#ifndef __NIOS2_UNALIGNED_H
#define __NIOS2_UNALIGNED_H

#include <linux/unaligned/be_byteshift.h>
#include <linux/unaligned/le_byteshift.h>
#include <linux/unaligned/generic.h>

#define get_unaligned	__get_unaligned_le
#define put_unaligned	__put_unaligned_le

#endif				/* __NIOS2_UNALIGNED_H */
