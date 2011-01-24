#ifndef _ASM_NIOS2_TLBSTATS_H_
#define _ASM_NIOS2_TLBSTATS_H_

struct tlb_stat {
      uint32_t local_flush_tlb_mm;
      uint32_t local_flush_tlb_one_pid;
      uint32_t local_flush_tlb_range;
      uint32_t local_flush_tlb_page;
      uint32_t local_flush_tlb_kernel_range;
      uint32_t local_flush_tlb_one;
      uint32_t flush_tlb_pid;
      uint32_t local_flush_tlb_all;
      uint32_t tlb_fast_handler;
      uint32_t tlb_c_handler;
      uint32_t do_page_fault_prot;
      uint32_t do_page_fault;
};

#endif /* _ASM_NIOS2_TLBSTATS_H_ */
