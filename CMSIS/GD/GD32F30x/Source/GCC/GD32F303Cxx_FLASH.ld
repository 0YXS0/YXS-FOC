/* 入口点 */
ENTRY(Reset_Handler)

/* 用户模式堆栈的最高地址 */
_estack = ORIGIN(RAM) + LENGTH(RAM);    /* RAM的末尾，堆栈的起始点 */
/* 如果堆和栈不适合RAM，则生成链接错误 */
_Min_Heap_Size = 0x200;                  /* 堆所需的最小大小 */
_Min_Stack_Size = 0x400;                 /* 栈所需的最小大小 */

/* 指定内存区域 */
MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 48K /* 随机访问存储器，可读可写 */
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 256K /* 闪存，只读 */
}

/* 定义输出段 */
SECTIONS
{
  /* 中断向量表放置在FLASH的最开始位置 */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector)) /* 保留中断向量表 */
    . = ALIGN(4);
  } >FLASH

  /* 程序代码放置在FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* 程序代码 */
    *(.text*)          /* 所有其他文本相关的段 */
    *(.glue_7)         /* ARM到Thumb的胶水代码 */
    *(.glue_7t)        /* Thumb到ARM的胶水代码 */
    *(.eh_frame)       /* C++异常处理信息 */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* 程序代码结束的全局符号 */
  } >FLASH

  /* 只读数据放置在FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* 只读数据段 */
    *(.rodata*)        /* 所有其他只读数据段 */
    . = ALIGN(4);
  } >FLASH

  /* ARM异常处理表 */
  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* 初始化之前的数组 */
  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  /* 初始化时执行的数组 */
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  /* 清理时执行的数组 */
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  /* 数据段的加载地址，用于初始化RAM中的.data段 */
  _sidata = LOADADDR(.data);

  /* 初始化的数据段放置在RAM */
  .data : 
  {
    . = ALIGN(4);
    _sdata = .;        /* 初始化数据段开始的全局符号 */
    *(.data)           /* 初始化数据段 */
    *(.data*)          /* 所有其他初始化数据段 */

    . = ALIGN(4);
    _edata = .;        /* 初始化数据段结束的全局符号 */
  } >RAM AT> FLASH

  /* 未初始化的数据段（BSS段）放置在RAM */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;         /* 未初始化数据段开始的全局符号 */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* 未初始化数据段结束的全局符号 */
    __bss_end__ = _ebss;
  } >RAM

  /* 用户堆栈段，用于检查RAM剩余空间 */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size; /* 堆空间 */
    . = . + _Min_Stack_Size; /* 堆栈空间 */
    . = ALIGN(8);
  } >RAM

  /* 从标准库中排除不需要的段 */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  /* ARM编译器属性段 */
  .ARM.attributes 0 : { *(.ARM.attributes) }
}