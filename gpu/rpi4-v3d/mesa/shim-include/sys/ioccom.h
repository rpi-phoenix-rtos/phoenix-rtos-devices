/* sys/ioccom.h shim — Linux-style _IOC macros for the DRM UAPI on Phoenix.
 * The encoding must be self-consistent with the winsys backend's _IOC_NR().
 *
 * Copyright 2026 Phoenix Systems  %LICENSE%
 */
#ifndef PHX_SYS_IOCCOM_H
#define PHX_SYS_IOCCOM_H
#define _IOC_NRBITS    8
#define _IOC_TYPEBITS  8
#define _IOC_SIZEBITS  14
#define _IOC_NRSHIFT   0
#define _IOC_TYPESHIFT (_IOC_NRSHIFT+_IOC_NRBITS)
#define _IOC_SIZESHIFT (_IOC_TYPESHIFT+_IOC_TYPEBITS)
#define _IOC_DIRSHIFT  (_IOC_SIZESHIFT+_IOC_SIZEBITS)
#define _IOC_NONE  0U
#define _IOC_WRITE 1U
#define _IOC_READ  2U
#define _IOC(dir,type,nr,size) (((dir)<<_IOC_DIRSHIFT)|((type)<<_IOC_TYPESHIFT)|((nr)<<_IOC_NRSHIFT)|((size)<<_IOC_SIZESHIFT))
#define _IO(type,nr)        _IOC(_IOC_NONE,(type),(nr),0)
#define _IOR(type,nr,sz)    _IOC(_IOC_READ,(type),(nr),sizeof(sz))
#define _IOW(type,nr,sz)    _IOC(_IOC_WRITE,(type),(nr),sizeof(sz))
#define _IOWR(type,nr,sz)   _IOC(_IOC_READ|_IOC_WRITE,(type),(nr),sizeof(sz))
#define _IOC_NR(nr)   (((nr)>>_IOC_NRSHIFT)&((1<<_IOC_NRBITS)-1))
#define _IOC_TYPE(nr) (((nr)>>_IOC_TYPESHIFT)&((1<<_IOC_TYPEBITS)-1))
#endif
