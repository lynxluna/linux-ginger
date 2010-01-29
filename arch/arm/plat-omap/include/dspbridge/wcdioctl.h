/*
 * wcdioctl.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Contains structures and commands that are used for interaction
 * between the DDSP API and class driver.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef WCDIOCTL_
#define WCDIOCTL_

#include <dspbridge/mem.h>
#include <dspbridge/cmm.h>
#include <dspbridge/strmdefs.h>
#include <dspbridge/dbdcd.h>

union Trapped_Args {

	/* MGR Module */
	struct {
		u32 uNode;
		struct DSP_NDBPROPS __user *pNDBProps;
		u32 uNDBPropsSize;
		u32 __user *puNumNodes;
	} ARGS_MGR_ENUMNODE_INFO;

	struct {
		u32 uProcessor;
		struct DSP_PROCESSORINFO __user *pProcessorInfo;
		u32 uProcessorInfoSize;
		u32 __user *puNumProcs;
	} ARGS_MGR_ENUMPROC_INFO;

	struct {
		struct DSP_UUID *pUuid;
		enum DSP_DCDOBJTYPE objType;
		char *pszPathName;
	} ARGS_MGR_REGISTEROBJECT;

	struct {
		struct DSP_UUID *pUuid;
		enum DSP_DCDOBJTYPE objType;
	} ARGS_MGR_UNREGISTEROBJECT;

	struct {
		struct DSP_NOTIFICATION  __user *__user *aNotifications;
		u32 uCount;
		u32 __user *puIndex;
		u32 uTimeout;
	} ARGS_MGR_WAIT;

	/* PROC Module */
	struct {
		u32 uProcessor;
		struct DSP_PROCESSORATTRIN __user *pAttrIn;
		DSP_HPROCESSOR __user *phProcessor;
	} ARGS_PROC_ATTACH;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 dwCmd;
		struct DSP_CBDATA __user *pArgs;
	} ARGS_PROC_CTRL;

	struct {
		DSP_HPROCESSOR hProcessor;
	} ARGS_PROC_DETACH;

	struct {
		DSP_HPROCESSOR hProcessor;
		DSP_HNODE __user *aNodeTab;
		u32 uNodeTabSize;
		u32 __user *puNumNodes;
		u32 __user *puAllocated;
	} ARGS_PROC_ENUMNODE_INFO;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 uResourceType;
		struct DSP_RESOURCEINFO *pResourceInfo;
		u32 uResourceInfoSize;
	} ARGS_PROC_ENUMRESOURCES;

	struct {
		DSP_HPROCESSOR hProcessor;
		struct DSP_PROCESSORSTATE __user *pProcStatus;
		u32 uStateInfoSize;
	} ARGS_PROC_GETSTATE;

	struct {
		DSP_HPROCESSOR hProcessor;
		u8 __user *pBuf;
		u8 __user *pSize;
		u32 uMaxSize;
	} ARGS_PROC_GETTRACE;

	struct {
		DSP_HPROCESSOR hProcessor;
		s32 iArgc;
		char __user *__user *aArgv;
		char *__user *aEnvp;
	} ARGS_PROC_LOAD;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 uEventMask;
		u32 uNotifyType;
		struct DSP_NOTIFICATION __user *hNotification;
	} ARGS_PROC_REGISTER_NOTIFY;

	struct {
		DSP_HPROCESSOR hProcessor;
	} ARGS_PROC_START;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 ulSize;
		void *__user *ppRsvAddr;
	} ARGS_PROC_RSVMEM;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 ulSize;
		void *pRsvAddr;
	} ARGS_PROC_UNRSVMEM;

	struct {
		DSP_HPROCESSOR hProcessor;
		void *pMpuAddr;
		u32 ulSize;
		void *pReqAddr;
		void *__user *ppMapAddr;
		u32 ulMapAttr;
	} ARGS_PROC_MAPMEM;

	struct {
		DSP_HPROCESSOR hProcessor;
		u32 ulSize;
		void *pMapAddr;
	} ARGS_PROC_UNMAPMEM;

	struct {
		DSP_HPROCESSOR hProcessor;
		void *pMpuAddr;
		u32 ulSize;
		u32 ulFlags;
	} ARGS_PROC_FLUSHMEMORY;

	struct {
		DSP_HPROCESSOR hProcessor;
	} ARGS_PROC_STOP;

	struct {
		DSP_HPROCESSOR hProcessor;
		void *pMpuAddr;
		u32 ulSize;
	} ARGS_PROC_INVALIDATEMEMORY;


	/* NODE Module */
	struct {
		DSP_HPROCESSOR hProcessor;
		struct DSP_UUID __user *pNodeID;
		struct DSP_CBDATA __user *pArgs;
		struct DSP_NODEATTRIN __user *pAttrIn;
		DSP_HNODE __user *phNode;
	} ARGS_NODE_ALLOCATE;

	struct {
		DSP_HNODE hNode;
		u32 uSize;
		struct DSP_BUFFERATTR __user *pAttr;
		u8 *__user *pBuffer;
	} ARGS_NODE_ALLOCMSGBUF;

	struct {
		DSP_HNODE hNode;
		s32 iPriority;
	} ARGS_NODE_CHANGEPRIORITY;

	struct {
		DSP_HNODE hNode;
		u32 uStream;
		DSP_HNODE hOtherNode;
		u32 uOtherStream;
		struct DSP_STRMATTR __user *pAttrs;
		struct DSP_CBDATA __user *pConnParam;
	} ARGS_NODE_CONNECT;

	struct {
		DSP_HNODE hNode;
	} ARGS_NODE_CREATE;

	struct {
		DSP_HNODE hNode;
	} ARGS_NODE_DELETE;

	struct {
		DSP_HNODE hNode;
		struct DSP_BUFFERATTR __user *pAttr;
		u8 *pBuffer;
	} ARGS_NODE_FREEMSGBUF;

	struct {
		DSP_HNODE hNode;
		struct DSP_NODEATTR __user *pAttr;
		u32 uAttrSize;
	} ARGS_NODE_GETATTR;

	struct {
		DSP_HNODE hNode;
		struct DSP_MSG __user *pMessage;
		u32 uTimeout;
	} ARGS_NODE_GETMESSAGE;

	struct {
		DSP_HNODE hNode;
	} ARGS_NODE_PAUSE;

	struct {
		DSP_HNODE hNode;
		struct DSP_MSG __user *pMessage;
		u32 uTimeout;
	} ARGS_NODE_PUTMESSAGE;

	struct {
		DSP_HNODE hNode;
		u32 uEventMask;
		u32 uNotifyType;
		struct DSP_NOTIFICATION __user *hNotification;
	} ARGS_NODE_REGISTERNOTIFY;

	struct {
		DSP_HNODE hNode;
	} ARGS_NODE_RUN;

	struct {
		DSP_HNODE hNode;
		DSP_STATUS __user *pStatus;
	} ARGS_NODE_TERMINATE;

	struct {
		DSP_HPROCESSOR hProcessor;
		struct DSP_UUID __user *pNodeID;
		struct DSP_NDBPROPS __user *pNodeProps;
	} ARGS_NODE_GETUUIDPROPS;

	/* STRM module */

	struct {
		DSP_HSTREAM hStream;
		u32 uSize;
		u8 *__user *apBuffer;
		u32 uNumBufs;
	} ARGS_STRM_ALLOCATEBUFFER;

	struct {
		DSP_HSTREAM hStream;
	} ARGS_STRM_CLOSE;

	struct {
		DSP_HSTREAM hStream;
		u8 *__user *apBuffer;
		u32 uNumBufs;
	} ARGS_STRM_FREEBUFFER;

	struct {
		DSP_HSTREAM hStream;
		HANDLE *phEvent;
	} ARGS_STRM_GETEVENTHANDLE;

	struct {
		DSP_HSTREAM hStream;
		struct STRM_INFO __user *pStreamInfo;
		u32 uStreamInfoSize;
	} ARGS_STRM_GETINFO;

	struct {
		DSP_HSTREAM hStream;
		bool bFlush;
	} ARGS_STRM_IDLE;

	struct {
		DSP_HSTREAM hStream;
		u8 *pBuffer;
		u32 dwBytes;
		u32 dwBufSize;
		u32 dwArg;
	} ARGS_STRM_ISSUE;

	struct {
		DSP_HNODE hNode;
		u32 uDirection;
		u32 uIndex;
		struct STRM_ATTR __user *pAttrIn;
		DSP_HSTREAM __user *phStream;
	} ARGS_STRM_OPEN;

	struct {
		DSP_HSTREAM hStream;
		u8 *__user *pBufPtr;
		u32 __user *pBytes;
		u32 __user *pBufSize;
		u32 __user *pdwArg;
	} ARGS_STRM_RECLAIM;

	struct {
		DSP_HSTREAM hStream;
		u32 uEventMask;
		u32 uNotifyType;
		struct DSP_NOTIFICATION __user *hNotification;
	} ARGS_STRM_REGISTERNOTIFY;

	struct {
		DSP_HSTREAM __user *aStreamTab;
		u32 nStreams;
		u32 __user *pMask;
		u32 uTimeout;
	} ARGS_STRM_SELECT;

	/* CMM Module */
	struct {
		struct CMM_OBJECT *hCmmMgr;
		u32 uSize;
		struct CMM_ATTRS *pAttrs;
		OUT void **ppBufVA;
	} ARGS_CMM_ALLOCBUF;

	struct {
		struct CMM_OBJECT *hCmmMgr;
		void *pBufPA;
		u32 ulSegId;
	} ARGS_CMM_FREEBUF;

	struct {
		DSP_HPROCESSOR hProcessor;
		struct CMM_OBJECT *__user *phCmmMgr;
	} ARGS_CMM_GETHANDLE;

	struct {
		struct CMM_OBJECT *hCmmMgr;
		struct CMM_INFO __user *pCmmInfo;
	} ARGS_CMM_GETINFO;

	/* MEM Module */
	struct {
		u32 cBytes;
		enum MEM_POOLATTRS type;
		void *pMem;
	} ARGS_MEM_ALLOC;

	struct {
		u32 cBytes;
		enum MEM_POOLATTRS type;
		void *pMem;
	} ARGS_MEM_CALLOC;

	struct {
		void *pMem;
	} ARGS_MEM_FREE;

	struct {
		void *pBuffer;
		u32 cSize;
		void *pLockedBuffer;
	} ARGS_MEM_PAGELOCK;

	struct {
		void *pBuffer;
		u32 cSize;
	} ARGS_MEM_PAGEUNLOCK;

	/* UTIL module */
	struct {
		s32 cArgc;
		char **ppArgv;
	} ARGS_UTIL_TESTDLL;
} ;

/*
 * Dspbridge Ioctl numbering scheme
 *
 *    7                           0
 *  ---------------------------------
 *  |  Module   |   Ioctl Number    |
 *  ---------------------------------
 *  | x | x | x | 0 | 0 | 0 | 0 | 0 |
 *  ---------------------------------
 */

/* Ioctl driver identifier */
#define DB		0xDB

/*
 * Following are used to distinguish between module ioctls, this is needed
 * in case new ioctls are introduced.
 */
#define DB_MODULE_MASK		0xE0
#define DB_IOC_MASK		0x1F

/* Ioctl module masks */
#define DB_MGR		0x0
#define DB_PROC		0x20
#define DB_NODE		0x40
#define DB_STRM		0x60
#define DB_CMM		0x80

#define DB_MODULE_SHIFT		5

/* Used to calculate the ioctl per dspbridge module */
#define DB_IOC(module, num) \
			(((module) & DB_MODULE_MASK) | ((num) & DB_IOC_MASK))
/* Used to get dspbridge ioctl module */
#define DB_GET_MODULE(cmd)	((cmd) & DB_MODULE_MASK)
/* Used to get dspbridge ioctl number */
#define DB_GET_IOC(cmd)		((cmd) & DB_IOC_MASK)

/* TODO: Remove deprecated and not implemented */

/* MGR Module */
#define MGR_ENUMNODE_INFO	_IOWR(DB, DB_IOC(DB_MGR, 0), unsigned long)
#define MGR_ENUMPROC_INFO	_IOWR(DB, DB_IOC(DB_MGR, 1), unsigned long)
#define MGR_REGISTEROBJECT	_IOWR(DB, DB_IOC(DB_MGR, 2), unsigned long)
#define MGR_UNREGISTEROBJECT	_IOWR(DB, DB_IOC(DB_MGR, 3), unsigned long)
#define MGR_WAIT		_IOWR(DB, DB_IOC(DB_MGR, 4), unsigned long)
/* MGR_GET_PROC_RES Deprecated */
#define MGR_GET_PROC_RES	_IOR(DB, DB_IOC(DB_MGR, 5), unsigned long)

/* PROC Module */
#define PROC_ATTACH		_IOWR(DB, DB_IOC(DB_PROC, 0), unsigned long)
#define PROC_CTRL		_IOR(DB, DB_IOC(DB_PROC, 1), unsigned long)
/* PROC_DETACH Deprecated */
#define PROC_DETACH		_IOR(DB, DB_IOC(DB_PROC, 2), unsigned long)
#define PROC_ENUMNODE		_IOWR(DB, DB_IOC(DB_PROC, 3), unsigned long)
#define PROC_ENUMRESOURCES	_IOWR(DB, DB_IOC(DB_PROC, 4), unsigned long)
#define PROC_GET_STATE		_IOWR(DB, DB_IOC(DB_PROC, 5), unsigned long)
#define PROC_GET_TRACE		_IOWR(DB, DB_IOC(DB_PROC, 6), unsigned long)
#define PROC_LOAD		_IOW(DB, DB_IOC(DB_PROC, 7), unsigned long)
#define PROC_REGISTERNOTIFY	_IOWR(DB, DB_IOC(DB_PROC, 8), unsigned long)
#define PROC_START		_IOW(DB, DB_IOC(DB_PROC, 9), unsigned long)
#define PROC_RSVMEM		_IOWR(DB, DB_IOC(DB_PROC, 10), unsigned long)
#define PROC_UNRSVMEM		_IOW(DB, DB_IOC(DB_PROC, 11), unsigned long)
#define PROC_MAPMEM		_IOWR(DB, DB_IOC(DB_PROC, 12), unsigned long)
#define PROC_UNMAPMEM		_IOR(DB, DB_IOC(DB_PROC, 13), unsigned long)
#define PROC_FLUSHMEMORY	_IOW(DB, DB_IOC(DB_PROC, 14), unsigned long)
#define PROC_STOP		_IOWR(DB, DB_IOC(DB_PROC, 15), unsigned long)
#define PROC_INVALIDATEMEMORY	_IOW(DB, DB_IOC(DB_PROC, 16), unsigned long)

/* NODE Module */
#define NODE_ALLOCATE		_IOWR(DB, DB_IOC(DB_NODE, 0), unsigned long)
#define NODE_ALLOCMSGBUF	_IOWR(DB, DB_IOC(DB_NODE, 1), unsigned long)
#define NODE_CHANGEPRIORITY	_IOW(DB, DB_IOC(DB_NODE, 2), unsigned long)
#define NODE_CONNECT		_IOW(DB, DB_IOC(DB_NODE, 3), unsigned long)
#define NODE_CREATE		_IOW(DB, DB_IOC(DB_NODE, 4), unsigned long)
#define NODE_DELETE		_IOW(DB, DB_IOC(DB_NODE, 5), unsigned long)
#define NODE_FREEMSGBUF		_IOW(DB, DB_IOC(DB_NODE, 6), unsigned long)
#define NODE_GETATTR		_IOWR(DB, DB_IOC(DB_NODE, 7), unsigned long)
#define NODE_GETMESSAGE		_IOWR(DB, DB_IOC(DB_NODE, 8), unsigned long)
#define NODE_PAUSE		_IOW(DB, DB_IOC(DB_NODE, 9), unsigned long)
#define NODE_PUTMESSAGE		_IOW(DB, DB_IOC(DB_NODE, 10), unsigned long)
#define NODE_REGISTERNOTIFY	_IOWR(DB, DB_IOC(DB_NODE, 11), unsigned long)
#define NODE_RUN		_IOW(DB, DB_IOC(DB_NODE, 12), unsigned long)
#define NODE_TERMINATE		_IOWR(DB, DB_IOC(DB_NODE, 13), unsigned long)
#define NODE_GETUUIDPROPS	_IOWR(DB, DB_IOC(DB_NODE, 14), unsigned long)

/* STRM Module */
#define STRM_ALLOCATEBUFFER	_IOWR(DB, DB_IOC(DB_STRM, 0), unsigned long)
#define STRM_CLOSE		_IOW(DB, DB_IOC(DB_STRM, 1), unsigned long)
#define STRM_FREEBUFFER		_IOWR(DB, DB_IOC(DB_STRM, 2), unsigned long)
#define STRM_GETEVENTHANDLE	_IO(DB, DB_IOC(DB_STRM, 3))	/* Not Impl'd */
#define STRM_GETINFO		_IOWR(DB, DB_IOC(DB_STRM, 4), unsigned long)
#define STRM_IDLE		_IOW(DB, DB_IOC(DB_STRM, 5), unsigned long)
#define STRM_ISSUE		_IOW(DB, DB_IOC(DB_STRM, 6), unsigned long)
#define STRM_OPEN		_IOWR(DB, DB_IOC(DB_STRM, 7), unsigned long)
#define STRM_RECLAIM		_IOWR(DB, DB_IOC(DB_STRM, 8), unsigned long)
#define STRM_REGISTERNOTIFY	_IOWR(DB, DB_IOC(DB_STRM, 9), unsigned long)
#define STRM_SELECT		_IOWR(DB, DB_IOC(DB_STRM, 10), unsigned long)

/* CMM Module */
#define CMM_ALLOCBUF		_IO(DB, DB_IOC(DB_CMM, 0))	/* Not Impl'd */
#define CMM_FREEBUF		_IO(DB, DB_IOC(DB_CMM, 1))	/* Not Impl'd */
#define CMM_GETHANDLE		_IOR(DB, DB_IOC(DB_CMM, 2), unsigned long)
#define CMM_GETINFO		_IOR(DB, DB_IOC(DB_CMM, 3), unsigned long)

#endif				/* WCDIOCTL_ */
