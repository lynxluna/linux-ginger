/*
 * wcd.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Common WCD functions, also includes the wrapper
 * functions called directly by the DeviceIOControl interface.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/mem.h>
#include <dspbridge/ntfy.h>
#include <dspbridge/services.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/chnl.h>
#include <dspbridge/dev.h>
#include <dspbridge/drv.h>

#include <dspbridge/proc.h>
#include <dspbridge/strm.h>

/*  ----------------------------------- Resource Manager */
#include <dspbridge/disp.h>
#include <dspbridge/mgr.h>
#include <dspbridge/node.h>
#include <dspbridge/rmm.h>


/*  ----------------------------------- Others */
#include <dspbridge/msg.h>
#include <dspbridge/cmm.h>
#include <dspbridge/io.h>

/*  ----------------------------------- This */
#include <dspbridge/_dcd.h>
#include <dspbridge/dbdcd.h>

#include <dspbridge/resourcecleanup.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define MAX_TRACEBUFLEN 255
#define MAX_LOADARGS    16
#define MAX_NODES       64
#define MAX_STREAMS     16
#define MAX_BUFS	64

/* Used to get dspbridge ioctl table */
#define DB_GET_IOC_TABLE(cmd)	(DB_GET_MODULE(cmd) >> DB_MODULE_SHIFT)

/* Device IOCtl function pointer */
struct WCD_Cmd {
	u32(*fxn)(union Trapped_Args *args, void *pr_ctxt);
	u32 dwIndex;
} ;

/*  ----------------------------------- Globals */
static u32 WCD_cRefs;

/*
 *  Function tables.
 *  The order of these functions MUST be the same as the order of the command
 *  numbers defined in wcdioctl.h  This is how an IOCTL number in user mode
 *  turns into a function call in kernel mode.
 */

/* MGR wrapper functions */
static struct WCD_Cmd mgr_cmd[] = {
	{MGRWRAP_EnumNode_Info},		/* MGR_ENUMNODE_INFO */
	{MGRWRAP_EnumProc_Info},		/* MGR_ENUMPROC_INFO */
	{MGRWRAP_RegisterObject},		/* MGR_REGISTEROBJECT */
	{MGRWRAP_UnregisterObject},		/* MGR_UNREGISTEROBJECT */
	{MGRWRAP_WaitForBridgeEvents},		/* MGR_WAIT */
	{MGRWRAP_GetProcessResourcesInfo},	/* MGR_GET_PROC_RES */
};

/* PROC wrapper functions */
static struct WCD_Cmd proc_cmd[] = {
	{PROCWRAP_Attach},			/* PROC_ATTACH */
	{PROCWRAP_Ctrl},			/* PROC_CTRL */
	{PROCWRAP_Detach},			/* PROC_DETACH */
	{PROCWRAP_EnumNode_Info},		/* PROC_ENUMNODE */
	{PROCWRAP_EnumResources},		/* PROC_ENUMRESOURCES */
	{PROCWRAP_GetState},			/* PROC_GET_STATE */
	{PROCWRAP_GetTrace},			/* PROC_GET_TRACE */
	{PROCWRAP_Load},			/* PROC_LOAD */
	{PROCWRAP_RegisterNotify},		/* PROC_REGISTERNOTIFY */
	{PROCWRAP_Start},			/* PROC_START */
	{PROCWRAP_ReserveMemory},		/* PROC_RSVMEM */
	{PROCWRAP_UnReserveMemory},		/* PROC_UNRSVMEM */
	{PROCWRAP_Map},				/* PROC_MAPMEM */
	{PROCWRAP_UnMap},			/* PROC_UNMAPMEM */
	{PROCWRAP_FlushMemory},			/* PROC_FLUSHMEMORY */
	{PROCWRAP_Stop},			/* PROC_STOP */
	{PROCWRAP_InvalidateMemory},		/* PROC_INVALIDATEMEMORY */
};

/* NODE wrapper functions */
static struct WCD_Cmd node_cmd[] = {
	{NODEWRAP_Allocate},			/* NODE_ALLOCATE */
	{NODEWRAP_AllocMsgBuf},			/* NODE_ALLOCMSGBUF */
	{NODEWRAP_ChangePriority},		/* NODE_CHANGEPRIORITY */
	{NODEWRAP_Connect},			/* NODE_CONNECT */
	{NODEWRAP_Create},			/* NODE_CREATE */
	{NODEWRAP_Delete},			/* NODE_DELETE */
	{NODEWRAP_FreeMsgBuf},			/* NODE_FREEMSGBUF */
	{NODEWRAP_GetAttr},			/* NODE_GETATTR */
	{NODEWRAP_GetMessage},			/* NODE_GETMESSAGE */
	{NODEWRAP_Pause},			/* NODE_PAUSE */
	{NODEWRAP_PutMessage},			/* NODE_PUTMESSAGE */
	{NODEWRAP_RegisterNotify},		/* NODE_REGISTERNOTIFY */
	{NODEWRAP_Run},				/* NODE_RUN */
	{NODEWRAP_Terminate},			/* NODE_TERMINATE */
	{NODEWRAP_GetUUIDProps},		/* NODE_GETUUIDPROPS */
};

/* STRM wrapper functions */
static struct WCD_Cmd strm_cmd[] = {
	{STRMWRAP_AllocateBuffer},		/* STRM_ALLOCATEBUFFER */
	{STRMWRAP_Close},			/* STRM_CLOSE */
	{STRMWRAP_FreeBuffer},			/* STRM_FREEBUFFER */
	{STRMWRAP_GetEventHandle},		/* STRM_GETEVENTHANDLE */
	{STRMWRAP_GetInfo},			/* STRM_GETINFO */
	{STRMWRAP_Idle},			/* STRM_IDLE */
	{STRMWRAP_Issue},			/* STRM_ISSUE */
	{STRMWRAP_Open},			/* STRM_OPEN */
	{STRMWRAP_Reclaim},			/* STRM_RECLAIM */
	{STRMWRAP_RegisterNotify},		/* STRM_REGISTERNOTIFY */
	{STRMWRAP_Select},			/* STRM_SELECT */
};

/* CMM wrapper functions */
static struct WCD_Cmd cmm_cmd[] = {
	{CMMWRAP_CallocBuf},			/* CMM_ALLOCBUF */
	{CMMWRAP_FreeBuf},			/* CMM_FREEBUF */
	{CMMWRAP_GetHandle},			/* CMM_GETHANDLE */
	{CMMWRAP_GetInfo},			/* CMM_GETINFO */
};

/* Array used to store ioctl table sizes. It can hold up to 8 entries */
static u8 size_cmd[] = {
	ARRAY_SIZE(mgr_cmd),
	ARRAY_SIZE(proc_cmd),
	ARRAY_SIZE(node_cmd),
	ARRAY_SIZE(strm_cmd),
	ARRAY_SIZE(cmm_cmd),
};

static inline void __cp_fm_usr(void *to, const void __user *from,
			       DSP_STATUS *err, unsigned long bytes)
{
	if (DSP_FAILED(*err))
		return;

	if (unlikely(!from)) {
		*err = DSP_EPOINTER;
		return;
	}

	if (unlikely(copy_from_user(to, from, bytes)))
		*err = DSP_EPOINTER;
}
#define cp_fm_usr(to, from, err, n)				\
	__cp_fm_usr(to, from, &(err), (n) * sizeof(*(to)))

static inline void __cp_to_usr(void __user *to, const void *from,
			       DSP_STATUS *err, unsigned long bytes)
{
	if (DSP_FAILED(*err))
		return;

	if (unlikely(!to)) {
		*err = DSP_EPOINTER;
		return;
	}

	if (unlikely(copy_to_user(to, from, bytes)))
		*err = DSP_EPOINTER;
}
#define cp_to_usr(to, from, err, n)				\
	__cp_to_usr(to, from, &(err), (n) * sizeof(*(from)))

/*
 *  ======== WCD_CallDevIOCtl ========
 *  Purpose:
 *      Call the (wrapper) function for the corresponding WCD IOCTL.
 */
inline DSP_STATUS WCD_CallDevIOCtl(u32 cmd, union Trapped_Args *args,
				    u32 *result, void *pr_ctxt)
{
	u32 (*ioctl_cmd)(union Trapped_Args *args, void *pr_ctxt) = NULL;
	int i;

	if (_IOC_TYPE(cmd) != DB) {
		pr_err("%s: Incompatible dspbridge ioctl number\n", __func__);
		goto err;
	}

	if (DB_GET_IOC_TABLE(cmd) > ARRAY_SIZE(size_cmd)) {
		pr_err("%s: undefined ioctl module\n", __func__);
		goto err;
	}

	/* Check the size of the required cmd table */
	i = DB_GET_IOC(cmd);
	if (i > size_cmd[DB_GET_IOC_TABLE(cmd)]) {
		pr_err("%s: requested ioctl %d out of bounds for table %d\n",
					__func__, i, DB_GET_IOC_TABLE(cmd));
		goto err;
	}

	switch (DB_GET_MODULE(cmd)) {
	case DB_MGR:
		ioctl_cmd = mgr_cmd[i].fxn;
		break;
	case DB_PROC:
		ioctl_cmd = proc_cmd[i].fxn;
		break;
	case DB_NODE:
		ioctl_cmd = node_cmd[i].fxn;
		break;
	case DB_STRM:
		ioctl_cmd = strm_cmd[i].fxn;
		break;
	case DB_CMM:
		ioctl_cmd = cmm_cmd[i].fxn;
		break;
	}

	if (!ioctl_cmd) {
		pr_err("%s: requested ioctl not defined\n", __func__);
		goto err;
	} else {
		*result = (*ioctl_cmd)(args, pr_ctxt);
	}

	return DSP_SOK;

err:
	return -EINVAL;
}

/*
 *  ======== WCD_Exit ========
 */
void WCD_Exit(void)
{
	DBC_Require(WCD_cRefs > 0);
	WCD_cRefs--;

	if (WCD_cRefs == 0) {
		/* Release all WCD modules initialized in WCD_Init(). */
		COD_Exit();
		DEV_Exit();
		CHNL_Exit();
		MSG_Exit();
		IO_Exit();
		STRM_Exit();
		DISP_Exit();
		NODE_Exit();
		PROC_Exit();
		MGR_Exit();
		RMM_exit();
		DRV_Exit();
	}
	DBC_Ensure(WCD_cRefs >= 0);
}

/*
 *  ======== WCD_Init ========
 *  Purpose:
 *      Module initialization is done by SERVICES Init.
 */
bool WCD_Init(void)
{
	bool fInit = true;
	bool fDRV, fDEV, fCOD, fCHNL, fMSG, fIO;
	bool fMGR, fPROC, fNODE, fDISP, fSTRM, fRMM;

	if (WCD_cRefs == 0) {
		/* initialize class driver and other modules */
		fDRV = DRV_Init();
		fMGR = MGR_Init();
		fPROC = PROC_Init();
		fNODE = NODE_Init();
		fDISP = DISP_Init();
		fSTRM = STRM_Init();
		fRMM = RMM_init();
		fCHNL = CHNL_Init();
		fMSG = MSG_Init();
		fIO = IO_Init();
		fDEV = DEV_Init();
		fCOD = COD_Init();
		fInit = fDRV && fDEV && fCHNL && fCOD &&
			fMSG && fIO;
		fInit = fInit && fMGR && fPROC && fRMM;
		if (!fInit) {
			if (fDRV)
				DRV_Exit();

			if (fMGR)
				MGR_Exit();

			if (fSTRM)
				STRM_Exit();

			if (fPROC)
				PROC_Exit();

			if (fNODE)
				NODE_Exit();

			if (fDISP)
				DISP_Exit();

			if (fCHNL)
				CHNL_Exit();

			if (fMSG)
				MSG_Exit();

			if (fIO)
				IO_Exit();

			if (fDEV)
				DEV_Exit();

			if (fCOD)
				COD_Exit();

			if (fRMM)
				RMM_exit();

		}
	}
	if (fInit)
		WCD_cRefs++;

	return fInit;
}

/*
 *  ======== WCD_InitComplete2 ========
 *  Purpose:
 *      Perform any required WCD, and WMD initialization which
 *      cannot not be performed in WCD_Init() or DEV_StartDevice() due
 *      to the fact that some services are not yet
 *      completely initialized.
 *  Parameters:
 *  Returns:
 *      DSP_SOK:	Allow this device to load
 *      DSP_EFAIL:      Failure.
 *  Requires:
 *      WCD initialized.
 *  Ensures:
 */
DSP_STATUS WCD_InitComplete2(void)
{
	DSP_STATUS status = DSP_SOK;
	struct CFG_DEVNODE *DevNode;
	struct DEV_OBJECT *hDevObject;
	u32 devType;

	DBC_Require(WCD_cRefs > 0);

	 /*  Walk the list of DevObjects, get each devnode, and attempting to
	 *  autostart the board. Note that this requires COF loading, which
	 *  requires KFILE.  */
	for (hDevObject = DEV_GetFirst(); hDevObject != NULL;
	     hDevObject = DEV_GetNext(hDevObject)) {
		if (DSP_FAILED(DEV_GetDevNode(hDevObject, &DevNode)))
			continue;

		if (DSP_FAILED(DEV_GetDevType(hDevObject, &devType)))
			continue;

		if ((devType == DSP_UNIT) || (devType == IVA_UNIT)) {
			if (DSP_FAILED(PROC_AutoStart(DevNode, hDevObject))) {
				status = DSP_EFAIL;
				/* break; */
			}
		}
	}			/* End For Loop */

	return status;
}

/* TODO: Remove deprecated and not implemented ioctl wrappers */

/*
 * ======== MGRWRAP_EnumNode_Info ========
 */
u32 MGRWRAP_EnumNode_Info(union Trapped_Args *args, void *pr_ctxt)
{
	u8 *pNDBProps;
	u32 uNumNodes;
	DSP_STATUS status = DSP_SOK;
	u32 size = args->ARGS_MGR_ENUMNODE_INFO.uNDBPropsSize;

	if (size < sizeof(struct DSP_NDBPROPS))
		return DSP_ESIZE;

	pNDBProps = MEM_Alloc(size, MEM_NONPAGED);
	if (pNDBProps == NULL)
		status = DSP_EMEMORY;

	if (DSP_SUCCEEDED(status)) {
		status = MGR_EnumNodeInfo(args->ARGS_MGR_ENUMNODE_INFO.uNode,
					 (struct DSP_NDBPROPS *)pNDBProps,
					 size, &uNumNodes);
	}
	cp_to_usr(args->ARGS_MGR_ENUMNODE_INFO.pNDBProps, pNDBProps, status,
		 size);
	cp_to_usr(args->ARGS_MGR_ENUMNODE_INFO.puNumNodes, &uNumNodes, status,
		 1);
	kfree(pNDBProps);

	return status;
}

/*
 * ======== MGRWRAP_EnumProc_Info ========
 */
u32 MGRWRAP_EnumProc_Info(union Trapped_Args *args, void *pr_ctxt)
{
	u8 *pProcessorInfo;
	u32 uNumProcs;
	DSP_STATUS status = DSP_SOK;
	u32 size = args->ARGS_MGR_ENUMPROC_INFO.uProcessorInfoSize;

	if (size < sizeof(struct DSP_PROCESSORINFO))
		return DSP_ESIZE;

	pProcessorInfo = MEM_Alloc(size, MEM_NONPAGED);
	if (pProcessorInfo == NULL)
		status = DSP_EMEMORY;

	if (DSP_SUCCEEDED(status)) {
		status = MGR_EnumProcessorInfo(args->
				ARGS_MGR_ENUMPROC_INFO.uProcessor,
				(struct DSP_PROCESSORINFO *)pProcessorInfo,
				size, &uNumProcs);
	}
	cp_to_usr(args->ARGS_MGR_ENUMPROC_INFO.pProcessorInfo, pProcessorInfo,
		 status, size);
	cp_to_usr(args->ARGS_MGR_ENUMPROC_INFO.puNumProcs, &uNumProcs,
		 status, 1);
	kfree(pProcessorInfo);

	return status;
}

#define WRAP_MAP2CALLER(x) x
/*
 * ======== MGRWRAP_RegisterObject ========
 */
u32 MGRWRAP_RegisterObject(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;
	struct DSP_UUID pUuid;
	u32 pathSize = 0;
	char *pszPathName = NULL;
	DSP_STATUS status = DSP_SOK;

	cp_fm_usr(&pUuid, args->ARGS_MGR_REGISTEROBJECT.pUuid, status, 1);
	if (DSP_FAILED(status))
		goto func_end;
	/* pathSize is increased by 1 to accommodate NULL */
	pathSize = strlen_user((char *)
			args->ARGS_MGR_REGISTEROBJECT.pszPathName) + 1;
	pszPathName = MEM_Alloc(pathSize, MEM_NONPAGED);
	if (!pszPathName)
		goto func_end;
	retVal = strncpy_from_user(pszPathName,
			(char *)args->ARGS_MGR_REGISTEROBJECT.pszPathName,
			pathSize);
	if (!retVal) {
		status = DSP_EPOINTER;
		goto func_end;
	}

	if (args->ARGS_MGR_REGISTEROBJECT.objType >= DSP_DCDMAXOBJTYPE)
		return DSP_EINVALIDARG;

	status = DCD_RegisterObject(&pUuid,
				args->ARGS_MGR_REGISTEROBJECT.objType,
				(char *)pszPathName);
func_end:
	kfree(pszPathName);
	return status;
}

/*
 * ======== MGRWRAP_UnregisterObject ========
 */
u32 MGRWRAP_UnregisterObject(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_UUID pUuid;

	cp_fm_usr(&pUuid, args->ARGS_MGR_REGISTEROBJECT.pUuid, status, 1);
	if (DSP_FAILED(status))
		goto func_end;

	status = DCD_UnregisterObject(&pUuid,
			args->ARGS_MGR_UNREGISTEROBJECT.objType);
func_end:
	return status;

}

/*
 * ======== MGRWRAP_WaitForBridgeEvents ========
 */
u32 MGRWRAP_WaitForBridgeEvents(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK, real_status = DSP_SOK;
	struct DSP_NOTIFICATION *aNotifications[MAX_EVENTS];
	struct DSP_NOTIFICATION notifications[MAX_EVENTS];
	u32 uIndex, i;
	u32 uCount = args->ARGS_MGR_WAIT.uCount;

	if (uCount > MAX_EVENTS)
		status = DSP_EINVALIDARG;

	/* get the array of pointers to user structures */
	cp_fm_usr(aNotifications, args->ARGS_MGR_WAIT.aNotifications,
	 status, uCount);
	/* get the events */
	for (i = 0; i < uCount; i++) {
		cp_fm_usr(&notifications[i], aNotifications[i], status, 1);
		if (DSP_SUCCEEDED(status)) {
			/* set the array of pointers to kernel structures*/
			aNotifications[i] = &notifications[i];
		}
	}
	if (DSP_SUCCEEDED(status)) {
		real_status = MGR_WaitForBridgeEvents(aNotifications, uCount,
			 &uIndex, args->ARGS_MGR_WAIT.uTimeout);
	}
	cp_to_usr(args->ARGS_MGR_WAIT.puIndex, &uIndex, status, 1);
	return real_status;
}


/*
 * ======== MGRWRAP_GetProcessResourceInfo ========
 */
u32 __deprecated MGRWRAP_GetProcessResourcesInfo(union Trapped_Args *args,
						void *pr_ctxt)
{
	pr_err("%s: deprecated dspbridge ioctl\n", __func__);
	return DSP_SOK;
}


/*
 * ======== PROCWRAP_Attach ========
 */
u32 PROCWRAP_Attach(union Trapped_Args *args, void *pr_ctxt)
{
	void *processor;
	DSP_STATUS status = DSP_SOK;
	struct DSP_PROCESSORATTRIN attrIn, *pAttrIn = NULL;

	/* Optional argument */
	if (args->ARGS_PROC_ATTACH.pAttrIn) {
		cp_fm_usr(&attrIn, args->ARGS_PROC_ATTACH.pAttrIn, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrIn = &attrIn;
		else
			goto func_end;


	}
	status = PROC_Attach(args->ARGS_PROC_ATTACH.uProcessor, pAttrIn,
			    &processor, pr_ctxt);
	cp_to_usr(args->ARGS_PROC_ATTACH.phProcessor, &processor, status, 1);
func_end:
	return status;
}

/*
 * ======== PROCWRAP_Ctrl ========
 */
u32 PROCWRAP_Ctrl(union Trapped_Args *args, void *pr_ctxt)
{
	u32 cbDataSize, __user *pSize = (u32 __user *)
			args->ARGS_PROC_CTRL.pArgs;
	u8 *pArgs = NULL;
	DSP_STATUS status = DSP_SOK;

	if (pSize) {
		if (get_user(cbDataSize, pSize)) {
			status = DSP_EFAIL;
			goto func_end;
		}
		cbDataSize += sizeof(u32);
		pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
		if (pArgs == NULL) {
			status = DSP_EMEMORY;
			goto func_end;
		}

		cp_fm_usr(pArgs, args->ARGS_PROC_CTRL.pArgs, status,
			 cbDataSize);
	}
	if (DSP_SUCCEEDED(status)) {
		status = PROC_Ctrl(args->ARGS_PROC_CTRL.hProcessor,
				  args->ARGS_PROC_CTRL.dwCmd,
				  (struct DSP_CBDATA *)pArgs);
	}

	/* cp_to_usr(args->ARGS_PROC_CTRL.pArgs, pArgs, status, 1);*/
	kfree(pArgs);
func_end:
	return status;
}

/*
 * ======== PROCWRAP_Detach ========
 */
u32 __deprecated PROCWRAP_Detach(union Trapped_Args *args, void *pr_ctxt)
{
	/* PROC_Detach called at bridge_release only */
	pr_err("%s: deprecated dspbridge ioctl\n", __func__);
	return DSP_SOK;
}

/*
 * ======== PROCWRAP_EnumNode_Info ========
 */
u32 PROCWRAP_EnumNode_Info(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	void *aNodeTab[MAX_NODES];
	u32 uNumNodes;
	u32 uAllocated;

	if (!args->ARGS_PROC_ENUMNODE_INFO.uNodeTabSize)
		return DSP_ESIZE;

	status = PROC_EnumNodes(args->ARGS_PROC_ENUMNODE_INFO.hProcessor,
				aNodeTab,
				args->ARGS_PROC_ENUMNODE_INFO.uNodeTabSize,
				&uNumNodes, &uAllocated);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.aNodeTab, aNodeTab, status,
		 uNumNodes);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.puNumNodes, &uNumNodes,
		 status, 1);
	cp_to_usr(args->ARGS_PROC_ENUMNODE_INFO.puAllocated, &uAllocated,
		 status, 1);
	return status;
}

/*
 * ======== PROCWRAP_FlushMemory ========
 */
u32 PROCWRAP_FlushMemory(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;

	if (args->ARGS_PROC_FLUSHMEMORY.ulFlags >
					 PROC_WRBK_INV_ALL)
		return DSP_EINVALIDARG;

	status = PROC_FlushMemory(args->ARGS_PROC_FLUSHMEMORY.hProcessor,
				 args->ARGS_PROC_FLUSHMEMORY.pMpuAddr,
				 args->ARGS_PROC_FLUSHMEMORY.ulSize,
				 args->ARGS_PROC_FLUSHMEMORY.ulFlags);
	return status;
}


/*
 * ======== PROCWRAP_InvalidateMemory ========
 */
u32 PROCWRAP_InvalidateMemory(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;

	status = PROC_InvalidateMemory(
				  args->ARGS_PROC_INVALIDATEMEMORY.hProcessor,
				  args->ARGS_PROC_INVALIDATEMEMORY.pMpuAddr,
				  args->ARGS_PROC_INVALIDATEMEMORY.ulSize);
	return status;
}


/*
 * ======== PROCWRAP_EnumResources ========
 */
u32 PROCWRAP_EnumResources(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_RESOURCEINFO pResourceInfo;

	if (args->ARGS_PROC_ENUMRESOURCES.uResourceInfoSize <
		sizeof(struct DSP_RESOURCEINFO))
		return DSP_ESIZE;

	status = PROC_GetResourceInfo(args->ARGS_PROC_ENUMRESOURCES.hProcessor,
			args->ARGS_PROC_ENUMRESOURCES.uResourceType,
			&pResourceInfo,
			args->ARGS_PROC_ENUMRESOURCES.uResourceInfoSize);

	cp_to_usr(args->ARGS_PROC_ENUMRESOURCES.pResourceInfo, &pResourceInfo,
						status, 1);

	return status;

}

/*
 * ======== PROCWRAP_GetState ========
 */
u32 PROCWRAP_GetState(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	struct DSP_PROCESSORSTATE procStatus;

	if (args->ARGS_PROC_GETSTATE.uStateInfoSize <
		sizeof(struct DSP_PROCESSORSTATE))
		return DSP_ESIZE;

	status = PROC_GetState(args->ARGS_PROC_GETSTATE.hProcessor, &procStatus,
			      args->ARGS_PROC_GETSTATE.uStateInfoSize);
	cp_to_usr(args->ARGS_PROC_GETSTATE.pProcStatus, &procStatus, status, 1);
	return status;

}

/*
 * ======== PROCWRAP_GetTrace ========
 */
u32 PROCWRAP_GetTrace(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	u8 *pBuf;

	if (args->ARGS_PROC_GETTRACE.uMaxSize > MAX_TRACEBUFLEN)
		return DSP_ESIZE;

	pBuf = MEM_Calloc(args->ARGS_PROC_GETTRACE.uMaxSize, MEM_NONPAGED);
	if (pBuf != NULL) {
		status = PROC_GetTrace(args->ARGS_PROC_GETTRACE.hProcessor,
				      pBuf, args->ARGS_PROC_GETTRACE.uMaxSize);
	} else {
		status = DSP_EMEMORY;
	}
	cp_to_usr(args->ARGS_PROC_GETTRACE.pBuf, pBuf, status,
		 args->ARGS_PROC_GETTRACE.uMaxSize);
	kfree(pBuf);

	return status;
}

/*
 * ======== PROCWRAP_Load ========
 */
u32 PROCWRAP_Load(union Trapped_Args *args, void *pr_ctxt)
{
	s32 i, len;
	DSP_STATUS status = DSP_SOK;
	char *temp;
	s32 count = args->ARGS_PROC_LOAD.iArgc;
	u8 **argv = NULL, **envp = NULL;

	if (count <= 0 || count > MAX_LOADARGS) {
		status = DSP_EINVALIDARG;
		goto func_cont;
	}

	argv = MEM_Alloc(count * sizeof(u8 *), MEM_NONPAGED);
	if (!argv) {
		status = DSP_EMEMORY;
		goto func_cont;
	}

	cp_fm_usr(argv, args->ARGS_PROC_LOAD.aArgv, status, count);
	if (DSP_FAILED(status)) {
		kfree(argv);
		argv = NULL;
		goto func_cont;
	}

	for (i = 0; i < count; i++) {
		if (argv[i]) {
			/* User space pointer to argument */
			temp = (char *) argv[i];
			/* len is increased by 1 to accommodate NULL */
			len = strlen_user((char *)temp) + 1;
			/* Kernel space pointer to argument */
			argv[i] = MEM_Alloc(len, MEM_NONPAGED);
			if (argv[i]) {
				cp_fm_usr(argv[i], temp, status, len);
				if (DSP_FAILED(status)) {
					kfree(argv[i]);
					argv[i] = NULL;
					goto func_cont;
				}
			} else {
				status = DSP_EMEMORY;
				goto func_cont;
			}
		}
	}
	/* TODO: validate this */
	if (args->ARGS_PROC_LOAD.aEnvp) {
		/* number of elements in the envp array including NULL */
		count = 0;
		do {
			get_user(temp, args->ARGS_PROC_LOAD.aEnvp + count);
			count++;
		} while (temp);
		envp = MEM_Alloc(count * sizeof(u8 *), MEM_NONPAGED);
		if (!envp) {
			status = DSP_EMEMORY;
			goto func_cont;
		}

		cp_fm_usr(envp, args->ARGS_PROC_LOAD.aEnvp, status, count);
		if (DSP_FAILED(status)) {
			kfree(envp);
			envp = NULL;
			goto func_cont;
		}
		for (i = 0; envp[i]; i++) {
			/* User space pointer to argument */
			temp = (char *)envp[i];
			/* len is increased by 1 to accommodate NULL */
			len = strlen_user((char *)temp) + 1;
			/* Kernel space pointer to argument */
			envp[i] = MEM_Alloc(len, MEM_NONPAGED);
			if (envp[i]) {
				cp_fm_usr(envp[i], temp, status, len);
				if (DSP_FAILED(status)) {
					kfree(envp[i]);
					envp[i] = NULL;
					goto func_cont;
				}
			} else {
				status = DSP_EMEMORY;
				goto func_cont;
			}
		}
	}

	if (DSP_SUCCEEDED(status)) {
		status = PROC_Load(args->ARGS_PROC_LOAD.hProcessor,
				args->ARGS_PROC_LOAD.iArgc,
				(CONST char **)argv, (CONST char **)envp);
	}
func_cont:
	if (envp) {
		i = 0;
		while (envp[i])
			kfree(envp[i++]);

		kfree(envp);
	}

	if (argv) {
		count = args->ARGS_PROC_LOAD.iArgc;
		for (i = 0; (i < count) && argv[i]; i++)
			kfree(argv[i]);

		kfree(argv);
	}

	return status;
}

/*
 * ======== PROCWRAP_Map ========
 */
u32 PROCWRAP_Map(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	void *pMapAddr;

	if (!args->ARGS_PROC_MAPMEM.ulSize)
		return DSP_ESIZE;

	status = PROC_Map(args->ARGS_PROC_MAPMEM.hProcessor,
			 args->ARGS_PROC_MAPMEM.pMpuAddr,
			 args->ARGS_PROC_MAPMEM.ulSize,
			 args->ARGS_PROC_MAPMEM.pReqAddr, &pMapAddr,
			 args->ARGS_PROC_MAPMEM.ulMapAttr, pr_ctxt);
	if (DSP_SUCCEEDED(status)) {
		if (put_user(pMapAddr, args->ARGS_PROC_MAPMEM.ppMapAddr)) {
			status = DSP_EINVALIDARG;
			PROC_UnMap(args->ARGS_PROC_MAPMEM.hProcessor,
				pMapAddr, pr_ctxt);
		}

	}
	return status;
}

/*
 * ======== PROCWRAP_RegisterNotify ========
 */
u32 PROCWRAP_RegisterNotify(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	struct DSP_NOTIFICATION notification;

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	status = PROC_RegisterNotify(args->ARGS_PROC_REGISTER_NOTIFY.hProcessor,
				    args->ARGS_PROC_REGISTER_NOTIFY.uEventMask,
				    args->ARGS_PROC_REGISTER_NOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_PROC_REGISTER_NOTIFY.hNotification, &notification,
		 status, 1);
	return status;
}

/*
 * ======== PROCWRAP_ReserveMemory ========
 */
u32 PROCWRAP_ReserveMemory(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	void *pRsvAddr;

	if ((args->ARGS_PROC_RSVMEM.ulSize <= 0) ||
		(args->ARGS_PROC_RSVMEM.ulSize & (PG_SIZE_4K - 1)) != 0)
		return DSP_ESIZE;

	status = PROC_ReserveMemory(args->ARGS_PROC_RSVMEM.hProcessor,
				   args->ARGS_PROC_RSVMEM.ulSize, &pRsvAddr,
				   pr_ctxt);
	if (DSP_SUCCEEDED(status)) {
		if (put_user(pRsvAddr, args->ARGS_PROC_RSVMEM.ppRsvAddr)) {
			status = DSP_EINVALIDARG;
			PROC_UnReserveMemory(args->ARGS_PROC_RSVMEM.hProcessor,
				pRsvAddr, pr_ctxt);
		}
	}
	return status;
}

/*
 * ======== PROCWRAP_Start ========
 */
u32 PROCWRAP_Start(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = PROC_Start(args->ARGS_PROC_START.hProcessor);
	return retVal;
}

/*
 * ======== PROCWRAP_UnMap ========
 */
u32 PROCWRAP_UnMap(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;

	status = PROC_UnMap(args->ARGS_PROC_UNMAPMEM.hProcessor,
			   args->ARGS_PROC_UNMAPMEM.pMapAddr, pr_ctxt);
	return status;
}

/*
 * ======== PROCWRAP_UnReserveMemory ========
 */
u32 PROCWRAP_UnReserveMemory(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;

	status = PROC_UnReserveMemory(args->ARGS_PROC_UNRSVMEM.hProcessor,
			     args->ARGS_PROC_UNRSVMEM.pRsvAddr, pr_ctxt);
	return status;
}

/*
 * ======== PROCWRAP_Stop ========
 */
u32 PROCWRAP_Stop(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = PROC_Stop(args->ARGS_PROC_STOP.hProcessor);

	return retVal;
}

/*
 * ======== NODEWRAP_Allocate ========
 */
u32 NODEWRAP_Allocate(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_UUID nodeId;
	u32 cbDataSize = 0;
	u32 __user *pSize = (u32 __user *)args->ARGS_NODE_ALLOCATE.pArgs;
	u8 *pArgs = NULL;
	struct DSP_NODEATTRIN attrIn, *pAttrIn = NULL;
	struct NODE_OBJECT *hNode;

	/* Optional argument */
	if (pSize) {
		if (get_user(cbDataSize, pSize))
			status = DSP_EFAIL;

		cbDataSize += sizeof(u32);
		if (DSP_SUCCEEDED(status)) {
			pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
			if (pArgs == NULL)
				status = DSP_EMEMORY;

		}
		cp_fm_usr(pArgs, args->ARGS_NODE_ALLOCATE.pArgs, status,
			 cbDataSize);
	}
	cp_fm_usr(&nodeId, args->ARGS_NODE_ALLOCATE.pNodeID, status, 1);
	if (DSP_FAILED(status))
		goto func_cont;
	/* Optional argument */
	if (args->ARGS_NODE_ALLOCATE.pAttrIn) {
		cp_fm_usr(&attrIn, args->ARGS_NODE_ALLOCATE.pAttrIn, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrIn = &attrIn;
		else
			status = DSP_EMEMORY;

	}
	if (DSP_SUCCEEDED(status)) {
		status = NODE_Allocate(args->ARGS_NODE_ALLOCATE.hProcessor,
				      &nodeId, (struct DSP_CBDATA *)pArgs,
				      pAttrIn, &hNode, pr_ctxt);
	}
	if (DSP_SUCCEEDED(status)) {
		cp_to_usr(args->ARGS_NODE_ALLOCATE.phNode, &hNode, status, 1);
		if (DSP_FAILED(status)) {
			status = DSP_EPOINTER;
			NODE_Delete(hNode, pr_ctxt);
		}
	}
func_cont:
	kfree(pArgs);

	return status;
}

/*
 *  ======== NODEWRAP_AllocMsgBuf ========
 */
u32 NODEWRAP_AllocMsgBuf(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_BUFFERATTR *pAttr = NULL;
	struct DSP_BUFFERATTR attr;
	u8 *pBuffer = NULL;

	if (!args->ARGS_NODE_ALLOCMSGBUF.uSize)
		return DSP_ESIZE;

	if (args->ARGS_NODE_ALLOCMSGBUF.pAttr) {	/* Optional argument */
		cp_fm_usr(&attr, args->ARGS_NODE_ALLOCMSGBUF.pAttr, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttr = &attr;

	}
	/* IN OUT argument */
	cp_fm_usr(&pBuffer, args->ARGS_NODE_ALLOCMSGBUF.pBuffer, status, 1);
	if (DSP_SUCCEEDED(status)) {
		status = NODE_AllocMsgBuf(args->ARGS_NODE_ALLOCMSGBUF.hNode,
					 args->ARGS_NODE_ALLOCMSGBUF.uSize,
					 pAttr, &pBuffer);
	}
	cp_to_usr(args->ARGS_NODE_ALLOCMSGBUF.pBuffer, &pBuffer, status, 1);
	return status;
}

/*
 * ======== NODEWRAP_ChangePriority ========
 */
u32 NODEWRAP_ChangePriority(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = NODE_ChangePriority(args->ARGS_NODE_CHANGEPRIORITY.hNode,
			args->ARGS_NODE_CHANGEPRIORITY.iPriority);

	return retVal;
}

/*
 * ======== NODEWRAP_Connect ========
 */
u32 NODEWRAP_Connect(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_STRMATTR attrs;
	struct DSP_STRMATTR *pAttrs = NULL;
	u32 cbDataSize;
	u32 __user *pSize = (u32 __user *)args->ARGS_NODE_CONNECT.pConnParam;
	u8 *pArgs = NULL;

	/* Optional argument */
	if (pSize) {
		if (get_user(cbDataSize, pSize))
			status = DSP_EFAIL;

		cbDataSize += sizeof(u32);
		if (DSP_SUCCEEDED(status)) {
			pArgs = MEM_Alloc(cbDataSize, MEM_NONPAGED);
			if (pArgs == NULL) {
				status = DSP_EMEMORY;
				goto func_cont;
			}

		}
		cp_fm_usr(pArgs, args->ARGS_NODE_CONNECT.pConnParam, status,
			 cbDataSize);
		if (DSP_FAILED(status))
			goto func_cont;
	}
	if (args->ARGS_NODE_CONNECT.pAttrs) {	/* Optional argument */
		cp_fm_usr(&attrs, args->ARGS_NODE_CONNECT.pAttrs, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttrs = &attrs;

	}
	if (DSP_SUCCEEDED(status)) {
		status = NODE_Connect(args->ARGS_NODE_CONNECT.hNode,
				     args->ARGS_NODE_CONNECT.uStream,
				     args->ARGS_NODE_CONNECT.hOtherNode,
				     args->ARGS_NODE_CONNECT.uOtherStream,
				     pAttrs, (struct DSP_CBDATA *)pArgs);
	}
func_cont:
	kfree(pArgs);

	return status;
}

/*
 * ======== NODEWRAP_Create ========
 */
u32 NODEWRAP_Create(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = NODE_Create(args->ARGS_NODE_CREATE.hNode);

	return retVal;
}

/*
 * ======== NODEWRAP_Delete ========
 */
u32 NODEWRAP_Delete(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = NODE_Delete(args->ARGS_NODE_DELETE.hNode, pr_ctxt);

	return retVal;
}

/*
 *  ======== NODEWRAP_FreeMsgBuf ========
 */
u32 NODEWRAP_FreeMsgBuf(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_BUFFERATTR *pAttr = NULL;
	struct DSP_BUFFERATTR attr;
	if (args->ARGS_NODE_FREEMSGBUF.pAttr) {	/* Optional argument */
		cp_fm_usr(&attr, args->ARGS_NODE_FREEMSGBUF.pAttr, status, 1);
		if (DSP_SUCCEEDED(status))
			pAttr = &attr;

	}

	if (!args->ARGS_NODE_FREEMSGBUF.pBuffer)
		return DSP_EPOINTER;

	if (DSP_SUCCEEDED(status)) {
		status = NODE_FreeMsgBuf(args->ARGS_NODE_FREEMSGBUF.hNode,
					args->ARGS_NODE_FREEMSGBUF.pBuffer,
					pAttr);
	}

	return status;
}

/*
 * ======== NODEWRAP_GetAttr ========
 */
u32 NODEWRAP_GetAttr(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NODEATTR attr;

	status = NODE_GetAttr(args->ARGS_NODE_GETATTR.hNode, &attr,
			     args->ARGS_NODE_GETATTR.uAttrSize);
	cp_to_usr(args->ARGS_NODE_GETATTR.pAttr, &attr, status, 1);

	return status;
}

/*
 * ======== NODEWRAP_GetMessage ========
 */
u32 NODEWRAP_GetMessage(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	struct DSP_MSG msg;

	status = NODE_GetMessage(args->ARGS_NODE_GETMESSAGE.hNode, &msg,
				args->ARGS_NODE_GETMESSAGE.uTimeout);

	cp_to_usr(args->ARGS_NODE_GETMESSAGE.pMessage, &msg, status, 1);

	return status;
}

/*
 * ======== NODEWRAP_Pause ========
 */
u32 NODEWRAP_Pause(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = NODE_Pause(args->ARGS_NODE_PAUSE.hNode);

	return retVal;
}

/*
 * ======== NODEWRAP_PutMessage ========
 */
u32 NODEWRAP_PutMessage(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_MSG msg;

	cp_fm_usr(&msg, args->ARGS_NODE_PUTMESSAGE.pMessage, status, 1);

	if (DSP_SUCCEEDED(status)) {
		status = NODE_PutMessage(args->ARGS_NODE_PUTMESSAGE.hNode, &msg,
					args->ARGS_NODE_PUTMESSAGE.uTimeout);
	}

	return status;
}

/*
 * ======== NODEWRAP_RegisterNotify ========
 */
u32 NODEWRAP_RegisterNotify(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NOTIFICATION notification;

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	if (!args->ARGS_PROC_REGISTER_NOTIFY.uEventMask)
		cp_fm_usr(&notification,
			args->ARGS_PROC_REGISTER_NOTIFY.hNotification,
			status, 1);

	status = NODE_RegisterNotify(args->ARGS_NODE_REGISTERNOTIFY.hNode,
				    args->ARGS_NODE_REGISTERNOTIFY.uEventMask,
				    args->ARGS_NODE_REGISTERNOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_NODE_REGISTERNOTIFY.hNotification, &notification,
		 status, 1);
	return status;
}

/*
 * ======== NODEWRAP_Run ========
 */
u32 NODEWRAP_Run(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = NODE_Run(args->ARGS_NODE_RUN.hNode);

	return retVal;
}

/*
 * ======== NODEWRAP_Terminate ========
 */
u32 NODEWRAP_Terminate(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	DSP_STATUS tempstatus;

	status = NODE_Terminate(args->ARGS_NODE_TERMINATE.hNode, &tempstatus);

	cp_to_usr(args->ARGS_NODE_TERMINATE.pStatus, &tempstatus, status, 1);

	return status;
}


/*
 * ======== NODEWRAP_GetUUIDProps ========
 */
u32 NODEWRAP_GetUUIDProps(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_UUID nodeId;
	struct DSP_NDBPROPS    *pnodeProps = NULL;

	cp_fm_usr(&nodeId, args->ARGS_NODE_GETUUIDPROPS.pNodeID, status, 1);
	if (DSP_FAILED(status))
		goto func_cont;
	pnodeProps = MEM_Alloc(sizeof(struct DSP_NDBPROPS), MEM_NONPAGED);
	if (pnodeProps != NULL) {
		status = NODE_GetUUIDProps(args->
					  ARGS_NODE_GETUUIDPROPS.hProcessor,
					  &nodeId, pnodeProps);
		cp_to_usr(args->ARGS_NODE_GETUUIDPROPS.pNodeProps, pnodeProps,
			 status, 1);
	} else
		status = DSP_EMEMORY;
func_cont:
	kfree(pnodeProps);
	return status;
}

/*
 * ======== STRMWRAP_AllocateBuffer ========
 */
u32 STRMWRAP_AllocateBuffer(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status;
	u8 **apBuffer = NULL;
	u32 uNumBufs = args->ARGS_STRM_ALLOCATEBUFFER.uNumBufs;

	if (uNumBufs > MAX_BUFS)
		return DSP_EINVALIDARG;

	apBuffer = MEM_Alloc((uNumBufs * sizeof(u8 *)), MEM_NONPAGED);

	status = STRM_AllocateBuffer(args->ARGS_STRM_ALLOCATEBUFFER.hStream,
				     args->ARGS_STRM_ALLOCATEBUFFER.uSize,
				     apBuffer, uNumBufs, pr_ctxt);
	if (DSP_SUCCEEDED(status)) {
		cp_to_usr(args->ARGS_STRM_ALLOCATEBUFFER.apBuffer, apBuffer,
			status, uNumBufs);
		if (DSP_FAILED(status)) {
			status = DSP_EPOINTER;
			STRM_FreeBuffer(
				args->ARGS_STRM_ALLOCATEBUFFER.hStream,
				apBuffer, uNumBufs, pr_ctxt);
		}
	}
	kfree(apBuffer);

	return status;
}

/*
 * ======== STRMWRAP_Close ========
 */
u32 STRMWRAP_Close(union Trapped_Args *args, void *pr_ctxt)
{
	return STRM_Close(args->ARGS_STRM_CLOSE.hStream, pr_ctxt);
}

/*
 * ======== STRMWRAP_FreeBuffer ========
 */
u32 STRMWRAP_FreeBuffer(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	u8 **apBuffer = NULL;
	u32 uNumBufs = args->ARGS_STRM_FREEBUFFER.uNumBufs;

	if (uNumBufs > MAX_BUFS)
		return DSP_EINVALIDARG;

	apBuffer = MEM_Alloc((uNumBufs * sizeof(u8 *)), MEM_NONPAGED);

	cp_fm_usr(apBuffer, args->ARGS_STRM_FREEBUFFER.apBuffer, status,
		 uNumBufs);

	if (DSP_SUCCEEDED(status)) {
		status = STRM_FreeBuffer(args->ARGS_STRM_FREEBUFFER.hStream,
					 apBuffer, uNumBufs, pr_ctxt);
	}
	cp_to_usr(args->ARGS_STRM_FREEBUFFER.apBuffer, apBuffer, status,
		 uNumBufs);
	kfree(apBuffer);

	return status;
}

/*
 * ======== STRMWRAP_GetEventHandle ========
 */
u32 __deprecated STRMWRAP_GetEventHandle(union Trapped_Args *args,
					void *pr_ctxt)
{
	pr_err("%s: deprecated dspbridge ioctl\n", __func__);
	return DSP_ENOTIMPL;
}

/*
 * ======== STRMWRAP_GetInfo ========
 */
u32 STRMWRAP_GetInfo(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct STRM_INFO strmInfo;
	struct DSP_STREAMINFO user;
	struct DSP_STREAMINFO *temp;

	cp_fm_usr(&strmInfo, args->ARGS_STRM_GETINFO.pStreamInfo, status, 1);
	temp = strmInfo.pUser;

	strmInfo.pUser = &user;

	if (DSP_SUCCEEDED(status)) {
		status = STRM_GetInfo(args->ARGS_STRM_GETINFO.hStream,
			 &strmInfo, args->ARGS_STRM_GETINFO.uStreamInfoSize);
	}
	cp_to_usr(temp, strmInfo.pUser, status, 1);
	strmInfo.pUser = temp;
	cp_to_usr(args->ARGS_STRM_GETINFO.pStreamInfo, &strmInfo, status, 1);
	return status;
}

/*
 * ======== STRMWRAP_Idle ========
 */
u32 STRMWRAP_Idle(union Trapped_Args *args, void *pr_ctxt)
{
	u32 retVal;

	retVal = STRM_Idle(args->ARGS_STRM_IDLE.hStream,
			args->ARGS_STRM_IDLE.bFlush);

	return retVal;
}

/*
 * ======== STRMWRAP_Issue ========
 */
u32 STRMWRAP_Issue(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;

	if (!args->ARGS_STRM_ISSUE.pBuffer)
		return DSP_EPOINTER;

	/* No need of doing cp_fm_usr for the user buffer (pBuffer)
	as this is done in Bridge internal function WMD_CHNL_AddIOReq
	in chnl_sm.c */
	status = STRM_Issue(args->ARGS_STRM_ISSUE.hStream,
			args->ARGS_STRM_ISSUE.pBuffer,
			args->ARGS_STRM_ISSUE.dwBytes,
			args->ARGS_STRM_ISSUE.dwBufSize,
			args->ARGS_STRM_ISSUE.dwArg);

	return status;
}

/*
 * ======== STRMWRAP_Open ========
 */
u32 STRMWRAP_Open(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct STRM_ATTR attr;
	struct STRM_OBJECT *pStrm;
	struct DSP_STREAMATTRIN strmAttrIn;

	cp_fm_usr(&attr, args->ARGS_STRM_OPEN.pAttrIn, status, 1);

	if (attr.pStreamAttrIn != NULL) {	/* Optional argument */
		cp_fm_usr(&strmAttrIn, attr.pStreamAttrIn, status, 1);
		if (DSP_SUCCEEDED(status)) {
			attr.pStreamAttrIn = &strmAttrIn;
			if (attr.pStreamAttrIn->lMode == STRMMODE_LDMA)
				return DSP_ENOTIMPL;
		}

	}
	status = STRM_Open(args->ARGS_STRM_OPEN.hNode,
			  args->ARGS_STRM_OPEN.uDirection,
			  args->ARGS_STRM_OPEN.uIndex, &attr, &pStrm,
			  pr_ctxt);
	cp_to_usr(args->ARGS_STRM_OPEN.phStream, &pStrm, status, 1);
	return status;
}

/*
 * ======== STRMWRAP_Reclaim ========
 */
u32 STRMWRAP_Reclaim(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	u8 *pBufPtr;
	u32 ulBytes;
	u32 dwArg;
	u32 ulBufSize;

	status = STRM_Reclaim(args->ARGS_STRM_RECLAIM.hStream, &pBufPtr,
			     &ulBytes, &ulBufSize, &dwArg);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pBufPtr, &pBufPtr, status, 1);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pBytes, &ulBytes, status, 1);
	cp_to_usr(args->ARGS_STRM_RECLAIM.pdwArg, &dwArg, status, 1);

	if (args->ARGS_STRM_RECLAIM.pBufSize != NULL) {
		cp_to_usr(args->ARGS_STRM_RECLAIM.pBufSize, &ulBufSize,
			 status, 1);
	}

	return status;
}

/*
 * ======== STRMWRAP_RegisterNotify ========
 */
u32 STRMWRAP_RegisterNotify(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct DSP_NOTIFICATION notification;

	/* Initialize the notification data structure  */
	notification.psName = NULL;
	notification.handle = NULL;

	status = STRM_RegisterNotify(args->ARGS_STRM_REGISTERNOTIFY.hStream,
				    args->ARGS_STRM_REGISTERNOTIFY.uEventMask,
				    args->ARGS_STRM_REGISTERNOTIFY.uNotifyType,
				    &notification);
	cp_to_usr(args->ARGS_STRM_REGISTERNOTIFY.hNotification, &notification,
		 status, 1);

	return status;
}

/*
 * ======== STRMWRAP_Select ========
 */
u32 STRMWRAP_Select(union Trapped_Args *args, void *pr_ctxt)
{
	u32 mask;
	struct STRM_OBJECT *aStrmTab[MAX_STREAMS];
	DSP_STATUS status = DSP_SOK;

	if (args->ARGS_STRM_SELECT.nStreams > MAX_STREAMS)
		return DSP_EINVALIDARG;

	cp_fm_usr(aStrmTab, args->ARGS_STRM_SELECT.aStreamTab, status,
		 args->ARGS_STRM_SELECT.nStreams);
	if (DSP_SUCCEEDED(status)) {
		status = STRM_Select(aStrmTab, args->ARGS_STRM_SELECT.nStreams,
				    &mask, args->ARGS_STRM_SELECT.uTimeout);
	}
	cp_to_usr(args->ARGS_STRM_SELECT.pMask, &mask, status, 1);
	return status;
}

/* CMM */

/*
 * ======== CMMWRAP_CallocBuf ========
 */
u32 __deprecated CMMWRAP_CallocBuf(union Trapped_Args *args, void *pr_ctxt)
{
	/* This operation is done in kernel */
	pr_err("%s: deprecated dspbridge ioctl\n", __func__);
	return DSP_ENOTIMPL;
}

/*
 * ======== CMMWRAP_FreeBuf ========
 */
u32 __deprecated CMMWRAP_FreeBuf(union Trapped_Args *args, void *pr_ctxt)
{
	/* This operation is done in kernel */
	pr_err("%s: deprecated dspbridge ioctl\n", __func__);
	return DSP_ENOTIMPL;
}

/*
 * ======== CMMWRAP_GetHandle ========
 */
u32 CMMWRAP_GetHandle(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct CMM_OBJECT *hCmmMgr;

	status = CMM_GetHandle(args->ARGS_CMM_GETHANDLE.hProcessor, &hCmmMgr);

	cp_to_usr(args->ARGS_CMM_GETHANDLE.phCmmMgr, &hCmmMgr, status, 1);

	return status;
}

/*
 * ======== CMMWRAP_GetInfo ========
 */
u32 CMMWRAP_GetInfo(union Trapped_Args *args, void *pr_ctxt)
{
	DSP_STATUS status = DSP_SOK;
	struct CMM_INFO cmmInfo;

	status = CMM_GetInfo(args->ARGS_CMM_GETINFO.hCmmMgr, &cmmInfo);

	cp_to_usr(args->ARGS_CMM_GETINFO.pCmmInfo, &cmmInfo, status, 1);

	return status;
}
