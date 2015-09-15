#include <uchar.h> // undefined char16_t in matrix.h->tmwtypes.h ...bug in Matlab headers?
#include "matrix.h"
#include "mex.h"
#include <string.h>
#include <spnav.h>

// Compiler command line with system-wide libspnav installation:
// mex spnav.c -lspnav
// Compiler command line with custom libspnav installation (adapt include/library path according to your installation!)
// mex spnav.c -I../libspnav -L../libspnav -lspnav

#define printf mexPrintf

typedef struct {
	int x, y, z, rx, ry, rz;
	unsigned buttonDownMask;
} SpState_t;

static SpState_t defaultState; // the library does not support more than one device (yet)
	
static bool isOpen() {
	return spnav_fd() != -1;
}
static void resetState(SpState_t *s) {
	s->x = s->y = s->z = 0;
	s->rx = s->ry = s->rz = 0;
	s->buttonDownMask = 0;
}

static double scaleTrans(int t) {
	return (double)t / 350.0;
}
static double scaleRot(int r) {
	return (double)r / 350.0;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
	if (nrhs > 1) mexErrMsgTxt("Too many input arguments");
	if (nlhs > 1) mexErrMsgTxt("Too many output arguments");
	
	enum {
		Cmd_Open,
		Cmd_Close,
		Cmd_Read
	} mode = Cmd_Read;
		
	if (nrhs == 1) {
		const mxArray *mxCmd = prhs[0];
		if (!mxIsChar(mxCmd)) mexErrMsgTxt("Invalid argument type - expected char array");
		
		char *pCmd = mxArrayToString(mxCmd);
		if (strcasecmp(pCmd, "open") == 0) mode = Cmd_Open;
		else if (strcasecmp(pCmd, "close") == 0) mode = Cmd_Close;
		else {
			mxFree(pCmd);
			mexErrMsgTxt("Invalid command");
		}
		mxFree(pCmd);		
	}

	if (mode == Cmd_Open) {
		bool success = spnav_open() != -1;
		if (success) {
			spnav_remove_events(SPNAV_EVENT_ANY);
			resetState(&defaultState);
		}
		plhs[0] = mxCreateLogicalScalar(success);
	} else if (mode == Cmd_Close) {
		bool success = spnav_close() != -1;
		plhs[0] = mxCreateLogicalScalar(success);
	} else if (mode == Cmd_Read) {		
		if (!isOpen()) mexErrMsgTxt("Connection was not opened");
		
		spnav_event ev;
		SpState_t *pS = &defaultState;
		
		const int maxBtn = 2;
		int pressEvents[maxBtn], releaseEvents[maxBtn];
		memset(pressEvents, 0, sizeof(pressEvents));
		memset(releaseEvents, 0, sizeof(releaseEvents));
		
		int motionEvents = 0;
		int buttonEvents = 0;
		while(true) {
			// process pending events...
			int eventType = spnav_poll_event(&ev);						
			if (eventType == SPNAV_EVENT_MOTION) {
				motionEvents++;
				pS->x = ev.motion.x;
				pS->y = ev.motion.y;
				pS->z = ev.motion.z;
				pS->rx = ev.motion.rx;
				pS->ry = ev.motion.ry;
				pS->rz = ev.motion.rz;
			} else if (eventType == SPNAV_EVENT_BUTTON) {				
				int btn = ev.button.bnum;
				if (btn < maxBtn) {
					buttonEvents++;
					if (ev.button.press) {
						pS->buttonDownMask |= (1 << btn);
						pressEvents[btn]++;
					} else {
						pS->buttonDownMask &= ~(1 << btn);
						releaseEvents[btn]++;
					}
				}
			} else break;
		}		
		
		if (nlhs >= 1) {			
			const char *fieldsNames[] = {"motionEvents", "buttonEvents", "trans", "rot", "buttons", "pressEvents", "releaseEvents" };
			mxArray *mxOut = mxCreateStructMatrix(1, 1, sizeof(fieldsNames) / sizeof(fieldsNames[0]), fieldsNames);
			mxSetField(mxOut, 0, fieldsNames[0], mxCreateDoubleScalar(motionEvents));
			mxSetField(mxOut, 0, fieldsNames[1], mxCreateDoubleScalar(buttonEvents));
			
			mxArray *mxTrans = mxCreateDoubleMatrix(3, 1, mxREAL);
			double *pTrans = mxGetPr(mxTrans);
			pTrans[0] = scaleTrans(pS->x); pTrans[1] = scaleTrans(pS->y); pTrans[2] = scaleTrans(pS->z);
			mxSetField(mxOut, 0, fieldsNames[2], mxTrans);
			
			mxArray *mxRot = mxCreateDoubleMatrix(3, 1, mxREAL);
			double *pRot = mxGetPr(mxRot);
			pRot[0] = scaleRot(pS->rx); pRot[1] = scaleRot(pS->ry); pRot[2] = scaleRot(pS->rz);
			mxSetField(mxOut, 0, fieldsNames[3], mxRot);

			mxArray *mxButtons = mxCreateLogicalMatrix(1, maxBtn);
			mxArray *mxPressEvents = mxCreateDoubleMatrix(1, maxBtn, mxREAL);
			mxArray *mxReleaseEvents = mxCreateDoubleMatrix(1, maxBtn, mxREAL);
			mxLogical *pButtons = (mxLogical *)mxGetData(mxButtons);
			double *pPressEvents = mxGetPr(mxPressEvents);
			double *pReleaseEvents = mxGetPr(mxReleaseEvents);
			for (int i = 0; i < maxBtn; ++i) {
				pButtons[i] = (pS->buttonDownMask & (1 << i)) ? 1 : 0;
				pPressEvents[i] = pressEvents[i];
				pReleaseEvents[i] = releaseEvents[i];
			}
			mxSetField(mxOut, 0, fieldsNames[4], mxButtons);
			mxSetField(mxOut, 0, fieldsNames[5], mxPressEvents);
			mxSetField(mxOut, 0, fieldsNames[6], mxReleaseEvents);
			
			plhs[0] = mxOut;
		}
	}	
}
