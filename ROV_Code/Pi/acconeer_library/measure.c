/*H**********************************************************************
 * * FILENAME :  measure.c
 * *
 * * DESCRIPTION :
 * *       File used to manually call the python function that measures
 * *       the ice thickness via radar
 * *
 * *
 * * AUTHOR :    Victor Runeare		vrr8483@rit.edu
 * *
 * */

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "measure.h"

//Initialize python.h data
PyObject *pName, *pModule, *pFunc;
PyObject *pArgs, *pValue;

//call to initialize the python embedded environment.
//returns: 0 on success, -1 on failure.
int setup_radar(){
	
	char* fileName = "envelope";
	char* funcName = "main";
	
	//Input checks
	if (fileName == NULL) {
		fprintf(stderr,"Usage: call pythonfile filename\n");
		return -1;
	}
	
	if (funcName == NULL) {
		fprintf(stderr,"Usage: call pythonfile funcname\n");
		return -1;
	}
	
	//Initialize python.h
	Py_Initialize();
	
	//Converts fileName to string in python
	pName = PyUnicode_DecodeFSDefault(fileName);
	
	//Appends the current path to python
	PyRun_SimpleString("import sys\nsys.path.append('/home/pi/P21321/ROV_Code/Pi/acconeer_library')\n");
	
	//Imports filename
	pModule = PyImport_Import(pName);
	
	//cleanup pName
	Py_DECREF(pName);
	
	if (pModule == NULL){
		PyErr_Print();
		fprintf(stderr, "Failed to load \"%s\"\n", fileName);
		return cleanup_radar();
	}
	
	//Grabs function from file
	pFunc = PyObject_GetAttrString(pModule, funcName);
	
	if (!pFunc || !PyCallable_Check(pFunc)){
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Cannot find function \"%s\"\n", funcName);
		return cleanup_radar();
	}
	
	//Creates a new NULL tuple since no arguments are required
	pArgs = PyTuple_New(0);

	return 0;
}

//Measures ice thickness with radar.
//returns: Ice thickness on success, -1 on failure.
double measure(){
	
	//Calls the function
	pValue = PyObject_CallObject(pFunc, pArgs);
	
	if (pValue != NULL) {
		
		//printf("Result of call: %ld\n", PyLong_AsLong(pValue));
		return PyLong_AsLong(pValue);
		
	} else {
		
		PyErr_Print();
		fprintf(stderr, "Call failed\n");
		
		return cleanup_radar();
	}
	
}

//Cleans up python session
//returns: 0 on success, -1 on failure.
int cleanup_radar(){
	
	Py_XDECREF(pFunc);
	Py_XDECREF(pModule);
	Py_XDECREF(pArgs);
	Py_XDECREF(pValue);
	
	return Py_FinalizeEx();
}

