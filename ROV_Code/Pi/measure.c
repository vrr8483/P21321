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
PyObject *pModule, *pFunc, *pValue = NULL;

const char* fileName = "envelope";

const char* funcName_setup = "initialize";
const char* funcName_maincall = "main";
const char* funcName_cleanup = "disconnect";

//call to initialize the python embedded environment.
//returns: 0 on success, -1 on failure.
int setup_radar(){
	
	//Initialize python.h
	Py_Initialize();
		
	
	const int cwd_buf_size = 1024;
	char cwd_buf[cwd_buf_size];
	getcwd(cwd_buf, cwd_buf_size);
	strncat(cwd_buf, "/acconeer_library", cwd_buf_size - strlen(cwd_buf));
	//printf("Path: %s\n", cwd_buf);
	
	const int pystr_size = 1024*2;
	char python_simpstring[pystr_size];
	
	sprintf(python_simpstring, "import sys\nsys.path.append('%s')\n", cwd_buf);

	//Appends the current path to python
	PyRun_SimpleString(python_simpstring);
	

	//Converts fileName to string in python
	PyObject* pName = PyUnicode_DecodeFSDefault(fileName);
	
	//Imports filename
	pModule = PyImport_Import(pName);
	
	//cleanup pName
	Py_CLEAR(pName);
	
	
	if (pModule == NULL){
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Failed to load \"%s\"\n", fileName);
		cleanup_radar();
		return -1;
	}

	
	//call setup code within python
	pFunc = PyObject_GetAttrString(pModule, funcName_setup);
	
	if (!pFunc || !PyCallable_Check(pFunc)){
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Cannot find function \"%s\" in \"%s\" (or it is not callable)\n", funcName_setup, fileName);
		cleanup_radar();
		return -1;
	}
	
	pValue = PyObject_CallObject(pFunc, NULL);
	
	if (pValue == NULL) {
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Python setup failed\n");
		cleanup_radar();
		return -1;
	}

	Py_CLEAR(pFunc);
	Py_CLEAR(pValue);
	
	//set up main call code
	pFunc = PyObject_GetAttrString(pModule, funcName_maincall);
	
	if (!pFunc || !PyCallable_Check(pFunc)){
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Cannot find function \"%s\" in \"%s\" (or it is not callable)\n", funcName_maincall, fileName);
		cleanup_radar();
		return -1;
	}

	return 0;
}

//Measures ice thickness with radar.
//returns: Ice thickness on success, -1 on failure.
double measure(){
	
	//Calls the function
	pValue = PyObject_CallObject(pFunc, NULL);
	
	double retval = -1.0;

	if (pValue != NULL) {
		
		//printf("Result of call: %ld\n", PyLong_AsLong(pValue));
		retval = PyFloat_AsDouble(pValue);
		
	} else {
		
		if (PyErr_Occurred()){
			PyErr_Print();
			PyErr_Clear();
		}

		fprintf(stderr, "Radar call failed.\n");
	}

	Py_CLEAR(pValue);

	return retval;
}

//Cleans up python session
//returns: 0 on success, -1 on failure.
int cleanup_radar(){

	Py_CLEAR(pFunc);
	
	//call cleanup code within python
	pFunc = PyObject_GetAttrString(pModule, funcName_cleanup);
	
	if (!pFunc || !PyCallable_Check(pFunc)){
		
		if (PyErr_Occurred()) PyErr_Print();
		fprintf(stderr, "Cannot find function \"%s\" in \"%s\" (or it is not callable)\n", funcName_cleanup, fileName);
		
	} else {

		Py_CLEAR(pValue);
	
		pValue = PyObject_CallObject(pFunc, NULL);
	
		if (pValue == NULL) {
			if (PyErr_Occurred()) PyErr_Print();
			fprintf(stderr, "Python cleanup failed\n");
		}
	}
	
	Py_CLEAR(pFunc);
	Py_CLEAR(pModule);
	Py_CLEAR(pValue);
	
	return Py_FinalizeEx();
}

