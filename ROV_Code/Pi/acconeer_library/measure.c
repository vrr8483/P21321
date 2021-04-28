/*H**********************************************************************
 * * FILENAME :  measure.c
 * *
 * * DESCRIPTION :
 * *       File used to manually call the python function that measures
 * *       the ice thickness via radar
 * *
 * * PUBLIC FUNCTIONS :
 * *       double     measure(fileName, funcName)       
 * *
 * * AUTHOR :    Victor Runeare		vrr8483@rit.edu
 * *
 * */

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "measure.h"

/*
 * * PURPOSE : Gets value from python file and returns a double
 * * 
 * * INPUTS : 	fileName-File name of the python file
 * *		funcName-Name of the function in the python file
 * *
 * * RETURN :  double-Radar value found
 * *
 * *F*/

double measure(){
	char* fileName = "envelope.py";
	char* funcName = "main";
	
    //Initialize python.h data
    PyObject *pName, *pModule, *pFunc;
    PyObject *pArgs, *pValue;

    //Input checks
    if (fileName == NULL) {
        fprintf(stderr,"Usage: call pythonfile filename\n");
        return 1;
    }
    
    if (funcName == NULL) {
	    fprintf(stderr,"Usage: call pythonfile funcname\n");
            return 1;
    }

    //Initialize python.h
    Py_Initialize();

    //Converts fileName to string in python
    pName = PyUnicode_DecodeFSDefault(fileName);
    
    //Appends the current path to python
    PyRun_SimpleString("import sys\nsys.path.append('/home/pi/testing/acconeer-python-exploration-master')\n");
    
    //Imports filename
    pModule = PyImport_Import(pName);
    Py_DECREF(pName);

    if (pModule != NULL) {
	//Grabs function from file
        pFunc = PyObject_GetAttrString(pModule, funcName);
	
        if (pFunc && PyCallable_Check(pFunc)) {
            
	    //Creates a new NULL tuple since no arguments are required
	    pArgs = PyTuple_New(0);
            
	    //Calls the function
            pValue = PyObject_CallObject(pFunc, pArgs);
            Py_DECREF(pArgs);
            if (pValue != NULL) {
                printf("Result of call: %ld\n", PyLong_AsLong(pValue));
                double temp = PyLong_AsLong(pValue);
		Py_DECREF(pValue);
		//returns value
		Py_Finalize();
		return temp;
            }
            else {
                Py_DECREF(pFunc);
                Py_DECREF(pModule);
                PyErr_Print();
                fprintf(stderr,"Call failed\n");
                return 1;
            }
        }
        else {
            if (PyErr_Occurred())
                PyErr_Print();
            fprintf(stderr, "Cannot find function \"%s\"\n", funcName);
        }
        Py_XDECREF(pFunc);
        Py_DECREF(pModule);
    }
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \"%s\"\n", fileName);
        return 1;
    }
    if (Py_FinalizeEx() < 0) {
        return 120;
    }
    return 0;
}

