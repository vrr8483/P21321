#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "measure.h"

int
main(int argc, char *argv[])
{
    measure("envelope","main");
}

