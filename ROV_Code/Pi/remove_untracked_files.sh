#!/bin/bash

rm -rf $(git ls-files --others --exclude-standard)
