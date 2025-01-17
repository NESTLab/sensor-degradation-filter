#!/usr/bin/env bash

PREFIX="rmsd_"
EXT="csv"
FLATDIR="tmp/flat_dir_for_tar"

# Create temporary directory to copy files to
mkdir -p ${FLATDIR}

# Find all file paths and store them in a variable
FILES=$(find . -type f -name "${PREFIX}*.${EXT}" ! -path "${FLATDIR}/*")

# Copy files into the target directory
echo -n "Copying files into ${FLATDIR}..."
for FILE in ${FILES}; do
    cp "${FILE}" "${FLATDIR}/"
done
echo -e "Done\n"

# Compress directory into archive
echo -e "Compressing archive ${PREFIX}${EXT}.tar.gz..."
tar -czvf ${PREFIX}${EXT}.tar.gz -C ${FLATDIR} .
echo -e "Done"

# Clean up
rm -rf ${FLATDIR}
