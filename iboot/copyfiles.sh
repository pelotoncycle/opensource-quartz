#!/bin/bash
#
# Author: James Wu
# E-Mail: james.wu@innocomm.com
# Date: 2013/08/19
#

#
# validate_folder folder
#
function validate_folder
{
	local LOCAL_FOLDER=${1}

	if !([ -d "${LOCAL_FOLDER}" ])
	then
		echo "======================================================================"
		echo "  Can't find '${LOCAL_FOLDER}' folder"
		echo "======================================================================"
		return 1
	fi
	return 0
}

#
# validate_file filepath
#
function validate_file
{
	local LOCAL_FILE=${1}

	if !([ -f "${LOCAL_FILE}" ])
	then
		echo "======================================================================"
		echo "  Can't find '${LOCAL_FILE}'"
		echo "======================================================================"
		return 1
	fi
	return 0
}

TARGET_NAME=${1}
TARGET_PATH=${2}

ANDROID_IBOOT_ROOT_PATH=`pwd`
ANDROID_IBOOT_OUT="${ANDROID_IBOOT_ROOT_PATH}/out/${TARGET_NAME}"

validate_folder "${ANDROID_IBOOT_OUT}"
if [ $? -ne 0 ]; then exit 1; fi

validate_folder "${TARGET_PATH}"
if [ $? -ne 0 ]; then exit 1; fi

validate_file "${ANDROID_IBOOT_OUT}/MLO"
if [ $? -ne 0 ]; then exit 1; fi

validate_file "${ANDROID_IBOOT_OUT}/iboot.img"
if [ $? -ne 0 ]; then exit 1; fi

validate_file "${ANDROID_IBOOT_OUT}/spl/u-boot-spl.bin"
if [ $? -ne 0 ]; then exit 1; fi

cp -fv ${ANDROID_IBOOT_OUT}/MLO ${TARGET_PATH}/iboot_MLO_gp 
cp -fv ${ANDROID_IBOOT_OUT}/iboot.img ${TARGET_PATH}/iboot.img 
cp -fv ${ANDROID_IBOOT_OUT}/spl/u-boot-spl.bin ${TARGET_PATH}/iboot_MLO.bin 

sync

exit 0
