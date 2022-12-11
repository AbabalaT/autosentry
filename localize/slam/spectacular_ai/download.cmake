function(download_file url filename)
    message(STATUS "Download to ${filename} ...")
    file(DOWNLOAD ${url} ${filename})
endfunction()

function(download_file_with_hash url filename hash_type hash)
    message(STATUS "Download to ${filename} ...")
    file(DOWNLOAD ${url} ${filename} EXPECTED_HASH ${hash_type}=${hash})
endfunction()

function(extract_file filename extract_dir)
    message(STATUS "Extract to ${extract_dir} ...")

    # 创建临时目录，用来解压，如果已经存在，则删除
    # 这里用的解压命令，是 cmake 内部实现的解压命令，可以实现跨平台解压：
    # cmake -E tar -xf filename.tgz
    set(temp_dir ${CMAKE_BINARY_DIR}/tmp_for_extract.dir)
    if(EXISTS ${temp_dir})
        file(REMOVE_RECURSE ${temp_dir})
    endif()
    file(MAKE_DIRECTORY ${temp_dir})
    execute_process(COMMAND ${CMAKE_COMMAND} -E tar -xf ${filename}
            WORKING_DIRECTORY ${temp_dir})

    # 这里比较关键，解压之后的临时目录中，可能是单个文件夹，里面包含着我们需要的内容，
    # 也可能是直接就包含着我们需要的内容，因此，这里就统一处理，如果包含单独的文件夹
    # 则将安装源目录设置为临时目录的下级目录
    file(GLOB contents "${temp_dir}/Linux_Ubuntu_x86-64/*")
    list(LENGTH contents n)
    if(NOT n EQUAL 1 OR NOT IS_DIRECTORY "${contents}")
        set(contents "${temp_dir}/Linux_Ubuntu_x86-64")
    endif()

    get_filename_component(contents ${contents} ABSOLUTE)
    # 这里选择 INSTALL 而不是 COPY，因为可以打印出文件拷贝的状态
    file(INSTALL "${contents}/" DESTINATION ${extract_dir})

    # 别忘删除临时目录
    file(REMOVE_RECURSE ${temp_dir})
endfunction()

function(download_and_extract)
    set(options REMOVE_EXTRACT_DIR_IF_EXISTS)
    set(oneValueArgs DESTINATION RENAME)
    set(multiValueArgs)
    set(oneValueArgs URL FILENAME HASH_TYPE HASH EXTRACT_DIR)
    cmake_parse_arguments(DAE "${options}" "${oneValueArgs}" "${multiValueArgs}"
            ${ARGN})
    if(NOT DEFINED DAE_URL)
        message(FATAL_ERROR "Missing URL")
    endif()
    if(NOT DEFINED DAE_FILENAME)
        message(FATAL_ERROR "Missing FILENAME")
    endif()
    if(NOT DEFINED DAE_HASH_TYPE)
        message(FATAL_ERROR "Missing HASH_TYPE")
    endif()
    if(NOT DEFINED DAE_HASH)
        message(FATAL_ERROR "Missing HASH")
    endif()
    if(NOT DEFINED DAE_EXTRACT_DIR)
        message(FATAL_ERROR "Missing EXTRACT_DIR")
    endif()

    if(EXISTS ${DAE_EXTRACT_DIR})
        if(DAE_REMOVE_EXTRACT_DIR_IF_EXISTS)
            message(STATUS "${DAE_EXTRACT_DIR} already exists, removing...")
            file(REMOVE_RECURSE ${DAE_EXTRACT_DIR})
        else()
            message(
                    STATUS "${DAE_EXTRACT_DIR} already exists, skip download & extract")
            return()
        endif()
    endif()

    if(EXISTS ${DAE_FILENAME})
        file(${DAE_HASH_TYPE} ${DAE_FILENAME} _ACTUAL_CHKSUM)
        if(NOT (${DAE_HASH} STREQUAL ${_ACTUAL_CHKSUM}))
            message(STATUS "Expect ${DAE_HASH_TYPE}=${DAE_HASH}")
            message(STATUS "Actual ${DAE_HASH_TYPE}=${_ACTUAL_CHKSUM}")
            message(WARNING "File hash mismatch, remove & retry ...")
            file(REMOVE ${DAE_FILENAME})
            download_file_with_hash(${DAE_URL} ${DAE_FILENAME} ${DAE_HASH_TYPE}
                    ${DAE_HASH})
        else()
            message(STATUS "Using exists local file ${DAE_FILENAME}")
        endif()
    else()
        download_file_with_hash(${DAE_URL} ${DAE_FILENAME} ${DAE_HASH_TYPE}
                ${DAE_HASH})
    endif()
    extract_file(${DAE_FILENAME} ${DAE_EXTRACT_DIR})
endfunction()


