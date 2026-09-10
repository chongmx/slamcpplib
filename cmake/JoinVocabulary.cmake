# ---------------------------------------------------------------------------
# Reassemble resources/vocabulary/superpoint_voc.yml.gz from its parts.
#
# The archive is 117 MB, past GitHub's 100 MB per-file limit, so the repository
# carries it as 45 MiB chunks named superpoint_voc.yml.gz.part00 and upward.
# Joining at configure time means a fresh clone builds without a manual step.
#
# scripts/join_vocabulary.sh does the same thing for people who are not
# configuring a build. Both read the expected digest from the .sha256 sidecar,
# so the checksum lives in exactly one place.
# ---------------------------------------------------------------------------

function(slamcpp_join_vocabulary out_file)
    get_filename_component(dir "${out_file}" DIRECTORY)
    get_filename_component(name "${out_file}" NAME)

    file(GLOB parts "${out_file}.part*")
    list(FILTER parts EXCLUDE REGEX "\\.sha256$")
    list(SORT parts)

    if(NOT parts)
        # Nothing to join. An unsplit checkout is a legitimate state.
        if(NOT EXISTS "${out_file}")
            message(STATUS "slamcpp: ${name} absent and no parts to join it from")
        endif()
        return()
    endif()

    # The joined size is the sum of the parts. Comparing against it is a cheap
    # way to skip the digest on every reconfigure, which would otherwise hash
    # 117 MB each time.
    set(expected_size 0)
    foreach(part IN LISTS parts)
        file(SIZE "${part}" part_size)
        math(EXPR expected_size "${expected_size} + ${part_size}")
    endforeach()

    if(EXISTS "${out_file}")
        file(SIZE "${out_file}" actual_size)
        if(actual_size EQUAL expected_size)
            return()
        endif()
        message(STATUS "slamcpp: ${name} is ${actual_size} bytes, expected "
                       "${expected_size}; rejoining")
    endif()

    list(LENGTH parts part_count)
    message(STATUS "slamcpp: joining ${name} from ${part_count} parts")

    if(CMAKE_VERSION VERSION_LESS 3.18)
        message(FATAL_ERROR
            "Joining ${name} needs CMake 3.18 or newer for \"cmake -E cat\". "
            "This build is ${CMAKE_VERSION}. Either upgrade CMake or run "
            "scripts/join_vocabulary.sh by hand first.")
    endif()

    # Write to a temporary and move it into place, so an interrupted configure
    # cannot leave a truncated archive that later looks joined. There is no
    # file() sub-command that concatenates binary files, so this shells out to
    # cmake -E cat, which opens its inputs in binary mode.
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E cat ${parts}
        OUTPUT_FILE "${out_file}.tmp"
        RESULT_VARIABLE cat_result
        ERROR_VARIABLE cat_error)

    if(NOT cat_result EQUAL 0)
        file(REMOVE "${out_file}.tmp")
        message(FATAL_ERROR "Joining ${name} failed: ${cat_error}")
    endif()

    set(sidecar "${out_file}.sha256")
    if(EXISTS "${sidecar}")
        file(READ "${sidecar}" sidecar_text)
        string(REGEX MATCH "[0-9a-fA-F]+" expected_sha "${sidecar_text}")
        string(TOLOWER "${expected_sha}" expected_sha)

        file(SHA256 "${out_file}.tmp" actual_sha)
        if(NOT actual_sha STREQUAL expected_sha)
            file(REMOVE "${out_file}.tmp")
            message(FATAL_ERROR
                "Checksum mismatch joining ${name}.\n"
                "  expected ${expected_sha}\n"
                "  actual   ${actual_sha}\n"
                "The parts in ${dir} are corrupt or incomplete.")
        endif()
    else()
        message(WARNING "slamcpp: no ${name}.sha256, joining unverified")
    endif()

    file(RENAME "${out_file}.tmp" "${out_file}")
    message(STATUS "slamcpp: ${name} joined and verified")
endfunction()
