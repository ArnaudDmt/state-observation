macro(ADD_CSV_READER name)
    add_executable(${name} ${name}.cpp)

    target_link_libraries(${name} PUBLIC state-observation)
endmacro()

ADD_CSV_READER(tilt-estimator-csv-reader)
ADD_CSV_READER(viking_txt_reader_panoramis)