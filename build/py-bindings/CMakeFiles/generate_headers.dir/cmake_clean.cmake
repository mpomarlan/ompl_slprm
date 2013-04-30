FILE(REMOVE_RECURSE
  "CMakeFiles/generate_headers"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/generate_headers.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
