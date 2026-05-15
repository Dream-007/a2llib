# A2L JSON Exporter

Command line tool that exports A2L measurements/signals and conversion definitions to JSON.

Build:

```bash
cmake -S a2l_json_exporter -B build-a2l-json-exporter
cmake --build build-a2l-json-exporter -- -j$(nproc)
```

Run:

```bash
./build-a2l-json-exporter/a2l_json_exporter input.a2l output.json
```

The default CMake configuration links against:

```text
/home/shiheping/QianLiPrj/a2llib/build-local-vcpkg/liba2l.a
```

Override paths when needed:

```bash
cmake -S a2l_json_exporter -B build-a2l-json-exporter \
  -DA2L_STATIC_LIB=/path/to/liba2l.a \
  -DA2L_INCLUDE_DIR=/path/to/a2llib/include \
  -DA2L_VCPKG_PREFIX=/path/to/vcpkg_installed/x64-linux
```
