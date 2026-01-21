# DARTT Tools

Tools for working with DARTT protocol configurations.

## dartt-describe.py

Extracts struct layout from ELF DWARF debug info. Use this to generate configuration files for dartt-dashboard.

### Requirements

```bash
pip install -r requirements.txt
```

### Usage

```bash
# Basic usage - output to stdout
python dartt-describe.py firmware.elf gl_dp

# Save to file
python dartt-describe.py firmware.elf gl_dp -o config.json

# Flatten nested structs into dot-notation paths
python dartt-describe.py firmware.elf gl_dp --flat -o config.json
```

### Compilation Requirements

Your firmware must be compiled with debug symbols (`-g` flag):

```bash
arm-none-eabi-gcc -g -O2 main.c -o firmware.elf
```

### Output Format

The tool outputs JSON with the following structure:

```json
{
  "symbol": "gl_dp",
  "address": "0x20001000",
  "address_int": 536875008,
  "size": 256,
  "type": {
    "type": "struct",
    "struct_name": "device_params_t",
    "size": 256,
    "fields": [
      {
        "name": "kp",
        "offset": 0,
        "type_info": {
          "type": "float",
          "size": 4,
          "encoding": "float"
        }
      },
      {
        "name": "ki",
        "offset": 4,
        "type_info": {
          "type": "float",
          "size": 4,
          "encoding": "float"
        }
      }
    ]
  }
}
```

With `--flat`, adds a `flat_fields` array for easy dashboard consumption:

```json
{
  "flat_fields": [
    {"name": "kp", "offset": 0, "size": 4, "type": "float"},
    {"name": "ki", "offset": 4, "size": 4, "type": "float"},
    {"name": "nested.value", "offset": 8, "size": 4, "type": "int32_t"},
    {"name": "array[0]", "offset": 12, "size": 4, "type": "uint32_t"},
    {"name": "array[1]", "offset": 16, "size": 4, "type": "uint32_t"}
  ]
}
```

### Supported Types

- Primitive types (int, float, char, etc.)
- Fixed-width types (uint8_t, int32_t, etc.)
- Structs (nested)
- Unions
- Arrays (including multidimensional)
- Enums (with enumerator names/values)
- Pointers
- Bitfields
- Typedefs
- const/volatile qualifiers

### Integration with dartt-dashboard

The output JSON is designed for use with dartt-dashboard:

1. Generate config: `python dartt-describe.py firmware.elf gl_dp --flat -o config.json`
2. Launch dashboard: `dartt-dashboard --config config.json --port COM3`
3. Select fields to subscribe and plot
