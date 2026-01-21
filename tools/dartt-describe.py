#!/usr/bin/env python3
"""
dartt-describe.py - Extract struct layout from ELF DWARF debug info

Usage:
    python dartt-describe.py firmware.elf gl_dp -o config.json

This tool parses DWARF debug information from an ELF file to extract
the memory layout of a global symbol (typically a struct). The output
JSON can be used by dartt-dashboard to subscribe to and plot individual
fields in real-time.

Requires: pyelftools (pip install pyelftools)
"""

import argparse
import copy
import json
import sys
from pathlib import Path

try:
    from elftools.elf.elffile import ELFFile
    from elftools.dwarf.descriptions import describe_form_class
except ImportError:
    print("Error: pyelftools not installed. Run: pip install pyelftools")
    sys.exit(1)


# Mapping of DWARF base type encodings to friendly names
DW_ATE_NAMES = {
    0x01: "address",
    0x02: "boolean",
    0x03: "complex_float",
    0x04: "float",
    0x05: "signed",
    0x06: "signed_char",
    0x07: "unsigned",
    0x08: "unsigned_char",
    0x0b: "numeric_string",
    0x0c: "edited",
    0x0d: "signed_fixed",
    0x0e: "unsigned_fixed",
    0x0f: "decimal_float",
    0x10: "UTF",
}


class DWARFStructParser:
    """Parses DWARF debug info to extract struct layouts."""

    def __init__(self, elffile):
        self.elffile = elffile
        self.dwarf_info = elffile.get_dwarf_info()
        self.type_cache = {}  # Cache parsed types by offset

    def get_symbol_address(self, symbol_name):
        """Find symbol address from symbol table."""
        symtab = self.elffile.get_section_by_name('.symtab')
        if not symtab:
            return None

        for symbol in symtab.iter_symbols():
            if symbol.name == symbol_name:
                return symbol['st_value']
        return None

    def get_symbol_size(self, symbol_name):
        """Find symbol size from symbol table."""
        symtab = self.elffile.get_section_by_name('.symtab')
        if not symtab:
            return None

        for symbol in symtab.iter_symbols():
            if symbol.name == symbol_name:
                return symbol['st_size']
        return None

    def find_variable_die(self, symbol_name):
        """Find the DWARF DIE for a global variable."""
        for cu in self.dwarf_info.iter_CUs():
            for die in cu.iter_DIEs():
                if die.tag == 'DW_TAG_variable':
                    if 'DW_AT_name' in die.attributes:
                        name = die.attributes['DW_AT_name'].value
                        if isinstance(name, bytes):
                            name = name.decode('utf-8')
                        if name == symbol_name:
                            return die, cu
        return None, None

    def get_type_die(self, die, cu):
        """Follow DW_AT_type reference to get the type DIE."""
        if 'DW_AT_type' not in die.attributes:
            return None

        type_offset = die.attributes['DW_AT_type'].value

        # Handle different reference forms
        form = die.attributes['DW_AT_type'].form
        form_class = describe_form_class(form)

        if form_class == 'reference':
            # Offset relative to CU
            ref_addr = cu.cu_offset + type_offset
        else:
            ref_addr = type_offset

        return self.get_die_at_offset(ref_addr)

    def get_die_at_offset(self, offset):
        """Get DIE at a specific offset in the debug info."""
        for cu in self.dwarf_info.iter_CUs():
            for die in cu.iter_DIEs():
                if die.offset == offset:
                    return die, cu
        return None, None

    def get_attr_value(self, die, attr_name):
        """Get attribute value, handling bytes vs string."""
        if attr_name not in die.attributes:
            return None
        val = die.attributes[attr_name].value
        if isinstance(val, bytes):
            return val.decode('utf-8')
        return val

    def resolve_type(self, die, cu, depth=0):
        """
        Recursively resolve a type DIE to a type description.
        Returns a dict with type information.
        """
        if die is None:
            return {"type": "void", "size": 0}

        # Prevent infinite recursion
        if depth > 50:
            return {"type": "unknown", "size": 0, "error": "max depth exceeded"}

        # Check cache
        if die.offset in self.type_cache:
            return self.type_cache[die.offset]

        tag = die.tag
        result = {}

        if tag == 'DW_TAG_base_type':
            # Primitive type (int, float, etc.)
            name = self.get_attr_value(die, 'DW_AT_name') or "unknown"
            size = self.get_attr_value(die, 'DW_AT_byte_size') or 0
            encoding = self.get_attr_value(die, 'DW_AT_encoding')

            result = {
                "type": name,
                "size": size,
                "encoding": DW_ATE_NAMES.get(encoding, "unknown")
            }

        elif tag == 'DW_TAG_typedef':
            # Typedef - follow to underlying type
            typedef_name = self.get_attr_value(die, 'DW_AT_name')
            underlying_die, underlying_cu = self.get_type_die(die, cu)
            underlying = self.resolve_type(underlying_die, underlying_cu, depth + 1)

            result = underlying.copy()
            result["typedef"] = typedef_name

        elif tag == 'DW_TAG_pointer_type':
            # Pointer type
            size = self.get_attr_value(die, 'DW_AT_byte_size') or 4  # Default 32-bit
            pointee_die, pointee_cu = self.get_type_die(die, cu)
            pointee = self.resolve_type(pointee_die, pointee_cu, depth + 1)

            result = {
                "type": "pointer",
                "size": size,
                "pointee": pointee
            }

        elif tag == 'DW_TAG_array_type':
            # Array type
            elem_die, elem_cu = self.get_type_die(die, cu)
            elem_type = self.resolve_type(elem_die, elem_cu, depth + 1)

            # Find array dimensions from children
            dimensions = []
            for child in die.iter_children():
                if child.tag == 'DW_TAG_subrange_type':
                    upper = self.get_attr_value(child, 'DW_AT_upper_bound')
                    count = self.get_attr_value(child, 'DW_AT_count')
                    if count is not None:
                        dimensions.append(count)
                    elif upper is not None:
                        dimensions.append(upper + 1)
                    else:
                        dimensions.append(0)  # Flexible array

            total_elements = 1
            for d in dimensions:
                total_elements *= d

            result = {
                "type": "array",
                "element_type": elem_type,
                "dimensions": dimensions,
                "total_elements": total_elements,
                "size": elem_type.get("size", 0) * total_elements
            }

        elif tag == 'DW_TAG_structure_type':
            # Struct type
            name = self.get_attr_value(die, 'DW_AT_name')
            size = self.get_attr_value(die, 'DW_AT_byte_size') or 0

            fields = []
            for child in die.iter_children():
                if child.tag == 'DW_TAG_member':
                    field = self.parse_member(child, cu, depth + 1)
                    if field:
                        fields.append(field)

            result = {
                "type": "struct",
                "struct_name": name,
                "size": size,
                "fields": fields
            }

        elif tag == 'DW_TAG_union_type':
            # Union type
            name = self.get_attr_value(die, 'DW_AT_name')
            size = self.get_attr_value(die, 'DW_AT_byte_size') or 0

            fields = []
            for child in die.iter_children():
                if child.tag == 'DW_TAG_member':
                    field = self.parse_member(child, cu, depth + 1)
                    if field:
                        fields.append(field)

            result = {
                "type": "union",
                "union_name": name,
                "size": size,
                "fields": fields
            }

        elif tag == 'DW_TAG_enumeration_type':
            # Enum type
            name = self.get_attr_value(die, 'DW_AT_name')
            size = self.get_attr_value(die, 'DW_AT_byte_size') or 4

            enumerators = []
            for child in die.iter_children():
                if child.tag == 'DW_TAG_enumerator':
                    enum_name = self.get_attr_value(child, 'DW_AT_name')
                    enum_val = self.get_attr_value(child, 'DW_AT_const_value')
                    enumerators.append({"name": enum_name, "value": enum_val})

            result = {
                "type": "enum",
                "enum_name": name,
                "size": size,
                "enumerators": enumerators
            }

        elif tag == 'DW_TAG_const_type':
            # const qualifier - follow to underlying type
            underlying_die, underlying_cu = self.get_type_die(die, cu)
            result = self.resolve_type(underlying_die, underlying_cu, depth + 1)
            result = result.copy()
            result["const"] = True

        elif tag == 'DW_TAG_volatile_type':
            # volatile qualifier - follow to underlying type
            underlying_die, underlying_cu = self.get_type_die(die, cu)
            result = self.resolve_type(underlying_die, underlying_cu, depth + 1)
            result = result.copy()
            result["volatile"] = True

        else:
            result = {
                "type": "unknown",
                "tag": tag,
                "size": self.get_attr_value(die, 'DW_AT_byte_size') or 0
            }

        self.type_cache[die.offset] = result
        return result

    def parse_member(self, die, cu, depth):
        """Parse a struct/union member DIE."""
        name = self.get_attr_value(die, 'DW_AT_name')

        # Get byte_offset (for struct members) - relative to parent struct
        # Default to 0 if not specified (common for unions, first members)
        byte_offset = 0
        if 'DW_AT_data_member_location' in die.attributes:
            loc = die.attributes['DW_AT_data_member_location']
            if hasattr(loc, 'value'):
                byte_offset = loc.value
                # Handle DWARF expressions (simplified)
                if isinstance(byte_offset, list):
                    # Usually DW_OP_plus_uconst
                    byte_offset = byte_offset[0] if byte_offset else 0
                # Ensure it's an int
                if byte_offset is None:
                    byte_offset = 0

        # Get bit offset/size for bitfields
        bit_offset = self.get_attr_value(die, 'DW_AT_bit_offset')
        bit_size = self.get_attr_value(die, 'DW_AT_bit_size')
        data_bit_offset = self.get_attr_value(die, 'DW_AT_data_bit_offset')

        # Get type
        type_die, type_cu = self.get_type_die(die, cu)
        type_info = self.resolve_type(type_die, type_cu, depth)

        field = {
            "name": name,
            "byte_offset": byte_offset,
            "type_info": type_info
        }

        # Add bitfield info if present
        if bit_size is not None:
            field["bit_size"] = bit_size
            if data_bit_offset is not None:
                field["bit_offset"] = data_bit_offset
            elif bit_offset is not None:
                # Convert from big-endian bit numbering
                byte_size = type_info.get("size", 4)
                field["bit_offset"] = (byte_size * 8) - bit_offset - bit_size

        return field


def compute_dartt_offsets(type_info, base_byte_offset=0, unaligned_fields=None):
    """
    Walk the type tree and add absolute dartt_offset to each field.
    Also collects any unaligned fields for reporting.

    All dartt_offset values are ABSOLUTE - referenced to the top-level struct base address.
    This matches how DARTT protocol addresses memory.

    Args:
        type_info: The type dictionary from resolve_type()
        base_byte_offset: Cumulative byte offset from TOP-LEVEL struct base
        unaligned_fields: List to collect unaligned field info (mutated)

    Returns:
        List of unaligned fields found
    """
    if unaligned_fields is None:
        unaligned_fields = []

    if type_info.get("type") in ("struct", "union"):
        for field in type_info.get("fields", []):
            # byte_offset starts as relative to parent, compute absolute from struct base
            relative_byte_offset = field.get("byte_offset") or 0
            absolute_byte_offset = base_byte_offset + relative_byte_offset

            # Store ABSOLUTE byte_offset (overwrite the relative value)
            field["byte_offset"] = absolute_byte_offset

            # Compute dartt_offset - ABSOLUTE 32-bit word index from struct base
            field["dartt_offset"] = absolute_byte_offset // 4

            # Flag unaligned fields
            if absolute_byte_offset % 4 != 0:
                field["unaligned"] = True
                unaligned_fields.append({
                    "name": field.get("name"),
                    "absolute_byte_offset": absolute_byte_offset,
                    "remainder": absolute_byte_offset % 4
                })

            # Recurse into nested types - pass the ABSOLUTE offset as the new base
            # IMPORTANT: deep copy the type_info to avoid shared references
            # (same struct type used at different offsets would otherwise share the dict)
            field_type = field.get("type_info", {})
            if field_type.get("type") in ("struct", "union"):
                # Make an independent copy for this specific field instance
                field["type_info"] = copy.deepcopy(field_type)
                compute_dartt_offsets(field["type_info"], absolute_byte_offset, unaligned_fields)
            elif field_type.get("type") == "array":
                elem_type = field_type.get("element_type", {})
                if elem_type.get("type") in ("struct", "union"):
                    # For arrays of structs, compute offsets for element type template
                    # Element 0 starts at the array's absolute offset
                    field["type_info"] = copy.deepcopy(field_type)
                    compute_dartt_offsets(field["type_info"]["element_type"], absolute_byte_offset, unaligned_fields)

    return unaligned_fields


def flatten_fields(type_info, prefix="", base_offset=0):
    """
    Flatten nested struct fields into a flat list with full paths.
    This makes it easier for the dashboard to address individual fields.

    Output uses DARTT protocol conventions:
    - dartt_offset: 32-bit word index (byte_offset / 4)
    - nbytes: size in bytes
    """
    fields = []

    if type_info.get("type") == "struct" or type_info.get("type") == "union":
        for field in type_info.get("fields", []):
            field_name = field.get("name", "")
            field_byte_offset = (field.get("byte_offset") or 0) + base_offset
            field_type = field.get("type_info", {})

            full_name = f"{prefix}.{field_name}" if prefix else field_name

            # If this field is itself a struct, recurse
            if field_type.get("type") in ("struct", "union"):
                fields.extend(flatten_fields(field_type, full_name, field_byte_offset))
            elif field_type.get("type") == "array":
                elem_type = field_type.get("element_type", {})
                elem_size = elem_type.get("size", 0)
                total = field_type.get("total_elements", 0)

                # If array of structs, expand each element
                if elem_type.get("type") in ("struct", "union"):
                    for i in range(total):
                        elem_name = f"{full_name}[{i}]"
                        elem_byte_offset = field_byte_offset + (i * elem_size)
                        fields.extend(flatten_fields(elem_type, elem_name, elem_byte_offset))
                else:
                    # Array of primitives - add as single field with array info
                    entry = {
                        "name": full_name,
                        "dartt_offset": field_byte_offset // 4,
                        "nbytes": field_type.get("size", 0),
                        "type": get_simple_type_name(elem_type),
                        "array_size": total,
                        "element_nbytes": elem_size
                    }
                    # Flag if not 32-bit aligned
                    if field_byte_offset % 4 != 0:
                        entry["unaligned"] = True
                        entry["byte_offset"] = field_byte_offset
                    fields.append(entry)
            else:
                # Primitive field
                entry = {
                    "name": full_name,
                    "dartt_offset": field_byte_offset // 4,
                    "nbytes": field_type.get("size", 0),
                    "type": get_simple_type_name(field_type)
                }

                # Flag if not 32-bit aligned
                if field_byte_offset % 4 != 0:
                    entry["unaligned"] = True
                    entry["byte_offset"] = field_byte_offset

                # Add bitfield info if present
                if "bit_size" in field:
                    entry["bit_size"] = field["bit_size"]
                    entry["bit_offset"] = field.get("bit_offset", 0)

                fields.append(entry)

    return fields


def get_simple_type_name(type_info):
    """Get a simple type name string from type info."""
    if "typedef" in type_info:
        return type_info["typedef"]

    t = type_info.get("type", "unknown")

    if t == "pointer":
        pointee = type_info.get("pointee", {})
        return f"{get_simple_type_name(pointee)}*"
    elif t == "array":
        elem = type_info.get("element_type", {})
        dims = type_info.get("dimensions", [])
        dim_str = "".join(f"[{d}]" for d in dims)
        return f"{get_simple_type_name(elem)}{dim_str}"
    elif t in ("struct", "union"):
        name = type_info.get("struct_name") or type_info.get("union_name")
        return f"{t} {name}" if name else t
    elif t == "enum":
        name = type_info.get("enum_name")
        return f"enum {name}" if name else "enum"
    else:
        return type_info.get("type", "unknown")


def main():
    parser = argparse.ArgumentParser(
        description="Extract struct layout from ELF DWARF debug info for DARTT dashboard",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python dartt-describe.py firmware.elf gl_dp
  python dartt-describe.py firmware.elf motor_config -o motor_config.json
  python dartt-describe.py firmware.elf gl_dp --flat
        """
    )

    parser.add_argument("elf_file", help="Path to ELF file with debug info")
    parser.add_argument("symbol", help="Name of global symbol to describe")
    parser.add_argument("-o", "--output", help="Output JSON file (default: stdout)")
    parser.add_argument("--flat", action="store_true",
                        help="Flatten nested structs into dot-notation paths")
    parser.add_argument("--pretty", action="store_true", default=True,
                        help="Pretty-print JSON output (default: true)")
    parser.add_argument("--compact", action="store_true",
                        help="Compact JSON output (no indentation)")

    args = parser.parse_args()

    # Open and parse ELF file
    elf_path = Path(args.elf_file)
    if not elf_path.exists():
        print(f"Error: ELF file not found: {elf_path}", file=sys.stderr)
        sys.exit(1)

    with open(elf_path, 'rb') as f:
        elffile = ELFFile(f)

        # Check for debug info
        if not elffile.has_dwarf_info():
            print("Error: ELF file has no DWARF debug info. Compile with -g", file=sys.stderr)
            sys.exit(1)

        parser_obj = DWARFStructParser(elffile)

        # Get symbol address
        symbol_addr = parser_obj.get_symbol_address(args.symbol)
        symbol_size = parser_obj.get_symbol_size(args.symbol)

        if symbol_addr is None:
            print(f"Error: Symbol '{args.symbol}' not found in symbol table", file=sys.stderr)
            sys.exit(1)

        # Find variable DIE
        var_die, var_cu = parser_obj.find_variable_die(args.symbol)

        if var_die is None:
            print(f"Error: Symbol '{args.symbol}' not found in DWARF debug info", file=sys.stderr)
            print("Note: The symbol exists but has no debug info. Ensure it's not optimized out.",
                  file=sys.stderr)
            sys.exit(1)

        # Get type information
        type_die, type_cu = parser_obj.get_type_die(var_die, var_cu)
        type_info_cached = parser_obj.resolve_type(type_die, type_cu)

        # Deep copy to avoid modifying cached type_info (shared across same-typed fields)
        type_info = copy.deepcopy(type_info_cached)

        # Compute absolute dartt_offset for all fields in the hierarchy
        unaligned_fields = compute_dartt_offsets(type_info)

        # Warn about unaligned fields
        if unaligned_fields:
            print(f"Warning: {len(unaligned_fields)} field(s) are not 32-bit aligned!", file=sys.stderr)
            print("DARTT protocol requires 32-bit aligned access. Unaligned fields:", file=sys.stderr)
            for f in unaligned_fields:
                byte_off = f.get("absolute_byte_offset", 0)
                print(f"  - {f['name']}: absolute_byte_offset={byte_off} (0x{byte_off:X}), "
                      f"remainder={f['remainder']}", file=sys.stderr)
            print("\nConsider restructuring your typedef to ensure 32-bit alignment.", file=sys.stderr)
            print("Hint: Group smaller types (uint8_t, uint16_t) together, or add explicit padding.\n",
                  file=sys.stderr)

        # Build output structure
        total_nbytes = symbol_size or type_info.get("size", 0)
        output = {
            "symbol": args.symbol,
            "address": f"0x{symbol_addr:08X}",
            "address_int": symbol_addr,
            "nbytes": total_nbytes,
            "nwords": (total_nbytes + 3) // 4,  # 32-bit words (rounded up)
            "type": type_info
        }

        # Add flattened fields if requested (for backwards compatibility)
        # Use the original cached type_info (with relative byte_offsets) since
        # flatten_fields computes absolute offsets itself
        if args.flat:
            output["flat_fields"] = flatten_fields(type_info_cached)

    # Output JSON
    indent = None if args.compact else 2
    json_output = json.dumps(output, indent=indent)

    if args.output:
        with open(args.output, 'w') as f:
            f.write(json_output)
            f.write('\n')
        print(f"Wrote {args.output}", file=sys.stderr)
    else:
        print(json_output)


if __name__ == "__main__":
    main()
