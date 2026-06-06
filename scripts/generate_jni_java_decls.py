import pathlib
import re


ROOT = pathlib.Path(__file__).resolve().parents[1]
JNI = ROOT / "src" / "abi" / "jni.rs"

TYPE_MAP = {
    "long": "long",
    "boolean": "boolean",
    "double": "double",
    "int": "int",
    "void": "void",
}


def camel(name: str) -> str:
    parts = name.split("_")
    return parts[0] + "".join(part[:1].upper() + part[1:] for part in parts[1:])


def main() -> None:
    jni = JNI.read_text(encoding="utf-8")
    for ret, name, args in re.findall(r"jni!\((\w+)\s+(\w+)\(([^)]*)\)", jni):
        params = []
        args = args.strip()
        if args:
            for arg in args.split(","):
                kind, arg_name = arg.strip().split()
                params.append(f"{TYPE_MAP[kind]} {camel(arg_name)}")
        print(f"    public static native {TYPE_MAP[ret]} {name}({', '.join(params)});")


if __name__ == "__main__":
    main()
